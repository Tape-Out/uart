"""uart 的自环测试台：把 txd 接回 rxd，发几个字节再收回来对。

寄存器一致性验的是生成的那部分，这一份验的是 IP 自己的逻辑——波特率分频、
起始位检测、移位方向、奇偶校验、FIFO。
"""
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)

BYTES = [0x41, 0x00, 0xFF, 0x5A]
DIV = 3       # 分频取小，仿真快；协议逻辑与分频大小无关

lst = "\n".join(f"      {i}: return 8'h{b:02X};" for i, b in enumerate(BYTES))

(out / "UartTb.bsv").write_text(f'''package UartTb;

import RegIf::*;
import Uart::*;

// 由 tb/mkuarttb.py 生成，勿手改。

Integer nbytes = {len(BYTES)};

function Bit#(8) want(Bit#(8) i);
  case (i)
{lst}
    default: return 0;
  endcase
endfunction

// 寄存器偏移，与 regmap.yaml 一致
Bit#(8) rTXDATA = 8'h00;
Bit#(8) rRXDATA = 8'h04;
Bit#(8) rTXCTRL = 8'h08;
Bit#(8) rRXCTRL = 8'h0C;
Bit#(8) rDIV    = 8'h18;

typedef enum {{ Init, Send, Recv, Done }} Phase deriving (Bits, Eq);

(* synthesize *)
module mkUartTb(Empty);
  UartIfc#(8, 32, 8) u <- mkUart(UartCfg {{ parity: True, flowctrl: False }});

  Reg#(Phase)    ph   <- mkReg(Init);
  Reg#(Bit#(8))  init <- mkReg(0);
  Reg#(Bit#(8))  sent <- mkReg(0);
  Reg#(Bit#(8))  got  <- mkReg(0);
  Reg#(Bit#(32)) cyc  <- mkReg(0);
  Reg#(Bool)     bad  <- mkReg(False);
  Reg#(Bit#(1))  line <- mkReg(1);

  // 自环：发出去的就是收到的
  rule loop;
    line <= u.pins.txd;
    u.pins.rxd(line);
    u.pins.cts(0);
  endrule

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 200000) begin
      $display("TIMEOUT after %0d cycles", cyc);
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) d) = action
    let _ <- u.regs.access(RegReq {{ addr: a, write: True,
                                     wdata: d, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Init);
    case (init)
      0: wr(rDIV, {DIV});
      1: wr(rTXCTRL, 32'h1);       // txen
      2: wr(rRXCTRL, 32'h1);       // rxen
      default: ph <= Send;
    endcase
    init <= init + 1;
  endrule

  // 一次塞一个字节，塞满 FIFO 就等
  rule send (ph == Send && sent < fromInteger(nbytes));
    wr(rTXDATA, zeroExtend(want(sent)));
    sent <= sent + 1;
  endrule

  rule sendDone (ph == Send && sent == fromInteger(nbytes));
    ph <= Recv;
  endrule

  rule recv (ph == Recv);
    let x <- u.regs.access(RegReq {{ addr: rRXDATA, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    // 位 31 为一表示队列空，读到的数据无效
    if (x.rdata[31] == 0) begin
      if (x.rdata[7:0] != want(got)) begin
        $display("FAIL byte %0d: got %02h want %02h",
                 got, x.rdata[7:0], want(got));
        bad <= True;
      end
      got <= got + 1;
      if (got + 1 == fromInteger(nbytes)) ph <= Done;
    end
  endrule

  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS uart loopback, %0d bytes", nbytes);
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
''', encoding="utf-8")
print(f"  uart 自环：{len(BYTES)} 个字节")
