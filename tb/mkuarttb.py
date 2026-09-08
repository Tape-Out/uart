"""uart 的自环测试台：把 txd 接回 rxd，发几个字节收回来对，并验接收水线。

寄存器一致性验的是生成的那部分，这一份验 IP 自己的逻辑——波特率分频、起始位
检测、移位方向、奇偶校验、FIFO，以及水线门限。

水线那一段是**决定性**的：先只发到刚好等于门限的数量，等足够久，标志必须还是
零；再补一个，标志才该抬起来。原来的实现只看「队列非空」，第一步就会露馅。

认矩阵：`fifoDepth`、`parity`、`flowctrl` 都从这一点的旋钮来。深度只有 1 的时候
门限取 0——队列装不下第二个，门限 1 永远够不着。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
depth = int(k.get("fifoDepth", 8))
parity = bool(k.get("parity", False))
flowctrl = bool(k.get("flowctrl", False))

BYTES = [0x41, 0x00, 0xFF, 0x5A]
DIV = 3           # 分频取小，仿真快；协议逻辑与分频大小无关
WM = 1 if depth >= 2 else 0      # 接收水线门限
QUIET = 400       # 等一个字节走完绰绰有余：十位 × 四拍
# 一次最多发到队列装得下的量：这一台是「先全发完再全读出」，
# 发多了接收队列会溢出丢字节，那测的是溢出不是回环。溢出本身没有状态位
# 可查（寄存器图里没有 overrun），已记进任务单。
NSEND = min(len(BYTES), depth)

lst = chr(10).join(f"      {i}: return 8'h{b:02X};" for i, b in enumerate(BYTES))

txt = f'''package Uart{label}Tb;

import RegIf::*;
import Uart::*;

// 由 tb/mkuarttb.py 生成，勿手改。
// 这一点：fifoDepth={depth} parity={parity} flowctrl={flowctrl} 水线={WM}

Integer nbytes = {NSEND};
Integer wm = {WM};

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
Bit#(8) rIE     = 8'h10;
Bit#(8) rIP     = 8'h14;
Bit#(8) rDIV    = 8'h18;

typedef enum {{ Init, SendLow, Quiet, SendRest, WaitWm, Drain, Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkUart{label}Tb(Empty);
  UartIfc#(8, 32, {depth}) u <- mkUart(
      UartCfg {{ parity: {"True" if parity else "False"},
                 flowctrl: {"True" if flowctrl else "False"} }});

  Reg#(Phase)    ph   <- mkReg(Init);
  Reg#(Bit#(16)) s    <- mkReg(0);
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
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) d) = action
    let _ <- u.regs.access(RegReq {{ addr: a, write: True,
                                     wdata: d, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Init);
    case (s)
      0: wr(rDIV, {DIV});
      1: wr(rTXCTRL, 32'h1);                    // txen
      2: wr(rRXCTRL, 32'h1 | ({WM} << 16));     // rxen + 接收水线门限
      3: wr(rIE, 32'h2);                        // 只开接收水线中断
      default: ph <= SendLow;
    endcase
    if (s < 4) s <= s + 1; else s <= 0;
  endrule

  // 先只发到刚好等于门限的数量
  rule sendLow (ph == SendLow);
    if (sent < fromInteger(wm)) begin
      wr(rTXDATA, zeroExtend(want(sent)));
      sent <= sent + 1;
    end else ph <= Quiet;
  endrule

  // 等足够久，标志必须还是零：只看「队列非空」的实现在这里就会露馅
  rule quiet (ph == Quiet);
    if (s > {QUIET}) begin
      ph <= SendRest;
      s  <= 0;
    end else s <= s + 1;
  endrule

  rule quietCheck (ph == Quiet && s == {QUIET});
    let x <- u.regs.access(RegReq {{ addr: rIP, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[1] == 1 || u.irq) begin
      $display("FAIL the receive watermark fired at or below the threshold");
      bad <= True;
    end
  endrule

  rule sendRest (ph == SendRest);
    if (sent < fromInteger(nbytes)) begin
      wr(rTXDATA, zeroExtend(want(sent)));
      sent <= sent + 1;
    end else ph <= WaitWm;
  endrule

  // 补上之后标志才该抬起来
  rule waitWm (ph == WaitWm);
    let x <- u.regs.access(RegReq {{ addr: rIP, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    if (x.rdata[1] == 1) begin ph <= Drain; s <= 0; end
    else if (s > 50000) begin      // s 是 16 位，别写超过它装得下的数
      $display("FAIL the receive watermark never fired above the threshold");
      bad <= True;
      ph <= Done;
    end else s <= s + 1;
  endrule

  // 收回来的字节要逐个对上，顺序也要对
  rule drain (ph == Drain);
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
    else $display("PASS uart: %0d bytes loop back, and the receive watermark "
                  + "waits for the threshold", nbytes);
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

(out / f"Uart{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  uart 自环 {NSEND} 字节：fifoDepth={depth} parity={parity} "
      f"flowctrl={flowctrl} 水线={WM}")
