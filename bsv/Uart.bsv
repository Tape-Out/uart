package Uart;

import FIFOF::*;
import RegIf::*;
import UartRegs::*;

// 恒零的只读寄存器：特性关掉时占位，写进去什么也不发生，综合器整片消掉
function Reg#(t) roReg(t v) =
  interface Reg;
    method t _read = v;
    method Action _write(t x) = noAction;
  endinterface;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool parity;
  Bool flowctrl;
} UartCfg;

// cts/rts 无论开不开流控都在接口上。类型不能随配置变形，而关掉时它们退化成
// 一个常量输出与一个没人读的输入，综合器会整片消掉，不额外花钱。
interface UartPins;
  (* always_ready, result = "txd" *) method Bit#(1) txd;
  (* always_ready, always_enabled, prefix = "" *)
  method Action rxd((* port = "rxd" *) Bit#(1) v);
  (* always_ready, always_enabled, prefix = "" *)
  method Action cts((* port = "cts_n" *) Bit#(1) v);
  (* always_ready, result = "rts_n" *) method Bit#(1) rts;
endinterface

interface UartIfc#(numeric type aw, numeric type dw, numeric type fifoDepth);
  interface RegIf#(aw, dw) regs;
  interface UartPins       pins;
  (* always_ready *) method Bool irq;
endinterface

module mkUart#(UartCfg cfg)(UartIfc#(aw, dw, fifoDepth))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw),
              Add#(_b, 1, dw), Add#(_c, 8, dw), Add#(_d, 3, dw), Add#(_e, 16, dw));

  UartRegsIfc#(aw, dw, fifoDepth) r <- mkUartRegs(
      UartRegsCfg { parity: cfg.parity, flowctrl: cfg.flowctrl });

  // 去守卫按端口给，不一刀切：软件那一端走 always_ready 的总线方法，必须无守卫；
  // 硬件那一端走规则，带守卫才对——一刀切 mkUG* 会让规则去 deq 空队列。
  FIFOF#(Bit#(8)) txq <- mkGSizedFIFOF(True,  False, valueOf(fifoDepth));
  FIFOF#(Bit#(8)) rxq <- mkGSizedFIFOF(False, True,  valueOf(fifoDepth));

  Reg#(Bit#(16)) txDiv  <- mkReg(0);
  Reg#(Bit#(4))  txBit  <- mkReg(0);      // 0 空闲，1 起始，2..9 数据，之后校验与停止
  Reg#(Bit#(12)) txSh   <- mkReg(12'hFFF);
  Reg#(Bool)     txPend <- mkReg(False);

  Reg#(Bit#(16)) rxDiv  <- mkReg(0);
  Reg#(Bit#(4))  rxBit  <- mkReg(0);
  Reg#(Bit#(8))  rxSh   <- mkReg(0);
  // 关掉校验就不例化这两个。只挡逻辑不挡例化的开关一分钱都不省。
  Reg#(Bit#(1))  rxPar = roReg(0);
  Reg#(Bit#(1))  rxErr = roReg(0);
  if (cfg.parity) begin
    rxPar <- mkReg(0);
    rxErr <- mkReg(0);
  end
  Wire#(Bit#(1)) rxLine <- mkBypassWire;
  Wire#(Bit#(1)) ctsLine <- mkBypassWire;
  Reg#(Bit#(1))  rxSync <- mkReg(1);

  Bit#(1) parEn  = cfg.parity   ? r.txctrl_pariten : 0;
  Bit#(1) parSel = cfg.parity   ? r.txctrl_paritysel : 0;
  Bool    txOk   = !cfg.flowctrl || r.txctrl_ctsen == 0 || ctsLine == 0;

  // 一帧几拍：起始 + 8 数据 + 可选校验 + 一或两个停止
  Bit#(4) txLast = 10 + zeroExtend(r.txctrl_nstop) + zeroExtend(parEn);

  // swmod 的脉冲与寄存器的新值差一拍：脉冲在写的当拍发出，寄存器下一拍才有新值。
  // 两件事必须分成两条规则——读脉冲逼着排在总线方法之后，读寄存器逼着排在之前，
  // 写一条里就是 G0021 自相矛盾。拆开之后次序是「入队 -> 总线方法 -> 记脉冲」。
  rule txEnq (txPend && txq.notFull);
    txq.enq(r.txdata_data);
  endrule

  rule txMark;
    txPend <= r.txdata_data_wr;
  endrule

  // 取数单列一条规则。写在下面那条的分支里的话，`first`/`deq` 的隐式条件会被
  // **提升到整条规则**——队列一空，连移位都停了，最后一个字节发一半就断在线上。
  // 自环测试逮到的就是这个：前三个字节好好的，第四个收回来是全零。
  rule txLoad (r.txctrl_txen == 1 && txBit == 0 && txOk);
    Bit#(8) d = txq.first;
    Bit#(1) p = (parSel == 1) ? ~(^d) : (^d);   // 0 偶校验，1 奇校验
    // 低位先出：起始 + 数据 + 校验(或第一个停止) + 停止
    txSh  <= {2'b11, (parEn == 1) ? p : 1'b1, d, 1'b0};
    txq.deq;
    txBit <= 1;
    txDiv <= r.div;
  endrule

  rule txShift (r.txctrl_txen == 1 && txBit != 0);
    if (txDiv == 0) begin
      txSh  <= {1'b1, txSh[11:1]};
      txDiv <= r.div;
      txBit <= (txBit == txLast) ? 0 : txBit + 1;
    end else
      txDiv <= txDiv - 1;
  endrule

  rule rxSample (r.rxctrl_rxen == 1);
    rxSync <= rxLine;
    if (rxBit == 0) begin
      // 起始位下降沿：半个位时间之后落在位中央
      if (rxSync == 1 && rxLine == 0) begin
        rxBit <= 1;
        rxDiv <= r.div >> 1;
        rxErr <= 0;
      end
    end else if (rxDiv == 0) begin
      rxDiv <= r.div;
      if (rxBit >= 2 && rxBit <= 9)
        rxSh <= {rxLine, rxSh[7:1]};
      if (cfg.parity && parEn == 1 && rxBit == 10)
        rxPar <= rxLine;
      Bit#(4) last = (cfg.parity && parEn == 1) ? 11 : 10;
      if (rxBit == last) begin
        Bit#(1) want = (parSel == 1) ? ~(^rxSh) : (^rxSh);
        if (cfg.parity && parEn == 1 && rxPar != want) rxErr <= 1;
        else if (rxq.notFull) rxq.enq(rxSh);
        rxBit <= 0;
      end else
        rxBit <= rxBit + 1;
    end else
      rxDiv <= rxDiv - 1;
  endrule

  // volatile 字段：没有存储，硬件每拍驱动
  rule status;
    r.txdata_full_in(txq.notFull ? 0 : 1);
    r.rxdata_data_in(rxq.first);
    r.rxdata_empty_in(rxq.notEmpty ? 0 : 1);
    r.ip_txwm_in(txq.notFull ? 1 : 0);
    r.ip_rxwm_in(rxq.notEmpty ? 1 : 0);
    if (cfg.parity) r.rxdata_parerr_in(rxErr);
  endrule

  // swacc：软件读过 rxdata 就弹一个。这一条同拍成立，因为 deq 是动作不是取值。
  rule rxPop (r.rxdata_data_rd && rxq.notEmpty);
    rxq.deq;
  endrule

  interface regs = r.regs;
  interface UartPins pins;
    method Bit#(1) txd = (txBit == 0) ? 1 : txSh[0];
    method Action rxd(Bit#(1) v); rxLine <= v; endmethod
    method Action cts(Bit#(1) v); ctsLine <= v; endmethod
    // 低有效：队列还收得下就拉低，请对端继续发
    method Bit#(1) rts = (cfg.flowctrl && rxq.notFull) ? 0 : 1;
  endinterface
  // volatile 字段是单向的：硬件驱动、软件只读，硬件不该读回自己驱动的值。
  // 中断直接从 FIFO 状态算，与喂给 ip 寄存器的是同一个表达式。
  method Bool irq = ((r.ie_txwm == 1) && txq.notFull) || ((r.ie_rxwm == 1) && rxq.notEmpty);
endmodule

endpackage
