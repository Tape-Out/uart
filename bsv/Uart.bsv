package Uart;

import FIFOF::*;
import Apb4::*;
import UartRegs::*;

typedef struct {
  Bool parity;
  Bool flowctrl;
} UartCfg;

interface UartPins;
  (* always_ready, result = "txd" *) method Bit#(1) txd;
  (* always_ready, always_enabled, prefix = "" *)
  method Action rxd((* port = "rxd" *) Bit#(1) v);
  (* always_ready, result = "irq" *) method Bool irq;
endinterface

interface UartIfc#(numeric type aw, numeric type dw, numeric type fifoDepth);
  interface Apb4SlavePins#(aw, dw) apb;
  interface UartPins                pins;
endinterface

module mkUart#(UartCfg cfg)(UartIfc#(aw, dw, fifoDepth))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw),
              Add#(_b, 1, dw), Add#(_c, 8, dw), Add#(_d, 3, dw), Add#(_e, 16, dw));

  UartRegsIfc#(aw, dw, fifoDepth) r <- mkUartRegs;

  // 去守卫按端口给，不一刀切：软件那一端走 always_ready 的总线方法，必须无守卫；
  // 硬件那一端走规则，带守卫才对——一刀切 mkUG* 会让规则去 deq 空队列。
  FIFOF#(Bit#(8)) txq <- mkGSizedFIFOF(True,  False, valueOf(fifoDepth));
  FIFOF#(Bit#(8)) rxq <- mkGSizedFIFOF(False, True,  valueOf(fifoDepth));

  Reg#(Bit#(16)) txDiv  <- mkReg(0);
  Reg#(Bit#(4))  txBit  <- mkReg(0);      // 0 空闲，1 起始，2..9 数据，10.. 停止
  Reg#(Bit#(10)) txSh   <- mkReg(10'h3FF);
  Reg#(Bool)     txPend <- mkReg(False);

  Reg#(Bit#(16)) rxDiv  <- mkReg(0);
  Reg#(Bit#(4))  rxBit  <- mkReg(0);
  Reg#(Bit#(8))  rxSh   <- mkReg(0);
  Wire#(Bit#(1)) rxLine <- mkBypassWire;
  Reg#(Bit#(1))  rxSync <- mkReg(1);

  // swmod 的脉冲与寄存器的新值差一拍：脉冲在写的当拍发出，寄存器下一拍才有新值。
  // 两件事必须分成两条规则——读脉冲逼着排在总线方法之后，读寄存器逼着排在之前，
  // 写一条里就是 G0021 自相矛盾。拆开之后次序是「入队 -> 总线方法 -> 记脉冲」。
  rule txEnq (txPend && txq.notFull);
    txq.enq(r.txdata_data);
  endrule

  rule txMark;
    txPend <= r.txdata_data_wr;
  endrule

  rule txRun (r.txctrl_txen == 1);
    if (txBit == 0) begin
      if (txq.notEmpty) begin
        txSh  <= {1'b1, txq.first, 1'b0};   // 停止 + 数据 + 起始，低位先出
        txq.deq;
        txBit <= 1;
        txDiv <= r.div;
      end
    end else if (txDiv == 0) begin
      txSh  <= {1'b1, txSh[9:1]};
      txDiv <= r.div;
      txBit <= (txBit == (r.txctrl_nstop == 1 ? 11 : 10)) ? 0 : txBit + 1;
    end else
      txDiv <= txDiv - 1;
  endrule

  rule rxSample;
    rxSync <= rxLine;
    if (rxBit == 0) begin
      // 起始位下降沿：半个位时间之后落在位中央
      if (rxSync == 1 && rxLine == 0) begin
        rxBit <= 1;
        rxDiv <= r.div >> 1;
      end
    end else if (rxDiv == 0) begin
      rxDiv <= r.div;
      if (rxBit >= 2 && rxBit <= 9)
        rxSh <= {rxLine, rxSh[7:1]};
      if (rxBit == 10) begin
        if (rxq.notFull) rxq.enq(rxSh);
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
  endrule

  // swacc：软件读过 rxdata 就弹一个。这一条同拍成立，因为 deq 是动作不是取值。
  rule rxPop (r.rxdata_data_rd && rxq.notEmpty);
    rxq.deq;
  endrule

  Apb4SlavePins#(aw, dw) sl <- mkApb4Slave(r.regs);

  interface apb = sl;
  interface UartPins pins;
    method Bit#(1) txd = (txBit == 0) ? 1 : txSh[0];
    method Action rxd(Bit#(1) v); rxLine <= v; endmethod
    // volatile 字段是单向的：硬件驱动、软件只读，硬件不该读回自己驱动的值。
    // 中断直接从 FIFO 状态算，与喂给 ip 寄存器的是同一个表达式。
    method Bool irq = ((r.ie_txwm == 1) && txq.notFull) || ((r.ie_rxwm == 1) && rxq.notEmpty);
  endinterface
endmodule

endpackage
