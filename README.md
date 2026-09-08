# uart

UART with a 16550-compatible register profile and a native profile.

![maturity](https://img.shields.io/badge/maturity-planned-lightgrey) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Planned. What sits in this repository today is the retired picorv32-era Verilog,
kept for provenance; the Bluespec rewrite has not landed yet.

## Notes

目前使用[UC Agent](https://open-verify.cc/mlvp/docs/ucagent/introduce/)测试

> picker export uart_mmio.v --rw 1 --sname uart_mmio --tdir output/ -c -w output/uart_mmio.fst

> ucagent output/ uart_mmio -s -hm --tui --mcp-server-no-file-tools --no-embed-tools

> verilator --lint-only -Wall --top-module uart_mmio uart_mmio.v

## License

Mulan PSL v2.
