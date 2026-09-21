# uart

UART with a 16550-compatible register profile and a native profile.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0%20OR%20MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. Written in Bluespec against the [`spec`](https://github.com/Tape-Out/xrspec)
contracts, with the register file generated from `regmap.yaml`. Three FIFO depths
synthesise clean with no scheduling warnings; the area curve is measured at six depths
and four further depths confirm the prediction never comes in low.

| FIFO depth | 1 | 2 | 4 | 8 | 16 | 32 |
| :--: | --: | --: | --: | --: | --: | --: |
| Area, um2 | 1689 | 1898 | 2533 | 3376 | 5001 | 8063 |

## Notes

目前使用[UC Agent](https://open-verify.cc/mlvp/docs/ucagent/introduce/)测试

> picker export uart_mmio.v --rw 1 --sname uart_mmio --tdir output/ -c -w output/uart_mmio.fst

> ucagent output/ uart_mmio -s -hm --tui --mcp-server-no-file-tools --no-embed-tools

> verilator --lint-only -Wall --top-module uart_mmio uart_mmio.v

## License

任选其一：

- [MIT](LICENSE-MIT)
- [Apache 2.0](LICENSE-APACHE)
- [木兰宽松许可证 第2版](LICENSE-MULAN)

`SPDX-License-Identifier: MIT OR Apache-2.0 OR MulanPSL-2.0`

除非另行说明，你提交的贡献按上述三者同时授权，不附加其他条件。
