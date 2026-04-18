![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Ring Oscillator with PVT Sensor

Three selectable ring oscillators (7, 11, and 15 inverter stages) with a 16-bit frequency counter for process, voltage, and temperature (PVT) characterization on the SkyWater 130 nm (sky130A) process node.

- [Detailed documentation](docs/info.md)

## Architecture

The design instantiates three ring oscillators using `sky130_fd_sc_hd` standard cells (NAND2 + inverter chains). A 2-stage synchronizer brings the selected RO output into the system clock domain, where a 16-bit counter accumulates rising edges during a user-defined gate time. The latched count is read out as two bytes via `byte_sel`.

Individual oscillators can be power-gated via `ui_in[7:5]` for selective current measurement.

## Pin Summary

| Pin | Direction | Function |
|-----|-----------|----------|
| `ui_in[1:0]` | Input | `ro_sel` (00=7-stage, 01=11-stage, 10=15-stage) |
| `ui_in[2]` | Input | `cnt_enable` |
| `ui_in[3]` | Input | `cnt_clear` (latch and reset counter) |
| `ui_in[4]` | Input | `byte_sel` (0=low byte, 1=high byte) |
| `ui_in[5]` | Input | `ro7_dis` (disable 7-stage RO) |
| `ui_in[6]` | Input | `ro11_dis` (disable 11-stage RO) |
| `ui_in[7]` | Input | `ro15_dis` (disable 15-stage RO) |
| `uo_out[7:0]` | Output | `freq_count` (selected byte of 16-bit counter) |
| `uio_out[0]` | Output | Overflow flag |
| `uio_out[1]` | Output | Raw RO output |
| `uio_out[2]` | Output | Synchronized RO output |

## Simulation

```bash
cd test
make
```

Requires cocotb and Icarus Verilog. RTL simulation uses clock-divider models (`ifdef SIM`) in place of the physical ring oscillators.

## Target

Tiny Tapeout [TTSKY26a](https://tinytapeout.com) shuttle, 1x1 tile, SkyWater 130 nm.
