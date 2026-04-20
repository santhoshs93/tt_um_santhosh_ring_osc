<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This design implements **five gatable ring oscillators** (7, 11, 15, 21, and 31 sky130_fd_sc_hd inverter-chain stages, each with a NAND2 enable gate) together with a 16-bit frequency counter, a three-stage CDC synchronizer, an auto gate timer, a configurable prescaler, an XOR-jitter true random number generator (TRNG), a frequency-bounds health monitor, and a differential (beat-frequency) measurement mode. It is intended for silicon PVT characterization, on-chip entropy, and fault/aging detection.

The block supports two control modes, selected by `ui_in[6]` (`spi_mode`):

- **Parallel mode (`spi_mode=0`)** — `ro_sel[2:0]` (`ui_in[2:0]`) chooses one of the five ROs, `cnt_enable` (`ui_in[3]`) starts/stops counting, a rising edge on `cnt_clear_latch` (`ui_in[4]`) latches the 16-bit count and clears the accumulator, and `byte_sel` (`ui_in[5]`) multiplexes `uo_out` between the low (0) and high (1) bytes of the latched count. `uio_out[4]` is the overflow flag, `uio_out[5]` exposes the raw RO output, `uio_out[6]` the synchronized RO output, and `uio_out[7]` is `meas_done` from the auto gate timer.
- **SPI mode (`spi_mode=1`)** — a 16-bit SPI slave (Mode 0, MSB first; CS on `uio_in[0]`, MOSI on `uio_in[1]`, MISO on `uio_out[2]`, SCK on `uio_in[3]`) exposes a register file: `reg_ctrl` (0x00, `[0]=auto_gate_start` self-clearing, `[1]=clear_meas_done`), `reg_ro_sel` (0x01, 3-bit), `reg_ro_en` (0x02, 5-bit per-RO enable — reset default `0x1F`), `reg_gate_l/h` (0x03/0x04, 16-bit auto-gate count), `reg_prescale` (0x05, `[1:0]` selecting ÷8/÷16/÷32/÷64; reset default ÷16), `reg_status` (0x06, read-only: `[0]=gate_active`, `[1]=overflow`, `[2]=meas_done`), `reg_count_l/h` (0x07/0x08, read-only latched count), `reg_trng_ctrl` (0x09, `[0]=trng_en`, `[1]=health_en`, `[2]=diff_mode`), `reg_diff_sel` (0x0A, 3-bit RO_B selector for beat-frequency/XOR), `reg_health_lo/hi` (0x0B/0x0C, bounds), `reg_trng_data` (0x0D, read-only), `reg_health_status` (0x0E, read-only).

The raw RO is synchronized into the system clock domain through a 3-stage flip-flop synchronizer (with an optional ÷8/÷16/÷32/÷64 prescaler ahead of it) before being rising-edge detected and counted. When `diff_mode` is set, two user-selected ROs are XORed prior to counting, producing the beat frequency |f_A − f_B|, which is very sensitive to local PVT mismatch. The TRNG collects XOR jitter bits into an 8-bit register readable at `reg_trng_data`; the health monitor flags the selected RO against `reg_health_lo`/`reg_health_hi` bounds into `reg_health_status`.

## How to test

1. Apply reset (`rst_n` low for ≥10 clock cycles, then release).
2. **Simple parallel measurement:** set `spi_mode=0`, write `ro_sel[2:0]` to select RO (0=7, 1=11, 2=15, 3=21, 4=31), pulse `cnt_enable` high for the desired gate time, pulse `cnt_clear_latch` high for ≥1 cycle to latch the count, then toggle `byte_sel` and read `uo_out` to retrieve the low and high bytes of the 16-bit count. Check `uio_out[4]` for overflow.
3. **SPI-timed measurement:** set `spi_mode=1`, write `reg_gate_l/h` with the desired gate cycle count, set `reg_ctrl[0]=1` to start the auto gate timer, poll `reg_status` (or watch `uio_out[7]` = `meas_done`) for completion, and read `reg_count_l` / `reg_count_h`. Write `reg_ctrl[1]=1` to clear `meas_done` before the next run.
4. **TRNG:** in SPI mode, set `reg_trng_ctrl[0]=1` and enable at least two ROs; read `reg_trng_data` (0x0D) to consume 8 jitter-XOR bits.
5. **Health monitor:** set `reg_trng_ctrl[1]=1`, write `reg_health_lo`/`reg_health_hi` to the expected frequency window, and read `reg_health_status` (0x0E) after each gate.
6. **Differential mode:** set `reg_trng_ctrl[2]=1`, set `reg_ro_sel` to RO_A and `reg_diff_sel` to RO_B, then gate as usual — the count represents |f_A − f_B|.

A cocotb test suite (30 tests in `test/test.py`) exercises parallel counting, SPI register roundtrip, auto gate timing, prescaler selection, TRNG/health/differential modes, and `uio_oe` direction control. All tests pass on the RTL. Gate-level simulation is intentionally skipped in CI because the real sky130 inverter-chain ring oscillators produce ≈10⁹ events/s of simulated time and are not tractable in icarus.

## External hardware

No external hardware is required for core operation. For SPI control, an SPI master (microcontroller or FPGA) is connected to `uio[0]` (CS), `uio[1]` (MOSI), `uio[2]` (MISO), `uio[3]` (SCK) at any clock rate up to half the system clock. Optional: an oscilloscope on `uio_out[5]` (raw RO) and `uio_out[6]` (synced RO) for direct frequency observation, and any 8-bit digital sink (logic analyzer or microcontroller GPIO) on `uo_out[7:0]` for reading the latched count byte in parallel mode.
