<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This design implements three ring oscillators (7, 11, and 15 inverter stages) with a 16-bit frequency counter for process/voltage/temperature (PVT) characterization. The user selects which RO to measure via `ro_sel[1:0]`, enables counting with `cnt_enable`, and latches the count with `cnt_clear`. The `byte_sel` input switches the 8-bit output between the low and high bytes of the 16-bit counter. The raw and synchronized RO signals are also available on bidirectional pins for direct oscilloscope measurement.

The gate time is controlled externally by the duration `cnt_enable` is held high — there is no internal programmable timer. A 3-stage clock-domain-crossing (CDC) synchronizer introduces approximately 3 system clock cycles of latency between the raw RO edge and the counter increment. The `cnt_clear` input is edge-detected (rising edge only), so holding it high will not continuously reset the counter.

## How to test

1. Apply reset (`rst_n` low, then high).
2. Set `ro_sel` (ui_in[1:0]) to choose the oscillator: 00=7-stage, 01=11-stage, 10=15-stage.
3. Assert `cnt_enable` (ui_in[2]) to start counting RO edges.
4. After the desired measurement window (gate time is determined by how long you hold `cnt_enable`), pulse `cnt_clear` (ui_in[3]) high for at least one cycle — this latches the count and resets the counter.
5. Read the low byte (byte_sel=0) and high byte (byte_sel=1, ui_in[4]) from uo_out.
6. The overflow flag is on uio_out[0].

## External hardware

No external hardware required. Optionally connect an oscilloscope to uio_out[1] (raw RO) and uio_out[2] (synchronized RO) for direct frequency measurement.
