/*
 * Copyright (c) 2026 Prof. Santhosh Sivasubramani, IIT Delhi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ring Oscillator with PVT Sensor
 * - 3 individually-gatable RO lengths: 7, 11, 15 stages
 * - 16-bit frequency counter with configurable gate time
 * - Process/voltage/temperature characterization baseline
 * - Uses sky130 standard cells for synthesizable ring oscillators
 */

`default_nettype none

module tt_um_santhosh_ring_osc (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ============================================================
    // Input assignments
    // ============================================================
    wire [1:0] ro_sel     = ui_in[1:0];   // RO select: 00=7, 01=11, 10=15
    wire       cnt_enable = ui_in[2];      // Counter enable
    wire       cnt_clear  = ui_in[3];      // Synchronous counter clear/latch
    wire       byte_sel   = ui_in[4];      // 0=counter[7:0], 1=counter[15:8]
    wire       ro7_dis    = ui_in[5];      // Disable 7-stage RO (active-high)
    wire       ro11_dis   = ui_in[6];      // Disable 11-stage RO (active-high)
    wire       ro15_dis   = ui_in[7];      // Disable 15-stage RO (active-high)

    // Per-RO enable gating (default ui_in[7:5]=000 → all ROs active)
    wire ro7_active  = ena & ~ro7_dis;
    wire ro11_active = ena & ~ro11_dis;
    wire ro15_active = ena & ~ro15_dis;

    // ============================================================
    // Ring Oscillators
    // ============================================================

`ifdef SIM
    // Simulation: clock dividers replace ring oscillators
    // (Sky130 cell ring oscillators have zero propagation delay in sim)
    reg [3:0] sim_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sim_div <= 4'd0;
        else if (ena) sim_div <= sim_div + 4'd1;
    end
    wire ro7_out  = ro7_active  ? sim_div[1] : 1'b0;  // clk/4 (gated)
    wire ro11_out = ro11_active ? sim_div[2] : 1'b0;  // clk/8 (gated)
    wire ro15_out = ro15_active ? sim_div[3] : 1'b0;  // clk/16 (gated)
`else
    // Synthesis: sky130 standard cell ring oscillators
    // NAND2 provides enable gating; inverter chain forms odd-inversion ring

    // 7-stage RO: NAND2 (enable) + 6 inverters = 7 inversions
    (* keep *) wire [6:0] ro7;
    /* verilator lint_off PINMISSING */
    (* keep, dont_touch *) sky130_fd_sc_hd__nand2_1 ro7_en (.A(ro7_active), .B(ro7[6]), .Y(ro7[0]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro7_i1 (.A(ro7[0]), .Y(ro7[1]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro7_i2 (.A(ro7[1]), .Y(ro7[2]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro7_i3 (.A(ro7[2]), .Y(ro7[3]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro7_i4 (.A(ro7[3]), .Y(ro7[4]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro7_i5 (.A(ro7[4]), .Y(ro7[5]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro7_i6 (.A(ro7[5]), .Y(ro7[6]));

    // 11-stage RO: NAND2 + 10 inverters
    (* keep *) wire [10:0] ro11;
    (* keep, dont_touch *) sky130_fd_sc_hd__nand2_1 ro11_en  (.A(ro11_active), .B(ro11[10]), .Y(ro11[0]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i1  (.A(ro11[0]), .Y(ro11[1]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i2  (.A(ro11[1]), .Y(ro11[2]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i3  (.A(ro11[2]), .Y(ro11[3]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i4  (.A(ro11[3]), .Y(ro11[4]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i5  (.A(ro11[4]), .Y(ro11[5]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i6  (.A(ro11[5]), .Y(ro11[6]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i7  (.A(ro11[6]), .Y(ro11[7]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i8  (.A(ro11[7]), .Y(ro11[8]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i9  (.A(ro11[8]), .Y(ro11[9]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i10 (.A(ro11[9]), .Y(ro11[10]));

    // 15-stage RO: NAND2 + 14 inverters
    (* keep *) wire [14:0] ro15;
    (* keep, dont_touch *) sky130_fd_sc_hd__nand2_1 ro15_en  (.A(ro15_active), .B(ro15[14]), .Y(ro15[0]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i1  (.A(ro15[0]),  .Y(ro15[1]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i2  (.A(ro15[1]),  .Y(ro15[2]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i3  (.A(ro15[2]),  .Y(ro15[3]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i4  (.A(ro15[3]),  .Y(ro15[4]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i5  (.A(ro15[4]),  .Y(ro15[5]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i6  (.A(ro15[5]),  .Y(ro15[6]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i7  (.A(ro15[6]),  .Y(ro15[7]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i8  (.A(ro15[7]),  .Y(ro15[8]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i9  (.A(ro15[8]),  .Y(ro15[9]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i10 (.A(ro15[9]),  .Y(ro15[10]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i11 (.A(ro15[10]), .Y(ro15[11]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i12 (.A(ro15[11]), .Y(ro15[12]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i13 (.A(ro15[12]), .Y(ro15[13]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro15_i14 (.A(ro15[13]), .Y(ro15[14]));
    /* verilator lint_on PINMISSING */

    wire ro7_out  = ro7[6];
    wire ro11_out = ro11[10];
    wire ro15_out = ro15[14];
`endif

    // RO output select mux
    reg ro_out;
    always @(*) begin
        case (ro_sel)
            2'b00:   ro_out = ro7_out;
            2'b01:   ro_out = ro11_out;
            2'b10:   ro_out = ro15_out;
            default: ro_out = ro7_out;
        endcase
    end

    // ============================================================
    // Synchronize RO output to system clock domain
    // ============================================================
    reg [2:0] ro_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ro_sync <= 3'b0;
        else
            ro_sync <= {ro_sync[1:0], ro_out};
    end
    wire ro_rising = (ro_sync[2:1] == 2'b01);

    // Edge-detect cnt_clear to prevent continuous reset when held high
    reg cnt_clear_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt_clear_prev <= 1'b0;
        else
            cnt_clear_prev <= cnt_clear;
    end
    wire cnt_clear_edge = cnt_clear & ~cnt_clear_prev;

    // ============================================================
    // 16-bit Frequency Counter
    // ============================================================
    reg [15:0] freq_counter;
    reg [15:0] freq_latch;
    reg        overflow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_counter <= 16'd0;
            freq_latch   <= 16'd0;
            overflow     <= 1'b0;
        end else if (cnt_clear_edge) begin
            freq_latch   <= freq_counter;
            freq_counter <= 16'd0;
            overflow     <= 1'b0;
        end else if (cnt_enable && ro_rising) begin
            if (freq_counter == 16'hFFFF)
                overflow <= 1'b1;
            else
                freq_counter <= freq_counter + 16'd1;
        end
    end

    // ============================================================
    // Output assignments
    // ============================================================
    assign uo_out = byte_sel ? freq_latch[15:8] : freq_latch[7:0];

    assign uio_out[0] = overflow;
    assign uio_out[1] = ro_out;       // Raw RO output for oscilloscope
    assign uio_out[2] = ro_sync[2];   // Synchronized RO
    assign uio_out[7:3] = 5'b0;

    assign uio_oe = 8'b0000_0111;     // uio[2:0] are outputs

    // Unused inputs
    wire _unused = &{uio_in, 1'b0};

endmodule
