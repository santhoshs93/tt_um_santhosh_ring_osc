/*
 * Copyright (c) 2026 Prof. Santhosh Sivasubramani, IIT Delhi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ring Oscillator with PVT Sensor
 * - 5 individually-gatable RO lengths: 7, 11, 15, 21, 31 stages
 * - 16-bit frequency counter with configurable gate time
 * - SPI-configurable: RO select, per-RO enable, auto gate timer, prescaler
 * - Parallel control mode for basic operation without SPI
 * - Configurable prescaler (÷8/÷16/÷32/÷64) for optimal synchronization
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
    // Input assignments (parallel control)
    // ============================================================
    wire [2:0] ro_sel_par  = ui_in[2:0];   // RO select: 0=7, 1=11, 2=15, 3=21, 4=31
    wire       cnt_en_par  = ui_in[3];      // Counter enable
    wire       cnt_clr_par = ui_in[4];      // Synchronous counter latch+clear
    wire       byte_sel    = ui_in[5];      // 0=counter[7:0], 1=counter[15:8]
    wire       spi_mode    = ui_in[6];      // 0=parallel control, 1=SPI control

    // SPI signals
    wire spi_cs_n = uio_in[0];
    wire spi_mosi = uio_in[1];
    wire spi_miso;
    wire spi_sck  = uio_in[3];

    // ============================================================
    // SPI Slave & Register File
    // ============================================================
    wire        wr_en;
    wire [7:0]  wr_addr, wr_data, rd_addr;
    reg  [7:0]  rd_data;

    spi_slave #(.NUM_REGS(16)) u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .spi_cs_n (spi_cs_n),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso),
        .spi_sck  (spi_sck),
        .wr_en    (wr_en),
        .wr_addr  (wr_addr),
        .wr_data  (wr_data),
        .rd_addr  (rd_addr),
        .rd_data  (rd_data)
    );

    // Configuration registers
    reg [7:0] reg_ctrl;        // 0x00: [0]=auto_gate_start (self-clear), [1]=clear_meas_done
    reg [7:0] reg_ro_sel;      // 0x01: [2:0]=RO select (0-4)
    reg [7:0] reg_ro_en;       // 0x02: [4:0]=per-RO enable mask
    reg [7:0] reg_gate_l;      // 0x03: auto gate time [7:0]
    reg [7:0] reg_gate_h;      // 0x04: auto gate time [15:8]
    reg [7:0] reg_prescale;    // 0x05: [1:0]=prescaler (0=÷8, 1=÷16, 2=÷32, 3=÷64)

    // Edge detect for auto gate start
    reg gate_start_prev;
    wire gate_start_edge = reg_ctrl[0] & ~gate_start_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) gate_start_prev <= 1'b0;
        else        gate_start_prev <= reg_ctrl[0];
    end

    // Register write logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl     <= 8'h00;
            reg_ro_sel   <= 8'h00;
            reg_ro_en    <= 8'h1F;   // All 5 ROs enabled
            reg_gate_l   <= 8'h00;
            reg_gate_h   <= 8'h00;
            reg_prescale <= 8'h01;   // Default ÷16
        end else begin
            // Auto-clear start bit after edge detected
            if (gate_start_edge)
                reg_ctrl[0] <= 1'b0;
            // Auto-clear meas_done clear bit
            if (reg_ctrl[1])
                reg_ctrl[1] <= 1'b0;

            if (wr_en) begin
                case (wr_addr)
                    8'h00: reg_ctrl     <= wr_data;
                    8'h01: reg_ro_sel   <= wr_data;
                    8'h02: reg_ro_en    <= wr_data;
                    8'h03: reg_gate_l   <= wr_data;
                    8'h04: reg_gate_h   <= wr_data;
                    8'h05: reg_prescale <= wr_data;
                    default: ;
                endcase
            end
        end
    end

    // Forward declarations for read mux
    wire [7:0] reg_status;
    reg [15:0] freq_latch;

    // Register read mux
    always @(*) begin
        case (rd_addr)
            8'h00: rd_data = reg_ctrl;
            8'h01: rd_data = reg_ro_sel;
            8'h02: rd_data = reg_ro_en;
            8'h03: rd_data = reg_gate_l;
            8'h04: rd_data = reg_gate_h;
            8'h05: rd_data = reg_prescale;
            8'h06: rd_data = reg_status;
            8'h07: rd_data = freq_latch[7:0];
            8'h08: rd_data = freq_latch[15:8];
            default: rd_data = 8'h00;
        endcase
    end

    // ============================================================
    // Per-RO enable (always via SPI register, default 0x1F = all on)
    // ============================================================
    wire ro7_active  = ena & reg_ro_en[0];
    wire ro11_active = ena & reg_ro_en[1];
    wire ro15_active = ena & reg_ro_en[2];
    wire ro21_active = ena & reg_ro_en[3];
    wire ro31_active = ena & reg_ro_en[4];

    // ============================================================
    // Ring Oscillators (5 lengths)
    // ============================================================

`ifdef SIM
    // Simulation: clock dividers replace ring oscillators
    reg [4:0] sim_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sim_div <= 5'd0;
        else if (ena) sim_div <= sim_div + 5'd1;
    end
    wire ro7_out  = ro7_active  ? sim_div[1] : 1'b0;  // clk/4
    wire ro11_out = ro11_active ? sim_div[2] : 1'b0;  // clk/8
    wire ro15_out = ro15_active ? sim_div[3] : 1'b0;  // clk/16
    wire ro21_out = ro21_active ? sim_div[3] : 1'b0;  // clk/16 (distinct in silicon)
    wire ro31_out = ro31_active ? sim_div[4] : 1'b0;  // clk/32
`else
    // Synthesis: sky130 standard cell ring oscillators
    // NAND2 provides enable gating; inverter chain forms odd-inversion ring

    // 7-stage RO: NAND2 + 6 inverters = 7 inversions
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
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i1  (.A(ro11[0]),  .Y(ro11[1]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i2  (.A(ro11[1]),  .Y(ro11[2]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i3  (.A(ro11[2]),  .Y(ro11[3]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i4  (.A(ro11[3]),  .Y(ro11[4]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i5  (.A(ro11[4]),  .Y(ro11[5]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i6  (.A(ro11[5]),  .Y(ro11[6]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i7  (.A(ro11[6]),  .Y(ro11[7]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i8  (.A(ro11[7]),  .Y(ro11[8]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i9  (.A(ro11[8]),  .Y(ro11[9]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro11_i10 (.A(ro11[9]),  .Y(ro11[10]));

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

    // 21-stage RO: NAND2 + 20 inverters
    (* keep *) wire [20:0] ro21;
    (* keep, dont_touch *) sky130_fd_sc_hd__nand2_1 ro21_en  (.A(ro21_active), .B(ro21[20]), .Y(ro21[0]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i1  (.A(ro21[0]),  .Y(ro21[1]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i2  (.A(ro21[1]),  .Y(ro21[2]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i3  (.A(ro21[2]),  .Y(ro21[3]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i4  (.A(ro21[3]),  .Y(ro21[4]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i5  (.A(ro21[4]),  .Y(ro21[5]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i6  (.A(ro21[5]),  .Y(ro21[6]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i7  (.A(ro21[6]),  .Y(ro21[7]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i8  (.A(ro21[7]),  .Y(ro21[8]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i9  (.A(ro21[8]),  .Y(ro21[9]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i10 (.A(ro21[9]),  .Y(ro21[10]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i11 (.A(ro21[10]), .Y(ro21[11]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i12 (.A(ro21[11]), .Y(ro21[12]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i13 (.A(ro21[12]), .Y(ro21[13]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i14 (.A(ro21[13]), .Y(ro21[14]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i15 (.A(ro21[14]), .Y(ro21[15]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i16 (.A(ro21[15]), .Y(ro21[16]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i17 (.A(ro21[16]), .Y(ro21[17]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i18 (.A(ro21[17]), .Y(ro21[18]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i19 (.A(ro21[18]), .Y(ro21[19]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro21_i20 (.A(ro21[19]), .Y(ro21[20]));

    // 31-stage RO: NAND2 + 30 inverters
    (* keep *) wire [30:0] ro31;
    (* keep, dont_touch *) sky130_fd_sc_hd__nand2_1 ro31_en  (.A(ro31_active), .B(ro31[30]), .Y(ro31[0]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i1  (.A(ro31[0]),  .Y(ro31[1]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i2  (.A(ro31[1]),  .Y(ro31[2]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i3  (.A(ro31[2]),  .Y(ro31[3]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i4  (.A(ro31[3]),  .Y(ro31[4]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i5  (.A(ro31[4]),  .Y(ro31[5]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i6  (.A(ro31[5]),  .Y(ro31[6]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i7  (.A(ro31[6]),  .Y(ro31[7]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i8  (.A(ro31[7]),  .Y(ro31[8]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i9  (.A(ro31[8]),  .Y(ro31[9]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i10 (.A(ro31[9]),  .Y(ro31[10]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i11 (.A(ro31[10]), .Y(ro31[11]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i12 (.A(ro31[11]), .Y(ro31[12]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i13 (.A(ro31[12]), .Y(ro31[13]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i14 (.A(ro31[13]), .Y(ro31[14]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i15 (.A(ro31[14]), .Y(ro31[15]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i16 (.A(ro31[15]), .Y(ro31[16]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i17 (.A(ro31[16]), .Y(ro31[17]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i18 (.A(ro31[17]), .Y(ro31[18]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i19 (.A(ro31[18]), .Y(ro31[19]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i20 (.A(ro31[19]), .Y(ro31[20]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i21 (.A(ro31[20]), .Y(ro31[21]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i22 (.A(ro31[21]), .Y(ro31[22]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i23 (.A(ro31[22]), .Y(ro31[23]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i24 (.A(ro31[23]), .Y(ro31[24]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i25 (.A(ro31[24]), .Y(ro31[25]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i26 (.A(ro31[25]), .Y(ro31[26]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i27 (.A(ro31[26]), .Y(ro31[27]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i28 (.A(ro31[27]), .Y(ro31[28]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i29 (.A(ro31[28]), .Y(ro31[29]));
    (* keep, dont_touch *) sky130_fd_sc_hd__inv_1   ro31_i30 (.A(ro31[29]), .Y(ro31[30]));
    /* verilator lint_on PINMISSING */

    wire ro7_out  = ro7[6];
    wire ro11_out = ro11[10];
    wire ro15_out = ro15[14];
    wire ro21_out = ro21[20];
    wire ro31_out = ro31[30];
`endif

    // ============================================================
    // RO output select mux (5-way, 3-bit select)
    // ============================================================
    wire [2:0] effective_ro_sel = spi_mode ? reg_ro_sel[2:0] : ro_sel_par;

    reg ro_out;
    always @(*) begin
        case (effective_ro_sel)
            3'd0:    ro_out = ro7_out;
            3'd1:    ro_out = ro11_out;
            3'd2:    ro_out = ro15_out;
            3'd3:    ro_out = ro21_out;
            3'd4:    ro_out = ro31_out;
            default: ro_out = ro7_out;
        endcase
    end

    // ============================================================
    // Configurable Prescaler
    // Divides RO frequency for reliable synchronization to 50 MHz clk.
    // Settings: 0=÷8, 1=÷16(default), 2=÷32, 3=÷64
    // In simulation, ROs are already slow clock dividers — skip prescaler.
    // ============================================================
`ifdef SIM
    wire ro_divided = ro_out;  // No prescaler needed in sim
`else
    reg [5:0] ro_prescaler;
    always @(posedge ro_out or negedge rst_n) begin
        if (!rst_n)
            ro_prescaler <= 6'd0;
        else
            ro_prescaler <= ro_prescaler + 6'd1;
    end

    reg ro_divided;
    always @(*) begin
        case (reg_prescale[1:0])
            2'd0:    ro_divided = ro_prescaler[2];  // ÷8
            2'd1:    ro_divided = ro_prescaler[3];  // ÷16 (default)
            2'd2:    ro_divided = ro_prescaler[4];  // ÷32
            2'd3:    ro_divided = ro_prescaler[5];  // ÷64
        endcase
    end
`endif

    // ============================================================
    // Synchronize prescaled RO output to system clock domain
    // ============================================================
    reg [2:0] ro_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ro_sync <= 3'b0;
        else
            ro_sync <= {ro_sync[1:0], ro_divided};
    end
    wire ro_rising = (ro_sync[2:1] == 2'b01);

    // ============================================================
    // Edge detectors
    // ============================================================
    // Parallel cnt_clear edge
    reg cnt_clr_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cnt_clr_prev <= 1'b0;
        else        cnt_clr_prev <= cnt_clr_par;
    end
    wire cnt_clr_edge = cnt_clr_par & ~cnt_clr_prev;

    // ============================================================
    // Auto Gate Timer (SPI mode only)
    // Write gate time to reg_gate_h:reg_gate_l, set reg_ctrl[0]=1
    // Hardware counts for gate_time cycles, auto-latches, sets meas_done
    // ============================================================
    reg [15:0] gate_timer;
    reg        gate_active;
    reg        meas_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_timer  <= 16'd0;
            gate_active <= 1'b0;
            meas_done   <= 1'b0;
        end else begin
            // Start gate measurement
            if (gate_start_edge) begin
                gate_timer  <= {reg_gate_h, reg_gate_l};
                gate_active <= 1'b1;
                meas_done   <= 1'b0;
            end else if (gate_active) begin
                if (gate_timer == 16'd0) begin
                    gate_active <= 1'b0;
                    meas_done   <= 1'b1;
                end else begin
                    gate_timer <= gate_timer - 16'd1;
                end
            end
            // Clear meas_done via reg_ctrl[1]
            if (reg_ctrl[1])
                meas_done <= 1'b0;
        end
    end

    // ============================================================
    // Effective counter controls (parallel vs SPI mode)
    // ============================================================
    wire effective_cnt_enable = spi_mode ? gate_active : cnt_en_par;

    // ============================================================
    // 16-bit Frequency Counter
    // ============================================================
    reg [15:0] freq_counter;
    // freq_latch declared above (forward ref for read mux)
    reg        overflow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_counter <= 16'd0;
            freq_latch   <= 16'd0;
            overflow     <= 1'b0;
        end else if (!spi_mode && cnt_clr_edge) begin
            // Parallel mode: latch + clear
            freq_latch   <= freq_counter;
            freq_counter <= 16'd0;
            overflow     <= 1'b0;
        end else if (spi_mode && gate_start_edge) begin
            // SPI mode: clear counter at gate start
            freq_counter <= 16'd0;
            overflow     <= 1'b0;
        end else begin
            // SPI mode: latch at gate done
            if (spi_mode && gate_active && gate_timer == 16'd0)
                freq_latch <= freq_counter;
            // Count edges (both modes)
            if (effective_cnt_enable && ro_rising) begin
                if (freq_counter == 16'hFFFF)
                    overflow <= 1'b1;
                else
                    freq_counter <= freq_counter + 16'd1;
            end
        end
    end

    // ============================================================
    // Status register (read-only at 0x06)
    // ============================================================
    assign reg_status = {5'b0, meas_done, overflow, gate_active};

    // ============================================================
    // Output assignments
    // ============================================================
    assign uo_out = byte_sel ? freq_latch[15:8] : freq_latch[7:0];

    assign uio_out[0]  = 1'b0;             // CS is input
    assign uio_out[1]  = 1'b0;             // MOSI is input
    assign uio_out[2]  = spi_miso;         // MISO output
    assign uio_out[3]  = 1'b0;             // SCK is input
    assign uio_out[4]  = overflow;          // Counter overflow flag
    assign uio_out[5]  = ro_out;            // Raw RO output
    assign uio_out[6]  = ro_sync[2];        // Synchronized RO
    assign uio_out[7]  = meas_done;         // Auto gate measurement done

    // MISO tri-stated when CS inactive; uio[7:4] always outputs
    assign uio_oe = {4'b1111, 1'b0, ~spi_cs_n, 2'b00};

    // Unused inputs
    wire _unused = &{ui_in[7], uio_in[2], uio_in[7:4], 1'b0};

endmodule
