# SPDX-FileCopyrightText: (C) 2026 Prof. Santhosh Sivasubramani, IIT Delhi
# SPDX-License-Identifier: Apache-2.0

"""
Cocotb tests for ring_osc with SPI, 5 RO lengths, auto gate timer, configurable prescaler.

Pin mapping (parallel mode, spi_mode=0):
  ui_in[2:0] = ro_sel (0=7stg, 1=11stg, 2=15stg, 3=21stg, 4=31stg)
  ui_in[3]   = cnt_enable
  ui_in[4]   = cnt_clear (edge: latch + clear counter)
  ui_in[5]   = byte_sel  (0=low byte, 1=high byte)
  ui_in[6]   = spi_mode  (0=parallel, 1=SPI control)
  ui_in[7]   = unused

SPI pin mapping:
  uio_in[0]  = spi_cs_n
  uio_in[1]  = spi_mosi
  uio_out[2] = spi_miso (active only when CS asserted)
  uio_in[3]  = spi_sck

Output pins:
  uo_out[7:0]  = freq_latch byte (selected by byte_sel)
  uio_out[4]   = overflow
  uio_out[5]   = raw RO output
  uio_out[6]   = synced RO output
  uio_out[7]   = meas_done

SPI register map:
  0x00  reg_ctrl     [0]=auto_gate_start (self-clear), [1]=clear_meas_done
  0x01  reg_ro_sel   [2:0] RO select
  0x02  reg_ro_en    [4:0] per-RO enable (default 0x1F)
  0x03  reg_gate_l   gate time low byte
  0x04  reg_gate_h   gate time high byte
  0x05  reg_prescale [1:0] prescaler (0=div8,1=div16,2=div32,3=div64; default 1)
  0x06  reg_status   RO: [0]=gate_active, [1]=overflow, [2]=meas_done
  0x07  reg_count_l  RO: freq_latch[7:0]
  0x08  reg_count_h  RO: freq_latch[15:8]
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


# Helpers

async def reset_dut(dut):
    """Apply reset and release. CS high (idle), parallel mode, all pins zero."""
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0b0000_0001  # CS=1 (inactive), everything else 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def spi_write(dut, addr, data):
    """SPI write: 16-bit frame [0][addr6:0][data7:0], MSB first, Mode 0."""
    cs_bit = 0
    mosi_bit = 1
    sck_bit = 3

    word = ((addr & 0x7F) << 8) | (data & 0xFF)

    # Assert CS (active low)
    dut.uio_in.value = 0  # CS=0, SCK=0
    await ClockCycles(dut.clk, 4)

    for i in range(16):
        bit_val = (word >> (15 - i)) & 1
        # MOSI + SCK low
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 4)
        # SCK rising - slave samples
        dut.uio_in.value = (bit_val << mosi_bit) | (1 << sck_bit)
        await ClockCycles(dut.clk, 4)
        # SCK falling
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 2)

    # Deassert CS
    dut.uio_in.value = (1 << cs_bit)
    await ClockCycles(dut.clk, 4)


async def spi_read(dut, addr):
    """SPI read: 16-bit frame [1][addr6:0][00000000], returns 8-bit data."""
    cs_bit = 0
    mosi_bit = 1
    sck_bit = 3

    word = (1 << 15) | ((addr & 0x7F) << 8)

    dut.uio_in.value = 0  # CS=0
    await ClockCycles(dut.clk, 4)

    read_data = 0
    for i in range(16):
        bit_val = (word >> (15 - i)) & 1
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 4)
        # SCK rising
        dut.uio_in.value = (bit_val << mosi_bit) | (1 << sck_bit)
        await ClockCycles(dut.clk, 2)
        # Sample MISO during data phase (bits 8-15)
        if i >= 8:
            miso = (int(dut.uio_out.value) >> 2) & 1
            read_data = (read_data << 1) | miso
        await ClockCycles(dut.clk, 2)
        # SCK falling
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 2)

    dut.uio_in.value = (1 << cs_bit)
    await ClockCycles(dut.clk, 4)
    return read_data


def set_ui(dut, ro_sel=0, cnt_en=0, cnt_clr=0, byte_sel=0, spi_mode=0):
    """Set ui_in from named fields."""
    val = (ro_sel & 0x7) | ((cnt_en & 1) << 3) | ((cnt_clr & 1) << 4) | \
          ((byte_sel & 1) << 5) | ((spi_mode & 1) << 6)
    dut.ui_in.value = val


# ============================
# PARALLEL-MODE TESTS
# ============================

@cocotb.test()
async def test_reset(dut):
    """After reset, counter outputs and flags are zero."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    assert int(dut.uo_out.value) == 0, "uo_out should be 0 after reset"
    # overflow = uio_out[4], meas_done = uio_out[7]
    assert (int(dut.uio_out.value) >> 4) & 1 == 0, "overflow should be 0"
    assert (int(dut.uio_out.value) >> 7) & 1 == 0, "meas_done should be 0"


@cocotb.test()
async def test_parallel_counter_basic(dut):
    """Enable counter in parallel mode, latch, verify non-zero count."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # RO 0 (7-stage, sim_div[1], period=4 clk), enable counting
    set_ui(dut, ro_sel=0, cnt_en=1)
    await ClockCycles(dut.clk, 100)

    # Latch: pulse cnt_clear
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=1)
    await ClockCycles(dut.clk, 2)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=0)
    await ClockCycles(dut.clk, 2)

    low = int(dut.uo_out.value)
    set_ui(dut, ro_sel=0, cnt_en=1, byte_sel=1)
    await ClockCycles(dut.clk, 2)
    high = int(dut.uo_out.value)
    count = (high << 8) | low

    dut._log.info(f"Parallel count after 100 cycles: {count}")
    assert count > 0, f"Count should be > 0, got {count}"
    # ro7 sim rate = 1 edge per 4 clk, approx 25 edges in 100 cycles
    assert 15 <= count <= 35, f"Count {count} out of expected range [15,35]"


@cocotb.test()
async def test_parallel_byte_select(dut):
    """byte_sel=0 gives low byte, byte_sel=1 gives high byte."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Count for a while, then latch
    set_ui(dut, ro_sel=0, cnt_en=1)
    await ClockCycles(dut.clk, 100)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=1)
    await ClockCycles(dut.clk, 2)
    set_ui(dut, ro_sel=0, cnt_en=0, cnt_clr=0, byte_sel=0)
    await ClockCycles(dut.clk, 2)

    low_byte = int(dut.uo_out.value)
    set_ui(dut, ro_sel=0, cnt_en=0, byte_sel=1)
    await ClockCycles(dut.clk, 2)
    high_byte = int(dut.uo_out.value)

    count = (high_byte << 8) | low_byte
    dut._log.info(f"Count={count}, low=0x{low_byte:02x}, high=0x{high_byte:02x}")
    assert count > 0, "Count should be non-zero after counting"
    # For approx 25 count, high byte should be 0
    assert high_byte == 0, f"High byte should be 0 for small count, got {high_byte}"


@cocotb.test()
async def test_parallel_cnt_clear_latch(dut):
    """cnt_clear edge latches current count, then clears counter for next measurement."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # First measurement
    set_ui(dut, ro_sel=0, cnt_en=1)
    await ClockCycles(dut.clk, 80)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=1)
    await ClockCycles(dut.clk, 2)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=0)
    await ClockCycles(dut.clk, 2)
    first_count = int(dut.uo_out.value)
    dut._log.info(f"First latch: {first_count}")
    assert first_count > 0, "First measurement should be non-zero"

    # Second measurement (counter was cleared)
    await ClockCycles(dut.clk, 80)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=1)
    await ClockCycles(dut.clk, 2)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=0)
    await ClockCycles(dut.clk, 2)
    second_count = int(dut.uo_out.value)
    dut._log.info(f"Second latch: {second_count}")
    assert second_count > 0, "Second measurement should also be non-zero"

    # Both should be similar (same gate window)
    diff = abs(first_count - second_count)
    assert diff <= 3, f"Counts should be similar: {first_count} vs {second_count}"


@cocotb.test()
async def test_ro_select_all_5(dut):
    """All 5 RO selections (0-4) produce valid counts in parallel mode."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())

    # Expected: ro7 > ro11 > ro15=ro21 > ro31 (sim frequencies)
    # sim_div[1]=4clk, [2]=8clk, [3]=16clk, [3]=16clk, [4]=32clk
    counts = []
    for sel in range(5):
        await reset_dut(dut)
        set_ui(dut, ro_sel=sel, cnt_en=1)
        await ClockCycles(dut.clk, 200)
        set_ui(dut, ro_sel=sel, cnt_en=1, cnt_clr=1)
        await ClockCycles(dut.clk, 2)
        set_ui(dut, ro_sel=sel, cnt_en=1, cnt_clr=0, byte_sel=0)
        await ClockCycles(dut.clk, 2)
        low = int(dut.uo_out.value)
        set_ui(dut, ro_sel=sel, cnt_en=1, byte_sel=1)
        await ClockCycles(dut.clk, 2)
        high = int(dut.uo_out.value)
        count = (high << 8) | low
        counts.append(count)
        dut._log.info(f"RO sel={sel}: count={count}")

    for i, c in enumerate(counts):
        assert c > 0, f"RO sel={i} count should be > 0, got {c}"

    # ro7 should be faster than ro31
    assert counts[0] > counts[4], \
        f"ro7({counts[0]}) should count faster than ro31({counts[4]})"


@cocotb.test()
async def test_parallel_overflow(dut):
    """Counter overflow sets overflow flag on uio_out[4]."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Use fastest RO (sel=0, 1 count per 4 clk), count until overflow
    # Need 65536 counts x 4 clk = 262144 clk
    set_ui(dut, ro_sel=0, cnt_en=1)
    await ClockCycles(dut.clk, 270000)

    overflow = (int(dut.uio_out.value) >> 4) & 1
    assert overflow == 1, "Overflow flag should be set after 65536+ counts"


@cocotb.test()
async def test_uio_oe_cs_high(dut):
    """When CS is high (inactive), uio_oe = 0xF0."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # CS=1 (default after reset)
    oe = int(dut.uio_oe.value)
    assert oe == 0xF0, f"uio_oe should be 0xF0 when CS high, got 0x{oe:02x}"


@cocotb.test()
async def test_uio_oe_cs_low(dut):
    """When CS is low (active), MISO output is enabled: uio_oe = 0xF4."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Assert CS low
    dut.uio_in.value = 0x00  # CS=0
    await ClockCycles(dut.clk, 4)

    oe = int(dut.uio_oe.value)
    assert oe == 0xF4, f"uio_oe should be 0xF4 when CS low, got 0x{oe:02x}"

    # Deassert CS
    dut.uio_in.value = 0x01
    await ClockCycles(dut.clk, 4)
    oe = int(dut.uio_oe.value)
    assert oe == 0xF0, f"uio_oe should return to 0xF0 when CS high, got 0x{oe:02x}"


@cocotb.test()
async def test_raw_vs_synced_output(dut):
    """Raw RO (uio[5]) and synced RO (uio[6]) toggle during counting."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    set_ui(dut, ro_sel=0, cnt_en=1)

    # Collect raw and synced over several cycles
    raw_vals = set()
    sync_vals = set()
    for _ in range(40):
        await ClockCycles(dut.clk, 1)
        raw_vals.add((int(dut.uio_out.value) >> 5) & 1)
        sync_vals.add((int(dut.uio_out.value) >> 6) & 1)

    assert len(raw_vals) == 2, "Raw RO output should toggle (have both 0 and 1)"
    assert len(sync_vals) == 2, "Synced RO output should toggle (have both 0 and 1)"


# ============================
# SPI-MODE TESTS
# ============================

@cocotb.test()
async def test_spi_register_roundtrip(dut):
    """Write and read back all R/W SPI registers."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Write distinct values to writable registers
    test_vals = {
        0x01: 0x03,  # reg_ro_sel
        0x02: 0x15,  # reg_ro_en
        0x03: 0xAB,  # reg_gate_l
        0x04: 0xCD,  # reg_gate_h
        0x05: 0x02,  # reg_prescale
    }
    for addr, val in test_vals.items():
        await spi_write(dut, addr, val)

    # Read them back
    for addr, expected in test_vals.items():
        got = await spi_read(dut, addr)
        assert got == expected, \
            f"Reg 0x{addr:02x}: expected 0x{expected:02x}, got 0x{got:02x}"


@cocotb.test()
async def test_spi_default_registers(dut):
    """After reset, registers have correct default values."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    defaults = {
        0x00: 0x00,  # reg_ctrl
        0x01: 0x00,  # reg_ro_sel
        0x02: 0x1F,  # reg_ro_en (all 5 enabled)
        0x03: 0x00,  # reg_gate_l
        0x04: 0x00,  # reg_gate_h
        0x05: 0x01,  # reg_prescale (div16 default)
    }
    for addr, expected in defaults.items():
        got = await spi_read(dut, addr)
        assert got == expected, \
            f"Default reg 0x{addr:02x}: expected 0x{expected:02x}, got 0x{got:02x}"


@cocotb.test()
async def test_spi_ro_select(dut):
    """SPI RO select (reg 0x01) controls which RO is measured in SPI mode."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())

    counts = []
    for sel in [0, 4]:  # 7-stage (fastest) vs 31-stage (slowest)
        await reset_dut(dut)
        set_ui(dut, spi_mode=1)

        # Set RO select via SPI
        await spi_write(dut, 0x01, sel)
        # Set gate time = 200
        await spi_write(dut, 0x03, 200)
        await spi_write(dut, 0x04, 0)
        # Start measurement
        await spi_write(dut, 0x00, 0x01)
        await ClockCycles(dut.clk, 400)

        # Read count
        low = await spi_read(dut, 0x07)
        high = await spi_read(dut, 0x08)
        count = (high << 8) | low
        counts.append(count)
        dut._log.info(f"SPI RO sel={sel}: count={count}")

    assert counts[0] > 0, "RO 0 count should be > 0"
    assert counts[1] > 0, "RO 4 count should be > 0"
    assert counts[0] > counts[1], \
        f"RO 0 ({counts[0]}) should be faster than RO 4 ({counts[1]})"


@cocotb.test()
async def test_spi_auto_gate_timer(dut):
    """Auto gate timer: start, count, latch, meas_done."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    set_ui(dut, spi_mode=1)

    # Select RO 0, gate time = 500
    await spi_write(dut, 0x01, 0)     # ro_sel=0
    await spi_write(dut, 0x03, 0xF4)  # gate_l = 244 (500 & 0xFF = 0xF4)
    await spi_write(dut, 0x04, 0x01)  # gate_h = 1   (500 >> 8 = 0x01)
    # Start
    await spi_write(dut, 0x00, 0x01)

    # Wait for measurement to complete
    await ClockCycles(dut.clk, 700)

    # Check meas_done
    meas_done = (int(dut.uio_out.value) >> 7) & 1
    assert meas_done == 1, "meas_done should be 1 after gate timer expires"

    # Read status via SPI
    status = await spi_read(dut, 0x06)
    assert (status >> 2) & 1 == 1, f"Status meas_done bit should be 1, status=0x{status:02x}"
    assert (status >> 0) & 1 == 0, f"Status gate_active should be 0, status=0x{status:02x}"

    # Read count
    low = await spi_read(dut, 0x07)
    high = await spi_read(dut, 0x08)
    count = (high << 8) | low
    dut._log.info(f"Auto gate count (gate=500, ro7): {count}")
    # ro7 at 1/4 clk rate, approx 125 counts in 500 cycles
    assert count > 50, f"Count should be > 50, got {count}"
    assert count < 200, f"Count should be < 200, got {count}"


@cocotb.test()
async def test_spi_meas_done_clear(dut):
    """Clear meas_done by writing reg_ctrl[1]=1."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    set_ui(dut, spi_mode=1)

    # Quick measurement
    await spi_write(dut, 0x01, 0)
    await spi_write(dut, 0x03, 50)
    await spi_write(dut, 0x04, 0)
    await spi_write(dut, 0x00, 0x01)  # start
    await ClockCycles(dut.clk, 200)

    # meas_done should be high
    meas_done = (int(dut.uio_out.value) >> 7) & 1
    assert meas_done == 1, "meas_done should be 1 after measurement"

    # Clear via reg_ctrl[1]
    await spi_write(dut, 0x00, 0x02)
    await ClockCycles(dut.clk, 5)

    meas_done = (int(dut.uio_out.value) >> 7) & 1
    assert meas_done == 0, "meas_done should be 0 after clear"


@cocotb.test()
async def test_spi_ro_enable_disable(dut):
    """Disabling an RO via reg_ro_en stops its counting."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    set_ui(dut, spi_mode=1)

    # First: measure with RO 0 enabled (default)
    await spi_write(dut, 0x01, 0)
    await spi_write(dut, 0x03, 200)
    await spi_write(dut, 0x04, 0)
    await spi_write(dut, 0x00, 0x01)
    await ClockCycles(dut.clk, 400)
    low = await spi_read(dut, 0x07)
    high = await spi_read(dut, 0x08)
    count_enabled = (high << 8) | low
    dut._log.info(f"RO 0 enabled: count={count_enabled}")

    # Disable RO 0 (clear bit 0 of reg_ro_en)
    await spi_write(dut, 0x02, 0x1E)  # all except bit 0

    # Clear meas_done for next measurement
    await spi_write(dut, 0x00, 0x02)
    await ClockCycles(dut.clk, 5)

    # Measure again with RO 0 disabled
    await spi_write(dut, 0x03, 200)
    await spi_write(dut, 0x04, 0)
    await spi_write(dut, 0x00, 0x01)
    await ClockCycles(dut.clk, 400)
    low = await spi_read(dut, 0x07)
    high = await spi_read(dut, 0x08)
    count_disabled = (high << 8) | low
    dut._log.info(f"RO 0 disabled: count={count_disabled}")

    assert count_enabled > 0, "Count with RO enabled should be > 0"
    assert count_disabled == 0, f"Count with RO disabled should be 0, got {count_disabled}"


@cocotb.test()
async def test_spi_status_register(dut):
    """Status register (0x06) reflects gate_active, overflow, meas_done."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    set_ui(dut, spi_mode=1)

    # After reset: all status bits should be 0
    status = await spi_read(dut, 0x06)
    assert status == 0, f"Status should be 0 after reset, got 0x{status:02x}"

    # Start a long gate time to catch gate_active=1
    await spi_write(dut, 0x01, 0)
    await spi_write(dut, 0x03, 0xFF)  # gate = 0x03FF = 1023
    await spi_write(dut, 0x04, 0x03)
    await spi_write(dut, 0x00, 0x01)  # start
    await ClockCycles(dut.clk, 50)

    # gate_active should be 1 during measurement
    status = await spi_read(dut, 0x06)
    gate_active = status & 1
    assert gate_active == 1, f"gate_active should be 1 during measurement, status=0x{status:02x}"


@cocotb.test()
async def test_spi_mode_isolation(dut):
    """In SPI mode, parallel cnt_enable pin has no effect on counting."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # SPI mode, but also set parallel cnt_enable
    set_ui(dut, ro_sel=0, cnt_en=1, spi_mode=1)
    await ClockCycles(dut.clk, 200)

    # Latch via parallel (should not work in SPI mode for counting)
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=1, spi_mode=1)
    await ClockCycles(dut.clk, 2)
    set_ui(dut, ro_sel=0, cnt_en=1, spi_mode=1)
    await ClockCycles(dut.clk, 2)

    # Read count via SPI
    low = await spi_read(dut, 0x07)
    high = await spi_read(dut, 0x08)
    count = (high << 8) | low
    dut._log.info(f"SPI mode + parallel cnt_en=1: count from SPI read = {count}")
    # In SPI mode, effective_cnt_enable = gate_active (which is 0, no gate started)
    # So counter should not have incremented
    assert count == 0, f"Counter should be 0 in SPI mode without gate start, got {count}"


@cocotb.test()
async def test_spi_read_counter_parallel_mode(dut):
    """SPI can read counter values even in parallel mode."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Parallel mode: count with RO 0
    set_ui(dut, ro_sel=0, cnt_en=1, spi_mode=0)
    await ClockCycles(dut.clk, 100)

    # Latch via parallel pin
    set_ui(dut, ro_sel=0, cnt_en=1, cnt_clr=1, spi_mode=0)
    await ClockCycles(dut.clk, 2)
    set_ui(dut, ro_sel=0, cnt_en=1, spi_mode=0)
    await ClockCycles(dut.clk, 2)

    # Read latched value via parallel output
    par_low = int(dut.uo_out.value)

    # Read via SPI (should match)
    spi_low = await spi_read(dut, 0x07)
    assert spi_low == par_low, \
        f"SPI read (0x{spi_low:02x}) should match parallel output (0x{par_low:02x})"


@cocotb.test()
async def test_consecutive_gate_measurements(dut):
    """Multiple auto gate measurements produce consistent results."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    set_ui(dut, spi_mode=1)

    await spi_write(dut, 0x01, 0)  # RO 0

    counts = []
    for trial in range(3):
        # Clear meas_done if set
        await spi_write(dut, 0x00, 0x02)
        await ClockCycles(dut.clk, 5)

        # Set gate time = 200
        await spi_write(dut, 0x03, 200)
        await spi_write(dut, 0x04, 0)
        # Start
        await spi_write(dut, 0x00, 0x01)
        await ClockCycles(dut.clk, 400)

        low = await spi_read(dut, 0x07)
        high = await spi_read(dut, 0x08)
        count = (high << 8) | low
        counts.append(count)
        dut._log.info(f"Trial {trial}: count={count}")

    # All counts should be similar (within +/-5 due to phase alignment)
    for i in range(1, len(counts)):
        diff = abs(counts[i] - counts[0])
        assert diff <= 5, \
            f"Trial {i} count {counts[i]} differs from trial 0 count {counts[0]} by {diff}"


@cocotb.test()
async def test_spi_prescale_register(dut):
    """Prescaler register can be written and read back."""
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Default should be 1 (div16)
    val = await spi_read(dut, 0x05)
    assert val == 0x01, f"Default prescale should be 0x01, got 0x{val:02x}"

    # Write each valid value
    for pval in [0, 1, 2, 3]:
        await spi_write(dut, 0x05, pval)
        got = await spi_read(dut, 0x05)
        assert got == pval, f"Prescale write {pval}: read back {got}"
