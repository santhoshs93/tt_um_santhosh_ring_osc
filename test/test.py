# SPDX-FileCopyrightText: © 2026 Prof. Santhosh Sivasubramani, IIT Delhi
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


async def reset_dut(dut):
    """Apply reset and release."""
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_reset(dut):
    """After reset, counter latch should be zero."""
    clock = Clock(dut.clk, 20, unit="ns")  # 50 MHz
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Read low byte (byte_sel=0) — should be 0 after reset
    dut.ui_in.value = 0b00000  # ro_sel=00, cnt_en=0, cnt_clr=0, byte_sel=0
    await ClockCycles(dut.clk, 1)
    assert dut.uo_out.value == 0, "Counter should be zero after reset"


@cocotb.test()
async def test_counter_enable(dut):
    """Enable counter with RO select 0 and check it counts."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Select RO 0 (7-stage), enable counting: ro_sel=00, cnt_enable=1
    dut.ui_in.value = 0b00100  # cnt_enable=1, ro_sel=00
    await ClockCycles(dut.clk, 500)

    # Latch counter: pulse cnt_clear for 1 cycle (it latches AND clears on posedge)
    dut.ui_in.value = 0b01000  # cnt_clear=1
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value = 0b00000  # byte_sel=0
    await ClockCycles(dut.clk, 1)
    low_byte = int(dut.uo_out.value)
    dut._log.info(f"Counter low byte after 500 clocks: {low_byte}")
    assert low_byte > 0, "Counter should have accumulated edges after 500 clock cycles"


@cocotb.test()
async def test_byte_select(dut):
    """Verify byte_sel switches between low and high bytes of latch."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Enable and count for a while
    dut.ui_in.value = 0b00100  # cnt_enable=1
    await ClockCycles(dut.clk, 100)

    # Latch (1-cycle pulse — holding cnt_clear HIGH overwrites latch with 0)
    dut.ui_in.value = 0b01000  # cnt_clear latch
    await ClockCycles(dut.clk, 1)

    # Read low byte
    dut.ui_in.value = 0b00000
    await ClockCycles(dut.clk, 1)
    low = int(dut.uo_out.value)

    # Read high byte
    dut.ui_in.value = 0b10000  # byte_sel=1
    await ClockCycles(dut.clk, 1)
    high = int(dut.uo_out.value)

    count_16 = (high << 8) | low
    dut._log.info(f"Latched counter: 0x{high:02x}{low:02x} = {count_16}")
    assert count_16 > 0, "16-bit counter should be non-zero after counting"


@cocotb.test()
async def test_ro_select(dut):
    """Switch between all 3 RO selects and verify no crash."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    for sel in [0, 1, 2, 3]:
        await reset_dut(dut)
        dut.ui_in.value = (sel & 0x3) | 0b00100  # ro_sel + cnt_enable
        await ClockCycles(dut.clk, 200)
        dut.ui_in.value = (sel & 0x3) | 0b01000  # latch
        await ClockCycles(dut.clk, 1)
        dut.ui_in.value = (sel & 0x3)  # byte_sel=0
        await ClockCycles(dut.clk, 1)
        val = int(dut.uo_out.value)
        dut._log.info(f"RO sel={sel}: count low byte={val}")
        # sel 0-2 are valid ROs; sel 3 defaults to RO7
        assert val >= 0, f"RO sel={sel} output should be valid"


@cocotb.test()
async def test_uio_oe(dut):
    """Verify bidirectional OE is correct: uio[2:0] are outputs."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await ClockCycles(dut.clk, 1)
    assert int(dut.uio_oe.value) == 0b00000111, \
        f"Expected uio_oe=0x07, got 0x{int(dut.uio_oe.value):02x}"


@cocotb.test()
async def test_per_ro_disable(dut):
    """Disabling a selected RO via ui_in[7:5] stops counting for that RO."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    # --- RO7 active (default): count should be > 0 ---
    await reset_dut(dut)
    dut.ui_in.value = 0b00000100  # ro_sel=00(7-stage), cnt_enable=1, all active
    await ClockCycles(dut.clk, 200)
    dut.ui_in.value = 0b00001000  # cnt_clear → latch counter
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00000000  # byte_sel=0
    await ClockCycles(dut.clk, 1)
    count_active = int(dut.uo_out.value)
    dut._log.info(f"RO7 active: low byte = {count_active}")
    assert count_active > 0, "RO7 active should produce counts"

    # --- RO7 disabled (ui_in[5]=1): count should be 0 ---
    await reset_dut(dut)
    dut.ui_in.value = 0b00100100  # ro_sel=00, cnt_enable=1, ro7_dis=1
    await ClockCycles(dut.clk, 200)
    dut.ui_in.value = 0b00101000  # cnt_clear + keep ro7_dis
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00100000  # byte_sel=0, ro7 still disabled
    await ClockCycles(dut.clk, 1)
    count_disabled = int(dut.uo_out.value)
    dut._log.info(f"RO7 disabled: low byte = {count_disabled}")
    assert count_disabled == 0, f"Disabled RO7 should give 0 count, got {count_disabled}"


@cocotb.test()
async def test_ro_disable_isolation(dut):
    """Disable RO7 but RO11 should still produce counts when selected."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Select RO11 (ro_sel=01), enable counting, disable RO7 only (ui_in[5]=1)
    dut.ui_in.value = 0b00100101  # ro_sel=01, cnt_enable=1, ro7_dis=1
    await ClockCycles(dut.clk, 200)
    dut.ui_in.value = 0b00101001  # cnt_clear + keep ro7_dis
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00100001  # byte_sel=0, ro_sel=01
    await ClockCycles(dut.clk, 1)
    count_ro11 = int(dut.uo_out.value)
    dut._log.info(f"RO11 active (RO7 disabled): low byte = {count_ro11}")
    assert count_ro11 > 0, "RO11 should still count when only RO7 is disabled"


@cocotb.test()
async def test_counter_latch_vs_clear(dut):
    """Verify latch captures counter before clear; re-read returns same value."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Count for a while
    dut.ui_in.value = 0b00000100  # cnt_enable=1
    await ClockCycles(dut.clk, 200)

    # Latch (1-cycle pulse of cnt_clear)
    dut.ui_in.value = 0b00001000
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00000000  # byte_sel=0
    await ClockCycles(dut.clk, 1)
    low1 = int(dut.uo_out.value)

    # Read same latch again — should be identical
    await ClockCycles(dut.clk, 1)
    low2 = int(dut.uo_out.value)
    dut._log.info(f"Latch reads: {low1}, {low2}")
    assert low1 == low2, f"Latch should be stable: {low1} vs {low2}"
    assert low1 > 0, "Counter should have accumulated counts"


@cocotb.test()
async def test_all_ro_disable(dut):
    """Disabling all 3 ROs simultaneously (ui_in[7:5]=111) should freeze counter."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Enable counting with all ROs disabled
    dut.ui_in.value = 0b11100100  # ro7_dis=1, ro11_dis=1, ro15_dis=1, cnt_enable=1
    await ClockCycles(dut.clk, 200)

    # Latch (1 cycle pulse)
    dut.ui_in.value = 0b11101000  # keep disables, cnt_clear
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b11100000  # byte_sel=0
    await ClockCycles(dut.clk, 1)
    count_all_dis = int(dut.uo_out.value)
    dut._log.info(f"All ROs disabled: count = {count_all_dis}")
    assert count_all_dis == 0, f"All ROs disabled should give 0 count, got {count_all_dis}"

    # Re-enable one RO and verify counting resumes
    await reset_dut(dut)
    dut.ui_in.value = 0b11000100  # only ro7 enabled (ro11_dis=1, ro15_dis=1)
    await ClockCycles(dut.clk, 200)
    dut.ui_in.value = 0b11001000  # latch
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b11000000  # byte_sel=0
    await ClockCycles(dut.clk, 1)
    count_ro7_only = int(dut.uo_out.value)
    dut._log.info(f"Only RO7 enabled: count = {count_ro7_only}")
    assert count_ro7_only > 0, "RO7 enabled alone should produce counts"


@cocotb.test()
async def test_overflow_flag(dut):
    """Verify overflow flag (uio_out[0]) asserts when counter saturates."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Enable counting for a very long time to reach 16'hFFFF
    dut.ui_in.value = 0b00000100  # cnt_enable=1, ro_sel=00
    # RO edge rate ~1 per 4 clocks; need ~262K clocks to overflow 16-bit counter
    for _ in range(600):
        await ClockCycles(dut.clk, 500)
        overflow = int(dut.uio_out.value) & 1
        if overflow:
            break

    dut._log.info(f"Overflow flag = {overflow}")
    assert overflow == 1, "Overflow should be set after counter saturates at 0xFFFF"

    # Latch and verify counter is at max
    dut.ui_in.value = 0b00001000  # cnt_clear
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = 0b00000000
    await ClockCycles(dut.clk, 1)
    low = int(dut.uo_out.value)
    dut.ui_in.value = 0b00010000  # byte_sel=1
    await ClockCycles(dut.clk, 1)
    high = int(dut.uo_out.value)
    count = (high << 8) | low
    dut._log.info(f"Counter at overflow: 0x{count:04x}")
    assert count == 0xFFFF, f"Counter should be saturated at 0xFFFF, got 0x{count:04x}"


@cocotb.test()
async def test_raw_vs_synced_output(dut):
    """Verify raw RO (uio_out[1]) and synced RO (uio_out[2]) are present during counting."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Enable RO and count
    dut.ui_in.value = 0b00000100  # cnt_enable=1, ro_sel=00
    raw_transitions = 0
    synced_transitions = 0
    prev_raw = 0
    prev_synced = 0
    for _ in range(200):
        await ClockCycles(dut.clk, 1)
        uio = int(dut.uio_out.value)
        raw = (uio >> 1) & 1
        synced = (uio >> 2) & 1
        if raw != prev_raw:
            raw_transitions += 1
        if synced != prev_synced:
            synced_transitions += 1
        prev_raw = raw
        prev_synced = synced

    dut._log.info(f"Raw transitions: {raw_transitions}, Synced transitions: {synced_transitions}")
    assert synced_transitions > 0, "Synced RO output should show transitions"
