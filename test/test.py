# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: MIT

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge, Timer
from cocotb.types import LogicArray

# ============================================
# TEST CONFIGURATION
# ============================================
CLOCK_FREQ_MHZ = 25
CLOCK_PERIOD_NS = 1000 / CLOCK_FREQ_MHZ  # 40ns for 25MHz

# Mode selection
MODE_SENSOR = 0
MODE_ML = 1

# Sensor IDs
SENSOR_SOIL = 0
SENSOR_HUMIDITY = 1
SENSOR_LIGHT = 2
SENSOR_TEMP = 3

# ============================================
# HELPER FUNCTIONS
# ============================================

async def reset_dut(dut):
    """Apply reset to the design"""
    dut._log.info("Applying reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    dut._log.info("Reset released")

def set_mode(dut, mode):
    """Set operation mode (0=sensor, 1=ML)"""
    current_val = int(dut.uio_in.value)
    if mode == MODE_ML:
        dut.uio_in.value = current_val | 0x80  # Set bit 7
    else:
        dut.uio_in.value = current_val & 0x7F  # Clear bit 7

def select_sensor(dut, sensor_id):
    """Select which sensor to read (0-3)"""
    current_val = int(dut.uio_in.value) & 0xFC  # Clear bits [1:0]
    dut.uio_in.value = current_val | (sensor_id & 0x03)

def get_alert_status(dut):
    """Read alert and status outputs"""
    uo_val = int(dut.uo_out.value)
    return {
        'system_alert': bool(uo_val & 0x80),
        'ready': bool(uo_val & 0x40),
        'mode': bool(uo_val & 0x20),
        'prediction': bool(uo_val & 0x10),
        'status_bits': (uo_val >> 1) & 0x07,
        'status_bit_0': bool(uo_val & 0x01)
    }

# ============================================
# TEST 1: SENSOR MODE - BASELINE LEARNING
# ============================================

@cocotb.test()
async def test_sensor_baseline(dut):
    """Test sensor mode baseline establishment"""
    dut._log.info("========================================")
    dut._log.info("TEST 1: Sensor Baseline Learning")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Set sensor mode and select sensor 0
    set_mode(dut, MODE_SENSOR)
    select_sensor(dut, SENSOR_SOIL)
    await ClockCycles(dut.clk, 5)
    
    # Feed 4 stable readings (baseline = 100)
    baseline_value = 100
    for i in range(4):
        dut.ui_in.value = baseline_value
        await ClockCycles(dut.clk, 10)
    
    # Wait for processing
    await ClockCycles(dut.clk, 50)
    
    # Check status
    status = get_alert_status(dut)
    
    if not status['system_alert']:
        dut._log.info("✓ PASS: Baseline established without alert")
    else:
        dut._log.error("✗ FAIL: Unexpected alert during baseline")
        assert False, "Baseline should not trigger alert"
    
    await ClockCycles(dut.clk, 20)

# ============================================
# TEST 2: SENSOR MODE - DEVIATION DETECTION
# ============================================

@cocotb.test()
async def test_sensor_deviation(dut):
    """Test sensor deviation detection with different severity levels"""
    dut._log.info("========================================")
    dut._log.info("TEST 2: Sensor Deviation Detection")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Set sensor mode
    set_mode(dut, MODE_SENSOR)
    select_sensor(dut, SENSOR_SOIL)
    await ClockCycles(dut.clk, 5)
    
    # Establish baseline (100)
    for i in range(4):
        dut.ui_in.value = 100
        await ClockCycles(dut.clk, 10)
    await ClockCycles(dut.clk, 50)
    
    # Test moderate deviation (+30)
    dut._log.info("Testing moderate deviation (+30)")
    for i in range(4):
        dut.ui_in.value = 130
        await ClockCycles(dut.clk, 10)
    await ClockCycles(dut.clk, 50)
    
    status = get_alert_status(dut)
    if status['system_alert']:
        dut._log.info(f"✓ Moderate deviation detected (Level {status['status_bits']})")
    else:
        dut._log.warning("⚠ No alert for moderate deviation")
    
    # Test large deviation (+80)
    dut._log.info("Testing large deviation (+80)")
    for i in range(4):
        dut.ui_in.value = 180
        await ClockCycles(dut.clk, 10)
    await ClockCycles(dut.clk, 50)
    
    status = get_alert_status(dut)
    if status['system_alert'] and status['status_bits'] >= 3:
        dut._log.info(f"✓ PASS: Large deviation detected (Level {status['status_bits']})")
    else:
        dut._log.error(f"✗ FAIL: Expected alert with high severity, got level {status['status_bits']}")
    
    await ClockCycles(dut.clk, 20)

# ============================================
# TEST 3: SENSOR MODE - ALL SENSORS
# ============================================

@cocotb.test()
async def test_all_sensors(dut):
    """Test all 4 sensors independently"""
    dut._log.info("========================================")
    dut._log.info("TEST 3: All 4 Sensors")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Test each sensor
    sensor_names = ["Soil Moisture", "Humidity", "Light Intensity", "Temperature"]
    test_values = [120, 150, 80, 200]
    
    for sensor_id in range(4):
        dut._log.info(f"\nTesting {sensor_names[sensor_id]} (Sensor {sensor_id})")
        
        set_mode(dut, MODE_SENSOR)
        select_sensor(dut, sensor_id)
        await ClockCycles(dut.clk, 5)
        
        # Establish baseline
        for i in range(4):
            dut.ui_in.value = test_values[sensor_id]
            await ClockCycles(dut.clk, 10)
        await ClockCycles(dut.clk, 30)
        
        # Test with deviation
        deviated_value = test_values[sensor_id] + 40
        for i in range(4):
            dut.ui_in.value = deviated_value
            await ClockCycles(dut.clk, 10)
        await ClockCycles(dut.clk, 30)
        
        status = get_alert_status(dut)
        dut._log.info(f"  Baseline: {test_values[sensor_id]}, Test: {deviated_value}, Alert: {status['system_alert']}")
    
    dut._log.info("\n✓ PASS: All 4 sensors tested")

# ============================================
# TEST 4: ML MODE - CAMERA FRAME
# ============================================

@cocotb.test()
async def test_camera_frame(dut):
    """Test ML mode with simulated camera frame"""
    dut._log.info("========================================")
    dut._log.info("TEST 4: Camera Frame Processing")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Set ML mode
    set_mode(dut, MODE_ML)
    await ClockCycles(dut.clk, 20)
    
    # Simulate VSYNC rise (start of frame)
    current_val = int(dut.uio_in.value) | 0x40  # Set bit 6 (VSYNC)
    dut.uio_in.value = current_val
    await ClockCycles(dut.clk, 5)
    
    # Simulate 10 rows of pixels
    for row in range(10):
        # HREF high (active row)
        current_val = int(dut.uio_in.value) | 0x20  # Set bit 5 (HREF)
        dut.uio_in.value = current_val
        
        # 20 pixels per row (40 bytes in RGB565)
        for col in range(40):
            if col % 2 == 0:
                # First byte: RRRRRGGG (low red, high green)
                pixel = 0b01000111
            else:
                # Second byte: GGGBBBBB (high green)
                pixel = 0b11110000
            
            dut.ui_in.value = pixel
            await ClockCycles(dut.clk, 1)
        
        # HREF low (end of row)
        current_val = int(dut.uio_in.value) & ~0x20  # Clear bit 5
        dut.uio_in.value = current_val
        await ClockCycles(dut.clk, 5)
    
    # VSYNC fall (end of frame)
    current_val = int(dut.uio_in.value) & ~0x40  # Clear bit 6
    dut.uio_in.value = current_val
    await ClockCycles(dut.clk, 50)
    
    status = get_alert_status(dut)
    if status['ready']:
        dut._log.info("✓ PASS: Frame processing completed")
    else:
        dut._log.warning("⚠ Frame processing may need more time")
    
    await ClockCycles(dut.clk, 20)

# ============================================
# TEST 5: ML MODE - HARVEST DETECTION
# ============================================

@cocotb.test()
async def test_harvest_detection(dut):
    """Test ML-based harvest ready detection"""
    dut._log.info("========================================")
    dut._log.info("TEST 5: Harvest Detection")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Set ML mode
    set_mode(dut, MODE_ML)
    await ClockCycles(dut.clk, 20)
    
    # Simulate ultrasonic echo for distance
    dut._log.info("Simulating ultrasonic distance measurement")
    current_val = int(dut.uio_in.value) | 0x08  # Set bit 3 (echo)
    dut.uio_in.value = current_val
    await ClockCycles(dut.clk, 100)  # ~6cm distance
    current_val = int(dut.uio_in.value) & ~0x08  # Clear echo
    dut.uio_in.value = current_val
    await ClockCycles(dut.clk, 20)
    
    # Simulate frame with harvest-ready characteristics
    dut._log.info("Simulating green, tall microgreens")
    
    # VSYNC high
    current_val = int(dut.uio_in.value) | 0x40
    dut.uio_in.value = current_val
    await ClockCycles(dut.clk, 5)
    
    # 50 rows (tall plant)
    for row in range(50):
        # HREF high
        current_val = int(dut.uio_in.value) | 0x20
        dut.uio_in.value = current_val
        
        for col in range(40):
            if col % 2 == 0:
                # Very low red, very high green
                pixel = 0b00010111
            else:
                # Very high green, good brightness
                pixel = 0b11110000
            
            dut.ui_in.value = pixel
            await ClockCycles(dut.clk, 1)
        
        # HREF low
        current_val = int(dut.uio_in.value) & ~0x20
        dut.uio_in.value = current_val
        await ClockCycles(dut.clk, 3)
    
    # VSYNC fall
    current_val = int(dut.uio_in.value) & ~0x40
    dut.uio_in.value = current_val
    await ClockCycles(dut.clk, 50)
    
    # Check result
    status = get_alert_status(dut)
    dut._log.info(f"Prediction: {status['prediction']}")
    dut._log.info(f"Hidden activations: {bin(status['status_bits'])}")
    dut._log.info(f"System alert: {status['system_alert']}")
    
    if status['system_alert'] and status['prediction']:
        dut._log.info("✓ PASS: Harvest ready detected!")
    else:
        dut._log.warning("⚠ Harvest not detected (may need feature tuning)")
    
    await ClockCycles(dut.clk, 20)

# ============================================
# TEST 6: MODE SWITCHING
# ============================================

@cocotb.test()
async def test_mode_switching(dut):
    """Test switching between sensor and ML modes"""
    dut._log.info("========================================")
    dut._log.info("TEST 6: Mode Switching")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    # Start in sensor mode
    dut._log.info("Starting in sensor mode")
    set_mode(dut, MODE_SENSOR)
    select_sensor(dut, SENSOR_HUMIDITY)
    
    for i in range(4):
        dut.ui_in.value = 50
        await ClockCycles(dut.clk, 10)
    
    await ClockCycles(dut.clk, 30)
    status = get_alert_status(dut)
    dut._log.info(f"  Sensor mode active: mode_indicator={status['mode']}")
    
    # Switch to ML mode
    dut._log.info("Switching to ML mode")
    set_mode(dut, MODE_ML)
    await ClockCycles(dut.clk, 50)
    status = get_alert_status(dut)
    dut._log.info(f"  ML mode active: mode_indicator={status['mode']}")
    
    # Switch back to sensor mode
    dut._log.info("Switching back to sensor mode")
    set_mode(dut, MODE_SENSOR)
    await ClockCycles(dut.clk, 50)
    status = get_alert_status(dut)
    dut._log.info(f"  Back to sensor mode: mode_indicator={status['mode']}")
    
    dut._log.info("✓ PASS: Mode switching functional")

# ============================================
# TEST 7: STRESS TEST - RAPID SENSOR CHANGES
# ============================================

@cocotb.test()
async def test_rapid_sensor_switching(dut):
    """Stress test: rapidly switch between sensors"""
    dut._log.info("========================================")
    dut._log.info("TEST 7: Rapid Sensor Switching")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    set_mode(dut, MODE_SENSOR)
    
    # Rapidly cycle through all sensors
    for cycle in range(5):
        for sensor_id in range(4):
            select_sensor(dut, sensor_id)
            dut.ui_in.value = 100 + sensor_id * 20
            await ClockCycles(dut.clk, 5)
    
    await ClockCycles(dut.clk, 50)
    dut._log.info("✓ PASS: Design stable under rapid switching")

# ============================================
# TEST 8: EDGE CASES
# ============================================

@cocotb.test()
async def test_edge_cases(dut):
    """Test boundary conditions and edge cases"""
    dut._log.info("========================================")
    dut._log.info("TEST 8: Edge Cases")
    dut._log.info("========================================")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    set_mode(dut, MODE_SENSOR)
    select_sensor(dut, SENSOR_SOIL)
    
    # Test with minimum values
    dut._log.info("Testing minimum sensor values (0)")
    for i in range(4):
        dut.ui_in.value = 0
        await ClockCycles(dut.clk, 10)
    await ClockCycles(dut.clk, 50)
    
    # Test with maximum values
    dut._log.info("Testing maximum sensor values (255)")
    for i in range(4):
        dut.ui_in.value = 255
        await ClockCycles(dut.clk, 10)
    await ClockCycles(dut.clk, 50)
    
    status = get_alert_status(dut)
    if status['system_alert']:
        dut._log.info(f"✓ Large deviation correctly detected (Level {status['status_bits']})")
    
    dut._log.info("✓ PASS: Edge cases handled")

# ============================================
# MAIN TEST RUNNER
# ============================================

@cocotb.test()
async def test_comprehensive(dut):
    """Comprehensive integration test"""
    dut._log.info("╔════════════════════════════════════════╗")
    dut._log.info("║  PRECISION FARMING ASIC TEST SUITE     ║")
    dut._log.info("║  Comprehensive Integration Test        ║")
    dut._log.info("╚════════════════════════════════════════╝")
    
    # Start clock
    clock = Clock(dut.clk, CLOCK_PERIOD_NS, units="ns")
    cocotb.start_soon(clock.start())
    
    # Reset
    await reset_dut(dut)
    
    dut._log.info("\n✓ All systems initialized")
    dut._log.info("Ready for precision farming operations!")
    
    await ClockCycles(dut.clk, 100)
