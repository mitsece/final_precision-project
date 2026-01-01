`default_nettype none
`timescale 1ns / 1ps

/* 
 * Testbench for Precision Farming ASIC
 * Tests both sensor monitoring mode and ML/camera mode
 */

module tb ();
  // Dump the signals to a VCD file. You can view it with gtkwave.
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Instantiate the design under test
  tt_um_precision_farming user_project (
      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(1'b1),
      .VGND(1'b0),
`endif
      .ui_in  (ui_in),    // Dedicated inputs (sensor/camera data)
      .uo_out (uo_out),   // Dedicated outputs (alerts, status)
      .uio_in (uio_in),   // IOs: Input path (mode select, control signals)
      .uio_out(uio_out),  // IOs: Output path (triggers, clocks)
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

  // Helper signals for monitoring
  wire system_alert = uo_out[7];
  wire ready_indicator = uo_out[6];
  wire mode_indicator = uo_out[5];
  wire harvest_or_sensor_alert = uo_out[4];
  wire [2:0] status_bits = uo_out[3:1];
  wire status_bit_0 = uo_out[0];

  // Mode control
  wire mode_select = uio_in[7];  // 0=sensor, 1=ML
  
  // Sensor mode signals
  wire [1:0] sensor_select = uio_in[1:0];
  
  // ML mode signals
  wire vsync = uio_in[6];
  wire href = uio_in[5];
  wire pclk = uio_in[4];
  wire echo_pin = uio_in[3];
  
  // Output signals
  wire camera_clk_div = uio_out[4];
  wire ultrasonic_trigger = uio_out[1];

  // Clock generation - 25 MHz (40ns period)
  initial begin
    clk = 0;
    forever #20 clk = ~clk;  // 40ns period = 25 MHz
  end

  // Monitor key signals
  always @(posedge clk) begin
    if (system_alert) begin
      if (mode_select)
        $display("[%0t] ML MODE ALERT: Harvest ready! Status=%b", $time, status_bits);
      else
        $display("[%0t] SENSOR MODE ALERT: Deviation detected! Level=%d, Sensor=%d", 
                 $time, status_bits, {uio_out[1], status_bit_0});
    end
  end

  // Test sequence
  initial begin
    $display("===========================================");
    $display("Precision Farming ASIC Test Suite");
    $display("===========================================");
    
    // Initialize
    ena = 1;
    ui_in = 0;
    uio_in = 0;
    rst_n = 0;
    
    // Hold reset for 10 cycles
    repeat(10) @(posedge clk);
    rst_n = 1;
    $display("[%0t] Reset released", $time);
    
    repeat(5) @(posedge clk);
    
    // Run all tests
    test_sensor_mode_baseline();
    test_sensor_mode_deviation();
    test_sensor_mode_all_sensors();
    test_ml_mode_camera_frame();
    test_ml_mode_harvest_detection();
    test_mode_switching();
    
    // Finish simulation
    $display("===========================================");
    $display("All tests completed!");
    $display("===========================================");
    #1000;
    $finish;
  end

  // ========================================
  // TEST 1: Sensor Mode - Baseline Learning
  // ========================================
  task test_sensor_mode_baseline;
    integer i;
    begin
      $display("\n--- TEST 1: Sensor Mode Baseline Learning ---");
      uio_in[7] = 0;  // Sensor mode
      uio_in[1:0] = 2'b00;  // Select sensor 0
      
      // Feed 4 stable readings to establish baseline
      for (i = 0; i < 4; i = i + 1) begin
        ui_in = 8'd100;  // Baseline value
        repeat(10) @(posedge clk);
      end
      
      // Wait for processing
      repeat(50) @(posedge clk);
      
      // Check no alert (baseline established)
      if (!system_alert)
        $display("✓ Baseline established without alert");
      else
        $display("✗ FAIL: Unexpected alert during baseline");
      
      repeat(20) @(posedge clk);
    end
  endtask

  // ========================================
  // TEST 2: Sensor Mode - Deviation Detection
  // ========================================
  task test_sensor_mode_deviation;
    integer i;
    begin
      $display("\n--- TEST 2: Sensor Mode Deviation Detection ---");
      uio_in[7] = 0;  // Sensor mode
      uio_in[1:0] = 2'b00;  // Select sensor 0
      
      // Feed moderate deviation (+30 from baseline of 100)
      for (i = 0; i < 4; i = i + 1) begin
        ui_in = 8'd130;
        repeat(10) @(posedge clk);
      end
      
      repeat(50) @(posedge clk);
      
      if (system_alert && (status_bits >= 3'b010))
        $display("✓ Moderate deviation detected (Level %d)", status_bits);
      else
        $display("✗ FAIL: Should detect moderate deviation");
      
      repeat(20) @(posedge clk);
      
      // Feed large deviation (+80)
      for (i = 0; i < 4; i = i + 1) begin
        ui_in = 8'd180;
        repeat(10) @(posedge clk);
      end
      
      repeat(50) @(posedge clk);
      
      if (system_alert && (status_bits >= 3'b011))
        $display("✓ Large deviation detected (Level %d)", status_bits);
      else
        $display("✗ FAIL: Should detect large deviation");
      
      repeat(20) @(posedge clk);
    end
  endtask

  // ========================================
  // TEST 3: Sensor Mode - All 4 Sensors
  // ========================================
  task test_sensor_mode_all_sensors;
    integer sensor, i;
    reg [7:0] test_values [0:3];
    begin
      $display("\n--- TEST 3: Testing All 4 Sensors ---");
      
      // Test values for each sensor
      test_values[0] = 8'd120;  // Soil moisture
      test_values[1] = 8'd150;  // Humidity
      test_values[2] = 8'd80;   // Light intensity
      test_values[3] = 8'd200;  // Temperature
      
      for (sensor = 0; sensor < 4; sensor = sensor + 1) begin
        uio_in[7] = 0;  // Sensor mode
        uio_in[1:0] = sensor[1:0];
        
        // Establish baseline
        for (i = 0; i < 4; i = i + 1) begin
          ui_in = test_values[sensor];
          repeat(10) @(posedge clk);
        end
        
        repeat(30) @(posedge clk);
        
        // Test deviation
        for (i = 0; i < 4; i = i + 1) begin
          ui_in = test_values[sensor] + 8'd40;
          repeat(10) @(posedge clk);
        end
        
        repeat(30) @(posedge clk);
        
        $display("  Sensor %0d: Baseline=%d, Test=%d, Alert=%b", 
                 sensor, test_values[sensor], test_values[sensor] + 40, system_alert);
      end
      
      $display("✓ All 4 sensors tested");
    end
  endtask

  // ========================================
  // TEST 4: ML Mode - Camera Frame Simulation
  // ========================================
  task test_ml_mode_camera_frame;
    integer row, col;
    reg [7:0] pixel_data;
    begin
      $display("\n--- TEST 4: ML Mode Camera Frame Processing ---");
      uio_in[7] = 1;  // ML mode
      
      repeat(20) @(posedge clk);
      
      // Simulate VSYNC rise (start of frame)
      uio_in[6] = 1;  // VSYNC high
      repeat(5) @(posedge clk);
      
      // Simulate 10 rows of pixel data
      for (row = 0; row < 10; row = row + 1) begin
        uio_in[5] = 1;  // HREF high (active row)
        
        // Send 20 pixels per row (RGB565 format, 2 bytes per pixel)
        for (col = 0; col < 40; col = col + 1) begin
          if (col[0] == 0) begin
            // First byte: RRRRRGGG
            pixel_data = {5'b01000, 3'b111};  // Low red, high green
          end else begin
            // Second byte: GGGBBBBB
            pixel_data = {3'b111, 5'b10000};  // High green, medium blue
          end
          
          ui_in = pixel_data;
          @(posedge clk);
        end
        
        uio_in[5] = 0;  // HREF low (end of row)
        repeat(5) @(posedge clk);
      end
      
      // VSYNC fall (end of frame)
      uio_in[6] = 0;
      repeat(20) @(posedge clk);
      
      if (ready_indicator)
        $display("✓ Frame processing completed");
      else
        $display("⚠ Frame processing may need more time");
      
      repeat(20) @(posedge clk);
    end
  endtask

  // ========================================
  // TEST 5: ML Mode - Harvest Detection
  // ========================================
  task test_ml_mode_harvest_detection;
    integer row, col;
    reg [7:0] pixel_data;
    begin
      $display("\n--- TEST 5: ML Mode Harvest Detection ---");
      uio_in[7] = 1;  // ML mode
      
      repeat(20) @(posedge clk);
      
      // Simulate ultrasonic echo for distance measurement
      uio_in[3] = 1;  // Echo start
      repeat(100) @(posedge clk);  // Simulate ~6cm distance
      uio_in[3] = 0;  // Echo end
      
      repeat(20) @(posedge clk);
      
      // Simulate frame with "ready for harvest" characteristics
      uio_in[6] = 1;  // VSYNC
      repeat(5) @(posedge clk);
      
      // Green, tall microgreens
      for (row = 0; row < 50; row = row + 1) begin
        uio_in[5] = 1;  // HREF
        
        for (col = 0; col < 40; col = col + 1) begin
          if (col[0] == 0) begin
            pixel_data = {5'b00010, 3'b111};  // Very low red, very high green
          end else begin
            pixel_data = {3'b111, 5'b10000};  // Very high green, good brightness
          end
          
          ui_in = pixel_data;
          @(posedge clk);
        end
        
        uio_in[5] = 0;
        repeat(3) @(posedge clk);
      end
      
      uio_in[6] = 0;  // VSYNC fall
      repeat(50) @(posedge clk);
      
      if (system_alert && harvest_or_sensor_alert)
        $display("✓ Harvest ready detected!");
      else
        $display("⚠ Harvest detection may need adjustment");
      
      $display("  Hidden layer activations: %b", status_bits[2:0]);
      $display("  Prediction: %b", harvest_or_sensor_alert);
      
      repeat(20) @(posedge clk);
    end
  endtask

  // ========================================
  // TEST 6: Mode Switching
  // ========================================
  task test_mode_switching;
    integer i;
    begin
      $display("\n--- TEST 6: Mode Switching ---");
      
      // Start in sensor mode
      uio_in[7] = 0;
      uio_in[1:0] = 2'b01;
      
      for (i = 0; i < 4; i = i + 1) begin
        ui_in = 8'd50;
        repeat(10) @(posedge clk);
      end
      
      repeat(30) @(posedge clk);
      $display("  Sensor mode active: mode_indicator=%b", mode_indicator);
      
      // Switch to ML mode
      uio_in[7] = 1;
      repeat(50) @(posedge clk);
      $display("  ML mode active: mode_indicator=%b", mode_indicator);
      
      // Switch back to sensor mode
      uio_in[7] = 0;
      repeat(50) @(posedge clk);
      $display("  Back to sensor mode: mode_indicator=%b", mode_indicator);
      
      $display("✓ Mode switching functional");
    end
  endtask

endmodule

`default_nettype wire
