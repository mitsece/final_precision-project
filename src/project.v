// SPDX-FileCopyrightText: © 2024 Tiny Tapeout
// SPDX-License-Identifier: Apache-2.0

// Integrated Precision Farming ASIC
// Combines ML-based growth detection with environmental monitoring

`default_nettype none

module tt_um_precision_farming (
    input  wire [7:0] ui_in,    // Sensor data input / Camera data
    output wire [7:0] uo_out,   // Status outputs
    input  wire [7:0] uio_in,   // Control signals
    output wire [7:0] uio_out,  // Output signals
    output wire [7:0] uio_oe,   // I/O enable
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // ========================================
    // MODE SELECTION
    // ========================================
    // uio_in[7] = mode select: 0=sensor monitoring, 1=camera/ML mode
    wire mode_select = uio_in[7];
    
    // Configure bidirectional pins
    // In sensor mode: mostly inputs
    // In camera mode: camera control outputs
    assign uio_oe = mode_select ? 8'b00011110 : 8'b00000011;

    // ========================================
    // SENSOR MONITORING SUBSYSTEM
    // ========================================
    wire [7:0] sensor_data = ui_in;
    wire [1:0] sensor_select = uio_in[1:0];  // Which sensor to read
    
    // Sensor baseline storage (moving averages)
    reg [9:0] sensor_baseline[0:3];  // 4 sensors
    reg [9:0] sensor_accumulator;
    reg [2:0] sample_count;
    reg [9:0] current_reading;
    reg baseline_set[0:3];
    
    // Deviation and fault detection
    reg [9:0] deviation;
    reg [2:0] fault_level;  // 0=OK, 1-4=severity
    reg [1:0] fault_sensor;
    reg sensor_alert;
    
    // Sensor monitoring state machine
    reg [2:0] sensor_state;
    localparam SENSOR_IDLE = 0, SENSOR_ACCUMULATE = 1, 
               SENSOR_COMPUTE = 2, SENSOR_CHECK = 3;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_accumulator <= 0;
            sample_count <= 0;
            sensor_alert <= 0;
            fault_level <= 0;
            fault_sensor <= 0;
            sensor_state <= SENSOR_IDLE;
            for (integer i = 0; i < 4; i = i + 1) begin
                sensor_baseline[i] <= 0;
                baseline_set[i] <= 0;
            end
        end else if (ena && !mode_select) begin
            case (sensor_state)
                SENSOR_IDLE: begin
                    sensor_accumulator <= {2'b0, sensor_data};
                    sample_count <= 1;
                    sensor_state <= SENSOR_ACCUMULATE;
                end
                
                SENSOR_ACCUMULATE: begin
                    if (sample_count < 4) begin
                        sensor_accumulator <= sensor_accumulator + {2'b0, sensor_data};
                        sample_count <= sample_count + 1;
                    end else begin
                        sensor_state <= SENSOR_COMPUTE;
                    end
                end
                
                SENSOR_COMPUTE: begin
                    current_reading <= sensor_accumulator >> 2;  // Average of 4 samples
                    sensor_state <= SENSOR_CHECK;
                end
                
                SENSOR_CHECK: begin
                    if (!baseline_set[sensor_select]) begin
                        // First reading becomes baseline
                        sensor_baseline[sensor_select] <= current_reading;
                        baseline_set[sensor_select] <= 1;
                        sensor_alert <= 0;
                    end else begin
                        // Calculate deviation
                        if (current_reading > sensor_baseline[sensor_select])
                            deviation <= current_reading - sensor_baseline[sensor_select];
                        else
                            deviation <= sensor_baseline[sensor_select] - current_reading;
                        
                        // Classify fault severity
                        if (deviation >= 100) begin
                            fault_level <= 4;
                            fault_sensor <= sensor_select;
                            sensor_alert <= 1;
                            sensor_baseline[sensor_select] <= current_reading; // Update baseline
                        end else if (deviation >= 50) begin
                            fault_level <= 3;
                            fault_sensor <= sensor_select;
                            sensor_alert <= 1;
                            sensor_baseline[sensor_select] <= current_reading;
                        end else if (deviation >= 25) begin
                            fault_level <= 2;
                            fault_sensor <= sensor_select;
                            sensor_alert <= 1;
                            sensor_baseline[sensor_select] <= current_reading;
                        end else if (deviation >= 10) begin
                            fault_level <= 1;
                            fault_sensor <= sensor_select;
                            sensor_alert <= 1;
                            sensor_baseline[sensor_select] <= current_reading;
                        end else begin
                            fault_level <= 0;
                            sensor_alert <= 0;
                        end
                    end
                    sensor_state <= SENSOR_IDLE;
                end
            endcase
        end
    end

    // ========================================
    // CAMERA & ML SUBSYSTEM
    // ========================================
    wire [7:0] camera_data = ui_in;
    wire vsync = uio_in[6];
    wire href = uio_in[5];
    wire pclk = uio_in[4];
    wire echo_pin = uio_in[3];
    
    // Camera clock output
    reg camera_clk_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) camera_clk_div <= 0;
        else if (ena && mode_select) camera_clk_div <= ~camera_clk_div;
    end
    
    // Ultrasonic trigger
    reg ultrasonic_trigger;
    reg [19:0] trigger_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ultrasonic_trigger <= 0;
            trigger_counter <= 0;
        end else if (ena && mode_select) begin
            trigger_counter <= trigger_counter + 1;
            ultrasonic_trigger <= (trigger_counter < 250);
            if (trigger_counter >= 1500000) trigger_counter <= 0;
        end
    end
    
    // Feature extraction from camera
    reg [15:0] green_acc, red_acc, brightness_acc, pixel_count;
    reg [7:0] max_row, min_row;
    reg [7:0] avg_green, avg_red, avg_brightness, height_pixels;
    reg vsync_prev, href_prev;
    reg [8:0] row_counter;
    reg [9:0] col_counter;
    reg frame_ready;
    
    wire vsync_rise = vsync && !vsync_prev;
    wire vsync_fall = !vsync && vsync_prev;
    wire href_rise = href && !href_prev;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            green_acc <= 0; red_acc <= 0; brightness_acc <= 0;
            pixel_count <= 0; row_counter <= 0; col_counter <= 0;
            max_row <= 0; min_row <= 255;
            vsync_prev <= 0; href_prev <= 0;
            frame_ready <= 0;
        end else if (ena && mode_select) begin
            vsync_prev <= vsync; href_prev <= href;
            
            if (vsync_rise) begin
                green_acc <= 0; red_acc <= 0; brightness_acc <= 0;
                pixel_count <= 0; row_counter <= 0;
                max_row <= 0; min_row <= 255;
            end
            
            if (href_rise) begin
                row_counter <= row_counter + 1;
                col_counter <= 0;
            end
            
            if (href) begin
                col_counter <= col_counter + 1;
                if (col_counter[0] == 0) begin
                    red_acc <= red_acc + {camera_data[7:3], 3'b0};
                    green_acc <= green_acc + {camera_data[2:0], 5'b0};
                end else begin
                    green_acc <= green_acc + {camera_data[7:5], 5'b0};
                    brightness_acc <= brightness_acc + camera_data;
                    pixel_count <= pixel_count + 1;
                    
                    // Track height (rows with green content)
                    if (camera_data[7:5] > 3'b100) begin
                        if (row_counter < min_row) min_row <= row_counter[7:0];
                        if (row_counter > max_row) max_row <= row_counter[7:0];
                    end
                end
            end
            
            if (vsync_fall && pixel_count > 0) begin
                avg_green <= green_acc[15:8];
                avg_red <= red_acc[15:8];
                avg_brightness <= brightness_acc[15:8];
                height_pixels <= max_row - min_row;
                frame_ready <= 1;
            end else begin
                frame_ready <= 0;
            end
        end
    end
    
    // Ultrasonic distance measurement
    reg [15:0] echo_timer;
    reg [7:0] distance_cm;
    reg measuring;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            echo_timer <= 0; distance_cm <= 0; measuring <= 0;
        end else if (ena && mode_select) begin
            if (echo_pin && !measuring) begin
                measuring <= 1;
                echo_timer <= 0;
            end else if (measuring) begin
                if (echo_pin) echo_timer <= echo_timer + 1;
                else begin
                    measuring <= 0;
                    distance_cm <= echo_timer[15:10];
                end
            end
        end
    end
    
    // ========================================
    // BINARY NEURAL NETWORK (BNN)
    // ========================================
    // Weight parameters
    localparam [3:0] W_IH_0 = 4'b1001;
    localparam [3:0] W_IH_1 = 4'b1011;
    localparam [3:0] W_IH_2 = 4'b1100;
    localparam [3:0] W_IH_3 = 4'b1110;
    localparam [3:0] W_HO_0 = 4'b1010;
    localparam [3:0] W_HO_1 = 4'b0101;
    localparam signed [3:0] BIAS_H0 = 4'sd1;
    localparam signed [3:0] BIAS_H1 = 4'sd1;
    localparam signed [3:0] BIAS_H2 = -4'sd1;
    localparam signed [3:0] BIAS_H3 = 4'sd1;
    
    // Feature extraction
    wire [3:0] feature_greenness = avg_green[7:4];
    wire [3:0] feature_color_ratio = (avg_green[7:4] > avg_red[7:4]) ?
                                      (avg_green[7:4] - avg_red[7:4]) : 4'd0;
    wire [3:0] feature_height = height_pixels[7:4];
    wire [3:0] feature_distance = distance_cm[7:4];
    wire [3:0] feature_combined_height = (feature_height + feature_distance) >> 1;
    
    // Binary input features
    wire [3:0] input_binary = {
        (feature_combined_height > 4'd7),
        (avg_brightness[7:4] > 4'd7),
        (feature_color_ratio > 4'd3),
        (feature_greenness > 4'd7)
    };
    
    // XNOR popcount function for BNN
    function signed [4:0] xnor_popcount_4bit(input [3:0] a, b);
        reg [3:0] x;
        begin
            x = ~(a ^ b);
            xnor_popcount_4bit = x[0] + x[1] + x[2] + x[3];
        end
    endfunction
    
    // Hidden layer computation
    wire signed [4:0] hidden_sum[0:3];
    assign hidden_sum[0] = xnor_popcount_4bit(input_binary, W_IH_0) + BIAS_H0;
    assign hidden_sum[1] = xnor_popcount_4bit(input_binary, W_IH_1) + BIAS_H1;
    assign hidden_sum[2] = xnor_popcount_4bit(input_binary, W_IH_2) + BIAS_H2;
    assign hidden_sum[3] = xnor_popcount_4bit(input_binary, W_IH_3) + BIAS_H3;
    
    reg [3:0] hidden;
    reg bnn_ready;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hidden <= 0;
            bnn_ready <= 0;
        end else if (ena && mode_select && frame_ready) begin
            hidden[0] <= (hidden_sum[0] >= 0);
            hidden[1] <= (hidden_sum[1] >= 0);
            hidden[2] <= (hidden_sum[2] >= 0);
            hidden[3] <= (hidden_sum[3] >= 0);
            bnn_ready <= 1;
        end
    end
    
    // Output layer
    wire signed [4:0] output_sum[0:1];
    assign output_sum[0] = xnor_popcount_4bit(hidden, W_HO_0);
    assign output_sum[1] = xnor_popcount_4bit(hidden, W_HO_1);
    
    wire harvest_ready = (output_sum[1] > output_sum[0]);
    
    // ========================================
    // INTEGRATED DECISION LOGIC
    // ========================================
    // Combine ML prediction with sensor status
    wire system_alert = (mode_select ? harvest_ready : sensor_alert);
    wire [2:0] alert_level = mode_select ? {2'b11, harvest_ready} : {1'b0, fault_level[1:0]};
    
    // ========================================
    // OUTPUT ASSIGNMENT
    // ========================================
    // uo_out assignments
    assign uo_out[7] = system_alert;        // Main alert/buzzer
    assign uo_out[6] = bnn_ready | (sensor_state == SENSOR_CHECK);  // Ready indicator
    assign uo_out[5] = mode_select;         // Mode indicator
    assign uo_out[4] = mode_select ? harvest_ready : sensor_alert;
    assign uo_out[3:1] = mode_select ? hidden[2:0] : fault_level;
    assign uo_out[0] = mode_select ? hidden[3] : fault_sensor[0];
    
    // uio_out assignments
    assign uio_out[7] = 1'b0;
    assign uio_out[6] = 1'b0;
    assign uio_out[5] = 1'b0;
    assign uio_out[4] = mode_select ? camera_clk_div : 1'b0;
    assign uio_out[3] = 1'b0;
    assign uio_out[2] = 1'b0;
    assign uio_out[1] = mode_select ? ultrasonic_trigger : fault_sensor[1];
    assign uio_out[0] = 1'b0;

endmodule

`default_nettype wire
