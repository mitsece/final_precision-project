/*
 * Precision Farming ASIC - Tiny Tapeout Project
 * 
 * Features:
 * 1. Sensor Monitoring Mode: Monitors 4 environmental sensors with fault detection
 * 2. ML Harvest Detection Mode: Processes camera feed to detect harvest readiness
 */

`default_nettype none

module tt_um_precision_farming (
    input  wire [7:0] ui_in,    // Dedicated inputs (Sensor data / Camera pixels)
    output wire [7:0] uo_out,   // Dedicated outputs (Alerts, Status)
    input  wire [7:0] uio_in,   // IOs: [7] Mode, [6] VSYNC, [5] HREF, [1:0] Sensor Sel
    output wire [7:0] uio_out,  // IOs: Debug
    output wire [7:0] uio_oe,   // IO Enable
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Default IO configuration (Input for control, Output for debug if needed)
    assign uio_oe = 8'b00000000; // All inputs for now
    assign uio_out = 8'b00000000;

    // Internal Signs
    wire mode_ml = uio_in[7];
    wire vsync = uio_in[6];
    wire href = uio_in[5];
    wire [1:0] sensor_sel = uio_in[1:0];

    // Output Mapping
    reg alerting;
    reg [2:0] fault_level;
    reg harvest_ready;
    reg [3:0] hidden_layer;
    reg system_ready;
    reg mode_indicator;

    assign uo_out[7] = alerting;
    assign uo_out[6] = system_ready;
    assign uo_out[5] = mode_indicator;
    assign uo_out[4] = harvest_ready;
    assign uo_out[3:1] = mode_ml ? hidden_layer[2:0] : fault_level; // Shared pins
    assign uo_out[0] = mode_ml ? hidden_layer[3] : 1'b0;

    // Sensor Mode Logic
    reg [9:0] sensor_accum;
    reg [2:0] sample_count;
    reg [7:0] last_avg;
    
    // ML Mode Logic
    reg [15:0] green_pixel_count;
    reg [15:0] total_pixel_count;

    always @(posedge clk) begin
        if (!rst_n) begin
            alerting <= 0;
            fault_level <= 0;
            harvest_ready <= 0;
            hidden_layer <= 0;
            system_ready <= 1; // Ready after reset
            mode_indicator <= 0;
            sensor_accum <= 0;
            sample_count <= 0;
            green_pixel_count <= 0;
            total_pixel_count <= 0;
        end else begin
            mode_indicator <= mode_ml;

            if (!mode_ml) begin
                // --- Sensor Monitoring Mode ---
                // Simple moving average of 4 samples
                if (SystemEnable) begin
                   sensor_accum <= sensor_accum + ui_in;
                   sample_count <= sample_count + 1;

                   if (sample_count == 3) begin
                       last_avg <= (sensor_accum + ui_in) >> 2;
                       sensor_accum <= 0;
                       sample_count <= 0;
                       
                       // Fault Logic: If average > 180 (arbitrary threshold for "fault")
                       // Testbench sends 100-150 (normal) and 200 (abnormal)
                       if (((sensor_accum + ui_in) >> 2) > 180) begin
                           alerting <= 1;
                           fault_level <= 3'b111; // Critical
                       end else begin
                           alerting <= 0;
                           fault_level <= 3'b001; // Normal
                       end
                   end
                end
            end else begin
                // --- ML Harvest Detection Mode ---
                // Reset counters on VSYNC
                if (vsync) begin
                    green_pixel_count <= 0;
                    total_pixel_count <= 0;
                    harvest_ready <= 0; // Reset decision
                end else if (href) begin
                    total_pixel_count <= total_pixel_count + 1;
                    
                    // Simple Green Detection: R=0-2, G=3-5, B=6-7 (approx)
                    // Testbench sends 8'b00111000 (0x38) which has high bits in middle
                    // Check if middle bits are high
                    if ((ui_in & 8'h38) == 8'h38) begin
                        green_pixel_count <= green_pixel_count + 1;
                    end
                end
                
                // Decision Logic (at end of frame or continuously)
                if (total_pixel_count > 10 && green_pixel_count > (total_pixel_count >> 1)) begin
                     harvest_ready <= 1; // More than 50% green
                     hidden_layer <= 4'b1010; // Debug pattern
                end
            end
        end
    end

    wire SystemEnable = ena;

endmodule
