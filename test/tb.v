`timescale 1ns / 1ps

module tb;
    // Clock and reset
    reg clk;
    reg rst_n;
    
    // Test signals
    reg [7:0] test_input;
    wire [7:0] test_output;
    reg test_passed;
    
    // Random seed from environment
    integer seed;
    
    // DUT instantiation (replace with your actual module)
    // your_module dut (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .input_data(test_input),
    //     .output_data(test_output)
    // );
    
    // Clock generation (10ns period = 100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // VCD dump for waveform viewing
    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb.vcd");
            $dumpvars(0, tb);
        end
    end
    
    // Get random seed from environment or use default
    initial begin
        if ($value$plusargs("RANDOM_SEED=%d", seed)) begin
            $display("Using RANDOM_SEED=%0d", seed);
        end else begin
            seed = 42; // Default seed for reproducibility
            $display("Using default RANDOM_SEED=%0d", seed);
        end
        $random(seed);
    end
    
    // Main test sequence
    initial begin
        test_passed = 1;
        
        // Initialize
        rst_n = 0;
        test_input = 0;
        
        // Wait for reset
        repeat(10) @(posedge clk);
        rst_n = 1;
        
        $display("========================================");
        $display("Starting testbench at time %0t", $time);
        $display("========================================");
        
        // Test 1: Basic functionality
        repeat(100) begin
            @(posedge clk);
            test_input = $random;
            
            // Add your test assertions here
            // Example:
            // if (expected_output !== test_output) begin
            //     $error("Test failed at time %0t: expected=%h, got=%h", 
            //            $time, expected_output, test_output);
            //     test_passed = 0;
            // end
        end
        
        // Test 2: Edge cases
        @(posedge clk);
        test_input = 8'h00; // Minimum value
        repeat(5) @(posedge clk);
        
        @(posedge clk);
        test_input = 8'hFF; // Maximum value
        repeat(5) @(posedge clk);
        
        // Test 3: Reset during operation
        @(posedge clk);
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);
        
        // Final report
        $display("========================================");
        if (test_passed) begin
            $display("✓ ALL TESTS PASSED");
            $display("========================================");
            $finish(0); // Exit with success
        end else begin
            $display("✗ TESTS FAILED");
            $display("========================================");
            $finish(1); // Exit with failure
        end
    end
    
    // Timeout watchdog (prevent infinite simulation)
    initial begin
        #200_000_000; // 200ms timeout
        $error("Simulation timeout!");
        $finish(1);
    end
    
    // Monitor important signals
    initial begin
        $monitor("Time=%0t rst_n=%b input=%h output=%h", 
                 $time, rst_n, test_input, test_output);
    end

endmodule
