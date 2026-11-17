
`timescale 1ns / 1ps

module traffic_controller_tb;

    reg clk;
    reg sw_vehicle;
    reg sw_pedestrian;
    reg reset_sw;

    wire [7:0] led;
    wire [3:0] D0_AN;
    wire [7:0] D0_SEG;
    wire [3:0] D1_AN;
    wire [7:0] D1_SEG;

    // Instantiate the traffic controller
    traffic_controller #(
        .CLK_HZ(10)
    ) dut (
        .clk(clk),
        .sw_vehicle(sw_vehicle),
        .sw_pedestrian(sw_pedestrian),
        .reset_sw(reset_sw),
        .led(led),
        .D0_AN(D0_AN),
        .D0_SEG(D0_SEG),
        .D1_AN(D1_AN),
        .D1_SEG(D1_SEG)
    );

    // Generate 10 ns clock period
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Apply test inputs
    initial begin

        // Initial conditions
        reset_sw = 1;
        sw_vehicle = 0;
        sw_pedestrian = 0;

        // Release reset
        #30;
        reset_sw = 0;

        // Vehicle request
        #30;
        sw_vehicle = 1;
        #20;
        sw_vehicle = 0;

        // Wait before pedestrian request
        #1300;

        // Pedestrian request
        sw_pedestrian = 1;
        #20;
        sw_pedestrian = 0;

        // Allow simulation to continue
        #1500;

        $finish;
    end

    // Display simulation values
    initial begin
        $monitor(
            "t=%0t | state=%0d | phase_count=%0d | LED=%h | D0_AN=%b | D0_SEG=%b | D1_AN=%b | D1_SEG=%b",
            $time,
            dut.state,
            dut.phase_count,
            led,
            D0_AN,
            D0_SEG,
            D1_AN,
            D1_SEG
        );
    end

endmodule
