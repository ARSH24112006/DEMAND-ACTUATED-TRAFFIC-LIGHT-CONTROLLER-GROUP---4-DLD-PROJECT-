`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.11.2026 15:26:03
// Design Name: 
// Module Name: traffic_controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////



module traffic_controller #(
    parameter integer CLK_HZ = 100_000_000
)(
    input wire clk,
    input wire sw_vehicle,
    input wire sw_pedestrian,
    input wire reset_sw,

    output reg [7:0] led,

    // Boolean Board seven-segment display 0 (countdown)
    output reg [3:0] D0_AN,
    output reg [7:0] D0_SEG,

    // Boolean Board seven-segment display 1 (FSM state number)
    output reg [3:0] D1_AN,
    output reg [7:0] D1_SEG
);

    // =========================================
    // 1. FSM STATE DEFINITIONS
    // =========================================

    localparam [3:0]
        MAIN_GREEN       = 4'd0,
        MAIN_YELLOW      = 4'd1,
        ALL_RED_SERVICE  = 4'd2,
        SIDE_GREEN       = 4'd3,
        SIDE_YELLOW      = 4'd4,
        PED_WALK         = 4'd5,
        PED_CLEARANCE    = 4'd6,
        ALL_RED_RETURN   = 4'd7;

    reg [3:0] state;

    // =========================================
    // 2. CLOCK DIVIDER: 100 MHz to 1 Hz
    // =========================================

    reg [26:0] clk_div_count;
    reg one_sec_tick;

    always @(posedge clk) begin
        if (reset_sw) begin
            clk_div_count <= 0;
            one_sec_tick <= 0;
        end
        else begin
            if (clk_div_count == CLK_HZ - 1) begin
                clk_div_count <= 0;
                one_sec_tick <= 1;
            end
            else begin
                clk_div_count <= clk_div_count + 1'b1;
                one_sec_tick <= 0;
            end
        end
    end

    // =========================================
    // 3. INPUT SYNCHRONIZATION
    // =========================================

    reg vehicle_meta, vehicle_sync, vehicle_prev;
    reg pedestrian_meta, pedestrian_sync, pedestrian_prev;

    always @(posedge clk) begin
        if (reset_sw) begin
            vehicle_meta <= 0;
            vehicle_sync <= 0;
            vehicle_prev <= 0;

            pedestrian_meta <= 0;
            pedestrian_sync <= 0;
            pedestrian_prev <= 0;
        end
        else begin
            vehicle_meta <= sw_vehicle;
            vehicle_sync <= vehicle_meta;
            vehicle_prev <= vehicle_sync;

            pedestrian_meta <= sw_pedestrian;
            pedestrian_sync <= pedestrian_meta;
            pedestrian_prev <= pedestrian_sync;
        end
    end

    // Detect rising edges of the switches

    wire vehicle_request =
        vehicle_sync & ~vehicle_prev;

    wire pedestrian_request =
        pedestrian_sync & ~pedestrian_prev;

    // =========================================
    // 4. REQUEST MEMORY
    // =========================================

    reg vehicle_pending;
    reg pedestrian_pending;

    reg [5:0] phase_count;

    // =========================================
    // 5. MAIN MOORE FSM
    // =========================================

    always @(posedge clk) begin

        if (reset_sw) begin
            state <= MAIN_GREEN;

            phase_count <= 0;

            vehicle_pending <= 0;
            pedestrian_pending <= 0;
        end

        else begin

            // Remember incoming requests

            if (vehicle_request)
                vehicle_pending <= 1;

            if (pedestrian_request)
                pedestrian_pending <= 1;

            case (state)

                // ---------------------------------
                // DEFAULT STATE
                // ---------------------------------

                MAIN_GREEN: begin

                    phase_count <= 0;

                    if (pedestrian_pending ||
                        pedestrian_request ||
                        vehicle_pending ||
                        vehicle_request) begin

                        state <= MAIN_YELLOW;
                        phase_count <= 0;

                    end
                end

                // ---------------------------------
                // MAIN ROAD YELLOW
                // ---------------------------------

                MAIN_YELLOW: begin

                    if (one_sec_tick) begin

                        if (phase_count == 2) begin
                            state <= ALL_RED_SERVICE;
                            phase_count <= 0;
                        end
                        else begin
                            phase_count <= phase_count + 1'b1;
                        end

                    end
                end

                // ---------------------------------
                // ALL RED BEFORE SERVICE
                // ---------------------------------

                ALL_RED_SERVICE: begin

                    if (one_sec_tick) begin

                        if (phase_count == 0) begin

                            phase_count <= 0;

                            // Pedestrian gets priority

                            if (pedestrian_pending ||
                                pedestrian_request) begin

                                state <= PED_WALK;
                                pedestrian_pending <= 0;

                            end

                            else if (vehicle_pending ||
                                     vehicle_request) begin

                                state <= SIDE_GREEN;
                                vehicle_pending <= 0;

                            end

                            else begin
                                state <= MAIN_GREEN;
                            end

                        end
                        else begin
                            phase_count <= 0;
                        end

                    end
                end

                // ---------------------------------
                // SIDE ROAD GREEN
                // ---------------------------------

                SIDE_GREEN: begin

                    if (one_sec_tick) begin

                        if (phase_count == 7) begin
                            state <= SIDE_YELLOW;
                            phase_count <= 0;
                        end
                        else begin
                            phase_count <= phase_count + 1'b1;
                        end

                    end
                end

                // ---------------------------------
                // SIDE ROAD YELLOW
                // ---------------------------------

                SIDE_YELLOW: begin

                    if (one_sec_tick) begin

                        if (phase_count == 2) begin
                            state <= ALL_RED_RETURN;
                            phase_count <= 0;
                        end
                        else begin
                            phase_count <= phase_count + 1'b1;
                        end

                    end
                end

                // ---------------------------------
                // PEDESTRIAN WALK
                // ---------------------------------

                PED_WALK: begin

                    if (one_sec_tick) begin

                        if (phase_count == 7) begin
                            state <= PED_CLEARANCE;
                            phase_count <= 0;
                        end
                        else begin
                            phase_count <= phase_count + 1'b1;
                        end

                    end
                end

                // ---------------------------------
                // PEDESTRIAN CLEARANCE
                // ---------------------------------

                PED_CLEARANCE: begin

                    if (one_sec_tick) begin

                        if (phase_count == 1) begin
                            state <= ALL_RED_RETURN;
                            phase_count <= 0;
                        end
                        else begin
                            phase_count <= phase_count + 1'b1;
                        end

                    end
                end

                // ---------------------------------
                // ALL RED BEFORE MAIN GREEN
                // ---------------------------------

                ALL_RED_RETURN: begin

                    if (one_sec_tick) begin

                        if (phase_count == 0) begin

                            phase_count <= 0;

                            // Serve any waiting pedestrian first

                            if (pedestrian_pending ||
                                pedestrian_request) begin

                                state <= PED_WALK;
                                pedestrian_pending <= 0;

                            end

                            // Then serve a waiting vehicle

                            else if (vehicle_pending ||
                                     vehicle_request) begin

                                state <= SIDE_GREEN;
                                vehicle_pending <= 0;

                            end

                            else begin
                                state <= MAIN_GREEN;
                            end

                        end
                        else begin
                            phase_count <= 0;
                        end

                    end
                end

                default: begin
                    state <= MAIN_GREEN;
                    phase_count <= 0;
                end

            endcase
        end
    end

    // =========================================
    // 6. MOORE OUTPUT DECODER
    // =========================================

    // LED mapping:
    // LED0 = Main Red
    // LED1 = Main Yellow
    // LED2 = Main Green
    // LED3 = Side Red
    // LED4 = Side Yellow
    // LED5 = Side Green
    // LED6 = Pedestrian Don't Walk
    // LED7 = Pedestrian Walk


always @(*) begin

    led = 8'b00000000;

    case (state)

        MAIN_GREEN:
            led = 8'b01001100;

        MAIN_YELLOW:
            led = 8'b01001010;

        ALL_RED_SERVICE:
            led = 8'b01001001;

        SIDE_GREEN:
            led = 8'b01100001;

        SIDE_YELLOW:
            led = 8'b01010001;

        PED_WALK:
            led = 8'b10001001;

        PED_CLEARANCE:
            led = 8'b01001001;

        ALL_RED_RETURN:
            led = 8'b01001001;

        default:
            led = 8'b01001001;

    endcase

end


    // =========================================
    // 7. TWO FOUR-DIGIT SEVEN-SEGMENT DISPLAYS
    // =========================================
    // Boolean Board displays are common-anode and active-low:
    // ANx = 0 selects a digit; SEG bit = 0 illuminates a segment.
    // D0 displays remaining seconds as 0000..0008.
    // D1 displays the FSM state number as 0000..0007.

    // Hardware default: each digit is selected for 1 ms.
    // This gives a complete four-digit refresh every 4 ms (250 Hz).
    // Use a small value in simulation to make multiplexing visible.
    parameter integer SCAN_DIV = 100_000;

    reg [16:0] scan_count;
    reg [1:0] scan_digit;

    reg [3:0] remaining_seconds;
    reg [3:0] state_number;
    reg [3:0] display_digit_0;
    reg [3:0] display_digit_1;

    // Calculate the time remaining in the current timed phase.
    // MAIN_GREEN is demand-waiting, so it displays 0000.
    always @(*) begin
        case (state)
            MAIN_GREEN:       remaining_seconds = 4'd0;
            MAIN_YELLOW:      remaining_seconds = 4'd3 - phase_count[3:0];
            ALL_RED_SERVICE:  remaining_seconds = 4'd1;
            SIDE_GREEN:       remaining_seconds = 4'd8 - phase_count[3:0];
            SIDE_YELLOW:      remaining_seconds = 4'd3 - phase_count[3:0];
            PED_WALK:         remaining_seconds = 4'd8 - phase_count[3:0];
            PED_CLEARANCE:    remaining_seconds = 4'd2 - phase_count[3:0];
            ALL_RED_RETURN:   remaining_seconds = 4'd1;
            default:          remaining_seconds = 4'd0;
        endcase
    end

    always @(*) begin
        state_number = state;
    end

    // Multiplexing scan counter. The two displays use the same digit index,
    // but have independent segment and anode pins.
    always @(posedge clk) begin
        if (reset_sw) begin
            scan_count <= 0;
            scan_digit <= 0;
        end
        else if (scan_count >= SCAN_DIV - 1) begin
            scan_count <= 0;
            scan_digit <= scan_digit + 1'b1;
        end
        else begin
            scan_count <= scan_count + 1'b1;
        end
    end

    // Select one decimal digit from each four-digit value.
    // Digit index 0 is the leftmost (thousands) position;
    // digit index 3 is the rightmost (ones) position.
    always @(*) begin
        case (scan_digit)
            2'd0: begin
                display_digit_0 = remaining_seconds / 1000;
                display_digit_1 = state_number / 1000;
            end
            2'd1: begin
                display_digit_0 = (remaining_seconds / 100) % 10;
                display_digit_1 = (state_number / 100) % 10;
            end
            2'd2: begin
                display_digit_0 = (remaining_seconds / 10) % 10;
                display_digit_1 = (state_number / 10) % 10;
            end
            default: begin
                display_digit_0 = remaining_seconds % 10;
                display_digit_1 = state_number % 10;
            end
        endcase
    end

    // Active-low seven-segment decoder. Bit order is
    // {DP, G, F, E, D, C, B, A}, matching SEG[7:0] in the board XDC.
    function [7:0] seven_seg_decode;
        input [3:0] digit;
        begin
            case (digit)
                4'd0: seven_seg_decode = 8'b11000000;
                4'd1: seven_seg_decode = 8'b11111001;
                4'd2: seven_seg_decode = 8'b10100100;
                4'd3: seven_seg_decode = 8'b10110000;
                4'd4: seven_seg_decode = 8'b10011001;
                4'd5: seven_seg_decode = 8'b10010010;
                4'd6: seven_seg_decode = 8'b10000010;
                4'd7: seven_seg_decode = 8'b11111000;
                4'd8: seven_seg_decode = 8'b10000000;
                4'd9: seven_seg_decode = 8'b10010000;
                default: seven_seg_decode = 8'b11111111;
            endcase
        end
    endfunction

    // Drive both displays. All anodes are inactive-high except the
    // currently selected digit, which is active-low.
    always @(*) begin
        D0_AN  = 4'b1111;
        D1_AN  = 4'b1111;
        D0_SEG = 8'b11111111;
        D1_SEG = 8'b11111111;

        case (scan_digit)
            2'd0: begin
                D0_AN = 4'b0111;
                D1_AN = 4'b0111;
            end
            2'd1: begin
                D0_AN = 4'b1011;
                D1_AN = 4'b1011;
            end
            2'd2: begin
                D0_AN = 4'b1101;
                D1_AN = 4'b1101;
            end
            2'd3: begin
                D0_AN = 4'b1110;
                D1_AN = 4'b1110;
            end
        endcase

        D0_SEG = seven_seg_decode(display_digit_0);
        D1_SEG = seven_seg_decode(display_digit_1);
    end


endmodule

