/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_jonnor_pdm_microphone (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // All output pins must be assigned. If not used, assign to 0.
    assign uio_out = 0;
    assign uio_oe  = 0;

    // List all unused inputs to prevent warnings
    wire _unused = &{ena, clk, rst_n, 1'b0};

    // Input mapping
    wire pdm_dat = ui_in[1];
    wire rst = ~rst_n;

    // Output mapping
    assign uo_out[0] = 0;
    wire uart_tx = uo_out[1];
    wire pdm_clk = uo_out[2];
    assign uo_out[3] = 0;
    assign uo_out[4] = 0;
    assign uo_out[5] = 0;
    assign uo_out[6] = 0;
    assign uo_out[7] = 0;

    // Internals
	wire clk_uart;
	reg [7:0]	char = 8'h6f;
	reg [15:0]	pcm;
	reg		pcm_valid;
	reg		uart_go = 0;
	wire		uart_ready;

	clk_div_pdm	clk_div_pdm_1(clk, pdm_clk);
	clk_div_uart	clk_div_uart_1(clk, clk_uart);
	uart_tx		uart_tx_1(clk_uart, char, uart_go, uart_tx, uart_ready);
	cic3_pdm	cic3_pdm_1(pdm_clk, rst, pdm_dat, pcm, pcm_valid);

	always @(posedge pdm_clk) begin
		char		<= pcm[7:0];
		uart_go		<= pcm_valid;
	end

endmodule

module clk_div_pdm(input clk, output reg clk_pdm);
	reg [3:0] t = 0;
	always @(posedge clk) begin
		t <= t<12-1 ? t+1 : 0;
		clk_pdm <= t<6;
	end
endmodule

module clk_div_uart(input clk, output reg clk_uart);
	reg [3:0] t = 0;
	always @(posedge clk) begin
		t <= t<4-1 ? t+1 : 0;
		clk_uart <= t<2;
	end
endmodule


module uart_tx(input clk, input [7:0] char, input go, output reg tx, output reg ready);
	parameter s_ready =  0;
	parameter s_start =  1;
	parameter s_data0 =  2;
	parameter s_data1 =  3;
	parameter s_data2 =  4;
	parameter s_data3 =  5;
	parameter s_data4 =  6;
	parameter s_data5 =  7;
	parameter s_data6 =  8;
	parameter s_data7 =  9;
	parameter s_stop1 = 10;
	parameter s_stop2 = 11;
	parameter s_stop3 = 12;
	parameter s_stop4 = 13;

	reg [3:0]	state = s_ready;
	reg [7:0]	data = 8'h41;

	always @(posedge clk) begin
		ready <= state == s_ready;
		case (state)
			s_ready: begin
				if (go) begin
					data <= char;
					state <= s_start;
				end
			end
			s_start:	state <= s_data0;
			s_data0:	state <= s_data1;
			s_data1:	state <= s_data2;
			s_data2:	state <= s_data3;
			s_data3:	state <= s_data4;
			s_data4:	state <= s_data5;
			s_data5:	state <= s_data6;
			s_data6:	state <= s_data7;
			s_data7:	state <= s_stop1;
			s_stop1:	state <= s_stop2;
			s_stop2:	state <= s_stop3;
			s_stop3:	state <= s_stop4;
			s_stop4: begin
				if (!go)
					state <= s_ready;
			end
		endcase
		case (state)
			s_ready:	tx <= 1;
			s_start:	tx <= 0;
			s_data0:	tx <= data[0];
			s_data1:	tx <= data[1];
			s_data2:	tx <= data[2];
			s_data3:	tx <= data[3];
			s_data4:	tx <= data[4];
			s_data5:	tx <= data[5];
			s_data6:	tx <= data[6];
			s_data7:	tx <= data[7];
			s_stop1:	tx <= 1;
			s_stop2:	tx <= 1;
			s_stop3:	tx <= 1;
			s_stop4:	tx <= 1;
		endcase
	end
endmodule
