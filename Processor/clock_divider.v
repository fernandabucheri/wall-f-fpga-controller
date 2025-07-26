module clock_divider (n_clock, clk, clk_read);

	input n_clock;
	output reg clk, clk_read;
	
	reg [25:0] aux_clk, aux_clk_read;
	
	always @ (posedge n_clock)
	begin

		if (aux_clk == 26'd10000000) begin
//		if (aux_clk == 26'd2) begin
			aux_clk <= 26'd0;
			clk <= 1;
		end
		else begin
			aux_clk <= aux_clk + 1;
			clk <= 0;
		end
	end
	
	always @ (posedge n_clock)
	begin
		if (aux_clk_read == 26'd5000000) begin
//		if (aux_clk_read == 26'd1) begin
			aux_clk_read <= 26'd0;
			clk_read <= 1;
		end
		else begin
			aux_clk_read <= aux_clk_read + 1;
			clk_read <= 0;
		end
	end
endmodule