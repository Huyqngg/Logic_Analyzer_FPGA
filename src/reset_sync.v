module reset_sync(
	input wire      i_clk,
	input wire      i_rstn,
	output reg  	o_rstsync
);
	reg ff_01;
	
	always @(posedge i_clk or negedge i_rstn) begin
		if(!i_rstn) begin
			ff_01       <= 0;
			o_rstsync   <= 0;
		end
		else begin
			ff_01       <= 1;
			o_rstsync   <= ff_01;
		end
	end
	
endmodule