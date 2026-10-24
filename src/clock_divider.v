module clock_divider(
    input wire            i_clk,
    input wire            i_enable,
    input wire  [15:0]    i_scaler,       // (CLK / scalar + 1) = sample rate
    output reg            o_sample_clk_posedge
    );
    
    reg [15:0] r_count;

    always @(posedge i_clk) begin
        if(!i_enable) begin
            r_count                 <= 0;
            o_sample_clk_posedge    <= 0;
        end
        else if (r_count >= i_scaler) begin
            r_count                 <= 0;
            o_sample_clk_posedge    <= 1;
        end
        else begin
            r_count                 <= r_count + 1;
            o_sample_clk_posedge    <= 0;
        end
    end   
    
endmodule