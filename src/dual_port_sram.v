module dual_port_sram #(
    parameter ADDR_WIDTH = 13,   // 2^13 = 8192 ô nhớ
    parameter DATA_WIDTH = 24,   // 16 bit dữ liệu kênh + 8 bit timestamp
    parameter DEPTH      = 8192  // Độ sâu bộ nhớ (tổng số ô nhớ)
    )(
    input wire i_clk,
    input wire [ADDR_WIDTH-1:0] i_wr_adr,
    input wire [ADDR_WIDTH-1:0] i_rd_adr,
    input wire i_wr_en,
    input wire [DATA_WIDTH-1:0] i_data,
    output reg [DATA_WIDTH-1:0] o_data
    );
    
    reg [DATA_WIDTH-1:0] memory_array [0:DEPTH-1]; 

    always @(posedge i_clk) begin
        if(i_wr_en) begin
            memory_array[i_wr_adr] <= i_data;
        end
        
        o_data <= memory_array[i_rd_adr];
    end 
    
endmodule