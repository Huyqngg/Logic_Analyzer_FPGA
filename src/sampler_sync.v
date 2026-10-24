// Minh su dung 16 kenh (16 inputs or 16bits) thi module nay tao 2*16 = 32 flip flop de khac phuc metastability
// 16 ff cuoi cung de keo day va chot du lieu cho toan he thong ra cung 1 luc (sampler)
module sampler_sync #(
    parameter DATA_WIDTH   =   16
)(
    input  wire                     i_clk,
    input  wire                     i_rstn,
    input  wire  [DATA_WIDTH-1:0]   i_async,
    output reg   [DATA_WIDTH-1:0]   o_sync
);
    //metastability
    reg    [DATA_WIDTH-1:0]    ff_01;
    reg    [DATA_WIDTH-1:0]    ff_02;

    always @(posedge i_clk or negedge i_rstn) begin
        if(!i_rstn) begin 
            ff_01       <=      {DATA_WIDTH{1'b0}};
            ff_02       <=      {DATA_WIDTH{1'b0}};
            o_sync      <=      {DATA_WIDTH{1'b0}};
        end
        else begin 
            //Tranh metastable => 2 flip-flop
            ff_01       <=      i_async;
            ff_02       <=      ff_01;
            o_sync      <=      ff_02;
        end
    end
endmodule