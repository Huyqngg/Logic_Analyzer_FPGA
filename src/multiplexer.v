module multiplexer #(
    parameter OUT_WIDTH  = 8,   
    parameter NUM_INPUTS = 4    
)(
    input  wire [(NUM_INPUTS * OUT_WIDTH) - 1 : 0] i_data, 
    input  wire [$clog2(NUM_INPUTS) - 1 : 0]       i_sel,  
    output wire [OUT_WIDTH - 1 : 0]                o_data 
);
    assign o_data = i_data >> (i_sel * OUT_WIDTH);

endmodule