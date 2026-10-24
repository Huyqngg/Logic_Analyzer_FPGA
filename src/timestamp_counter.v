module timestamp_counter(
    input wire          i_clk,
    input wire          i_enable,
    input wire          i_incr,
    input wire          i_event,
    output wire         o_rollover,
    output reg [7:0]    o_time
    );
    
    assign o_rollover = (& o_time) & i_incr;
    
    always @(posedge i_clk) begin
        if(!i_enable) begin
            o_time <= 1;
        end
        else if(i_event) begin
            o_time <= 1;
        end
        else if(i_incr) begin
            o_time <= o_time + 1;
        end    
    end    
     
endmodule