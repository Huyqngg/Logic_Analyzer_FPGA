module trigger_engine #(parameter DATA_WIDTH = 16)(
    input wire      i_clk,
    input wire      [DATA_WIDTH-1:0] i_data,
    input wire      [$clog2(DATA_WIDTH)-1:0] i_channel_select,
    input wire      i_trigger_type,
    input wire      i_enable,
    input wire      i_sample_clk_posedge,
    input wire      i_trigger_delay_en,
    input wire      [7:0] i_trigger_delay,
    output wire     o_triggered_state,
    output wire     o_event_pulse
    );
    
    reg [DATA_WIDTH-1:0] r_last;
    reg r_trigger_event;
    wire pe, ne, w_trigger_pulse;
    reg [7:0] trigger_delay_count;
    wire delayed_trigger_event;
    
    assign w_trigger_pulse      = ((pe & i_trigger_type) | (ne & ~i_trigger_type)) & i_enable;
    assign o_triggered_state    = i_trigger_delay_en ? delayed_trigger_event : r_trigger_event;
    assign o_event_pulse        = (r_last != i_data) & i_sample_clk_posedge;
    
    always @(posedge i_clk) begin
        if (i_sample_clk_posedge || !i_enable) begin
            r_last <= i_data;
        end
    end 
    
    // Posedge detection
    assign pe = i_data[i_channel_select] & ~r_last[i_channel_select];
    // Negedge detection
    assign ne = ~i_data[i_channel_select] & r_last[i_channel_select];
    
    always @(posedge i_clk) begin
        if (!i_enable) begin
            r_trigger_event <= 0;
        end
        else if (i_enable & w_trigger_pulse & i_sample_clk_posedge) begin
            r_trigger_event <= 1;
        end
    end 
    
    // trigger delay if enabled
    assign delayed_trigger_event = (trigger_delay_count == i_trigger_delay);
    always @(posedge i_clk) begin
        if (r_trigger_event) begin
            if (i_sample_clk_posedge) begin
                trigger_delay_count <= trigger_delay_count + 1;
            end
        end
        else begin
            trigger_delay_count     <= 0;
        end
    end
    
endmodule
