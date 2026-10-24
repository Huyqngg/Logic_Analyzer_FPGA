module fsm_controller #(
    parameter DATA_WIDTH    = 16, 
    parameter PACKET_WIDTH  = 24, 
    parameter MEM_DEPTH     = 8192
    )(
    input wire  i_clk,
    input wire  i_rstn,
    input wire  i_triggered_state,
    input wire  i_post_read,
    input wire  i_buffer_full,
    input wire  i_finished_read,
    input wire  i_t_rdy,
    input wire  i_rx_DV,
    input wire  i_tx_done,
    input wire  [7:0] i_rx_byte,
    input wire  [PACKET_WIDTH-1:0] i_data,
    output reg  [15:0] o_scaler,
    output reg  [$clog2(MEM_DEPTH)-1:0] o_precap_depth,
    output reg  [$clog2(DATA_WIDTH)-1:0] o_channel_select,
    output reg  o_trigger_type,
    output reg  o_trigger_delay_en,
    output reg  [7:0] o_trigger_delay,
    output reg  o_enable,
    output wire o_r_ack,
    output reg  o_start_read,
    output reg  o_stop,
    output wire o_tx_DV,
    output wire [7:0] o_tx_byte
    );
    
    data_width_divider #(
        .PACKET_WIDTH(PACKET_WIDTH)
        ) DWD ( 
        .i_clk(i_clk),
        .i_triggered_state(i_triggered_state),
        .i_start_read(o_start_read),
        .i_post_read(i_post_read),
        .i_buffer_full(i_buffer_full),
        .i_enable(o_enable),
        .i_data(i_data),
        .i_t_rdy(i_t_rdy),
        .i_tx_done(i_tx_done),
        .o_r_ack(o_r_ack),
        .o_tx_DV(o_tx_DV),
        .o_tx_byte(o_tx_byte)
    );
    
    localparam s_COMMAND    = 2'b00;
    localparam s_VALUE      = 2'b01;
    localparam s_SAVE       = 2'b10;
    
    localparam SET_TRIG_DELAY_EN    = 8'hF7;
    localparam SET_TRIG_DELAY       = 8'hF8;
    localparam SET_START_READ       = 8'hF9;
    localparam SET_SCALER           = 8'hFA;
    localparam SET_CHANNEL          = 8'hFB;
    localparam SET_TRIG_TYPE        = 8'hFC;
    localparam SET_ENABLE           = 8'hFD;
    localparam SET_PRECAP_DEPTH     = 8'hFE;
    localparam SET_STOP             = 8'hFF;
    
    reg r_save; 
    reg r_stored;
    reg r_start_read;

    reg [1:0] r_SM_cmd;
    reg [7:0] commandByte;
    reg [7:0] paramByte;
    
    always @(posedge i_clk) begin
        if (!i_rstn) begin
            r_SM_cmd <= s_COMMAND;
        end
        else begin
        case (r_SM_cmd)
            s_COMMAND:
                begin
                    if(i_rx_DV) begin
                        commandByte <= i_rx_byte;
                        r_SM_cmd    <= s_VALUE;
                    end
                end
            s_VALUE: 
                begin
                    if(i_rx_DV) begin
                        paramByte   <= i_rx_byte;
                        r_SM_cmd    <= s_SAVE;
                    end
                end
            s_SAVE:
                begin
                    r_save <= 1;
                    if (r_stored) begin
                        r_save      <= 0;
                        r_SM_cmd    <= s_COMMAND;
                    end
                end
        endcase
        end
    end
    
    always @(posedge i_clk) begin
        if (!i_rstn) begin
            o_scaler            <= 0;
            o_channel_select    <= 0;
            o_trigger_type      <= 0;
            o_enable            <= 0;
            r_start_read        <= 0;
            o_stop              <= 0;
            o_trigger_delay_en  <= 0;
            o_trigger_delay     <= 0;
            o_precap_depth      <= 0;
        end 
        else begin
            r_stored <= 0;
            if (r_save && !r_stored) begin
                r_stored <= 1;
                case (commandByte)
                    SET_START_READ: begin
                        r_start_read <= paramByte[0];
                    end
                    SET_SCALER:
                        begin
                            o_scaler            <= {o_scaler[7:0], paramByte};
                        end
                    SET_PRECAP_DEPTH:
                        begin
                            o_precap_depth      <= {o_precap_depth[7:0], paramByte};
                        end
                    SET_CHANNEL: 
                        begin
                            o_channel_select    <= paramByte[$clog2(DATA_WIDTH)-1:0];
                        end
                    SET_TRIG_TYPE:
                        begin
                            o_trigger_type      <= paramByte[0];
                        end
                    SET_ENABLE:
                        begin
                            o_enable            <= paramByte[0];
                        end
                    SET_STOP:
                        begin
                            o_stop              <= paramByte[0];
                        end
                    SET_TRIG_DELAY:
                        begin
                            o_trigger_delay     <= paramByte;
                        end
                    SET_TRIG_DELAY_EN:
                        begin
                            o_trigger_delay_en  <= paramByte[0];
                        end
                endcase
            end
        end
    end

    always @(posedge i_clk) begin
        o_start_read <= r_start_read & ~i_finished_read;
    end
endmodule