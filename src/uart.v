module uart #(
    parameter CLKS_PER_BIT = 870
    )(
    input wire  i_clk,
    input wire  i_Rx_Serial,
    input wire  i_Tx_DV,
    input wire  [7:0] i_Tx_Byte,
    output wire o_Tx_Serial,
    output wire o_Rx_DV,
    output wire [7:0] o_Rx_Byte,
    output wire o_Tx_Done
    );
    
    uart_rx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
        ) URX (
        .i_clk(i_clk),
        .i_Rx_Serial(i_Rx_Serial),
        .o_Rx_DV(o_Rx_DV),
        .o_Rx_Byte(o_Rx_Byte)
    );
    
    uart_tx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
        ) UTX (
        .i_clk(i_clk),
        .i_Tx_DV(i_Tx_DV),
        .i_Tx_Byte(i_Tx_Byte),
        .o_Tx_Serial(o_Tx_Serial),
        .o_Tx_Done(o_Tx_Done)  
    );
    
endmodule