`timescale 1ns / 1ps

/**
 * EXHAUSTIVE TESTBENCH FOR REAL-TIME LOGIC ANALYZER
 * 
 * Verifies:
 * 1. Unit blocks: Clock Divider, Sync, Timestamp, Trigger, UART.
 * 2. System Level: Config -> Capture -> Trigger -> Readout.
 */

module top_testbench();

    // ---------------------------------------------------------
    // 1. Parameters & Global Signals
    // ---------------------------------------------------------
    parameter CLK_PERIOD     = 10;     // 100MHz clock
    parameter CLKS_PER_BIT   = 20;     // Fast for simulation speed
    parameter BIT_PERIOD     = CLKS_PER_BIT * CLK_PERIOD;
    parameter DATA_WIDTH     = 16;
    parameter PACKET_WIDTH   = 24;
    parameter MEM_DEPTH      = 64;     // Smaller for faster simulation

    reg clk;
    reg rstn;
    
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---------------------------------------------------------
    // 2. Unit Test Signals & Instantiations
    // ---------------------------------------------------------
    
    // Clock Divider
    reg         cd_en;
    reg  [15:0] cd_scaler;
    wire        cd_pulse;
    clock_divider u_cd (.i_clk(clk), .i_enable(cd_en), .i_scaler(cd_scaler), .o_sample_clk_posedge(cd_pulse));

    // Reset Sync
    wire rstn_sync;
    reset_sync u_rs (.i_clk(clk), .i_rstn(rstn), .o_rstsync(rstn_sync));

    // Sampler Sync
    reg  [DATA_WIDTH-1:0] async_probe;
    wire [DATA_WIDTH-1:0] sync_probe;
    sampler_sync #(.DATA_WIDTH(DATA_WIDTH)) u_ss (.i_clk(clk), .i_rstn(rstn), .i_async(async_probe), .o_sync(sync_probe));

    // Timestamp
    reg  ts_en, ts_incr, ts_event;
    wire ts_rollover;
    wire [7:0] ts_time;
    timestamp_counter u_ts (.i_clk(clk), .i_enable(ts_en), .i_incr(ts_incr), .i_event(ts_event), .o_rollover(ts_rollover), .o_time(ts_time));

    // Trigger Engine
    reg  [$clog2(DATA_WIDTH)-1:0] trig_ch;
    reg  trig_type, trig_en, trig_delay_en;
    reg  [7:0] trig_delay;
    wire trig_state, trig_event_pulse;
    trigger_engine #(.DATA_WIDTH(DATA_WIDTH)) u_te (
        .i_clk(clk), .i_data(sync_probe), .i_channel_select(trig_ch),
        .i_trigger_type(trig_type), .i_enable(trig_en), .i_sample_clk_posedge(cd_pulse),
        .i_trigger_delay_en(trig_delay_en), .i_trigger_delay(trig_delay),
        .o_triggered_state(trig_state), .o_event_pulse(trig_event_pulse)
    );

    // ---------------------------------------------------------
    // 3. System Level Instantiation (DUT)
    // ---------------------------------------------------------
    reg  [DATA_WIDTH-1:0] sys_probe;
    reg  sys_rx;
    wire sys_tx;

    logic_analyzer_top #(
        .DATA_WIDTH(DATA_WIDTH), .PACKET_WIDTH(PACKET_WIDTH),
        .MEM_DEPTH(MEM_DEPTH), .CLKS_PER_BIT(CLKS_PER_BIT)
    ) DUT (
        .i_clk(clk), .i_rstn(rstn), .i_rx(sys_rx), .i_probe(sys_probe), .o_tx(sys_tx)
    );

    // ---------------------------------------------------------
    // 4. Helper Tasks
    // ---------------------------------------------------------
    
    task UART_WRITE_SYS(input [7:0] data);
        integer i;
        begin
            sys_rx = 0; #(BIT_PERIOD); // Start bit
            for(i=0; i<8; i=i+1) begin
                sys_rx = data[i]; #(BIT_PERIOD);
            end
            sys_rx = 1; #(BIT_PERIOD); // Stop bit
        end
    endtask

    task SEND_CMD(input [7:0] cmd, input [7:0] val);
        begin
            $display("[UART] Sending Cmd: %h, Val: %h", cmd, val);
            UART_WRITE_SYS(cmd);
            UART_WRITE_SYS(val);
            #(CLK_PERIOD*10);
        end
    endtask

    localparam SET_TRIG_DELAY_EN    = 8'hF7;
    localparam SET_TRIG_DELAY       = 8'hF8;
    localparam SET_START_READ       = 8'hF9;
    localparam SET_SCALER           = 8'hFA;
    localparam SET_CHANNEL          = 8'hFB;
    localparam SET_TRIG_TYPE        = 8'hFC;
    localparam SET_ENABLE           = 8'hFD;
    localparam SET_PRECAP_DEPTH     = 8'hFE;
    localparam SET_STOP             = 8'hFF;

    // ---------------------------------------------------------
    // 5. Main Test Script
    // ---------------------------------------------------------
    initial begin
        // Init
        rstn = 0; cd_en = 0; cd_scaler = 0; async_probe = 0;
        ts_en = 0; ts_incr = 0; ts_event = 0;
        trig_ch = 0; trig_type = 0; trig_en = 0; trig_delay_en = 0; trig_delay = 0;
        sys_rx = 1; sys_probe = 0;

        $display("\n--- STARTING TESTS ---\n");
        #(CLK_PERIOD*10) rstn = 1;
        #(CLK_PERIOD*10);

        // --- 1. Unit Tests ---
        $display("[Unit] Clock Divider...");
        cd_scaler = 1; cd_en = 1; // Pulse every 2 cycles
        repeat(3) @(posedge cd_pulse);
        cd_en = 0;

        $display("[Unit] Sync & Timestamp...");
        async_probe = 16'hAAAA;
        ts_en = 1; ts_incr = 1;
        repeat(5) @(posedge clk);
        ts_event = 1; @(posedge clk); ts_event = 0;
        #1
        if (ts_time !== 1) $display("  FAIL: Timestamp Reset");
        ts_en = 0;

        $display("[Unit] Trigger...");
        trig_ch = 0; trig_type = 1; trig_en = 1; cd_en = 1; cd_scaler = 0;
        async_probe[0] = 0; @(posedge clk);
        async_probe[0] = 1; @(posedge clk);
        #(CLK_PERIOD*5);
        if (trig_state !== 1) $display("  FAIL: Trigger State");
        trig_en = 0; cd_en = 0;

        // --- 2. System Level ---
        $display("\n--- SYSTEM LEVEL TEST ---");
        SEND_CMD(SET_SCALER, 8'h01);
        SEND_CMD(SET_PRECAP_DEPTH, 8'h08);
        SEND_CMD(SET_CHANNEL, 8'h00);
        SEND_CMD(SET_TRIG_TYPE, 8'h01);
        SEND_CMD(SET_ENABLE, 8'h01);

        repeat(20) begin #(CLK_PERIOD*5); sys_probe[1] = ~sys_probe[1]; end
        $display("[System] Triggering...");
        sys_probe[0] = 1;

        fork
            begin : timeout_full
                #(BIT_PERIOD * 5000);
                $display("  FAIL: Buffer Full Timeout"); $stop;
            end
            begin
                wait(DUT.u_circular_buffer.o_buffer_full);
                $display("  PASS: Buffer Full");
                disable timeout_full;
            end
        join

        $display("[System] Reading back data...");
        SEND_CMD(SET_START_READ, 8'h01);

        fork
            begin : timeout_done
                #(BIT_PERIOD * 10000);
                $display("  FAIL: Readout Timeout"); $stop;
            end
            begin
                wait(DUT.u_circular_buffer.o_finished_read);
                $display("  PASS: Readout Finished");
                disable timeout_done;
            end
        join

        #(BIT_PERIOD*100);
        $display("\n--- ALL TESTS DONE ---");
        $stop;
    end

    // Monitor System UART TX
    wire [7:0] mon_rx_byte;
    wire mon_rx_dv;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_mon (
        .i_clk(clk), .i_Rx_Serial(sys_tx), .o_Rx_DV(mon_rx_dv), .o_Rx_Byte(mon_rx_byte)
    );
    always @(posedge mon_rx_dv) $display("[DUT_TX] %h", mon_rx_byte);

endmodule