module logic_analyzer_top #(
    parameter DATA_WIDTH   = 16,
    parameter PACKET_WIDTH = 24, 
    parameter MEM_DEPTH    = 8192,
    parameter CLKS_PER_BIT = 870,
    parameter INVERT_PROBE = 1'b0  // 1 = đảo 16 kênh vào (khi tín hiệu đi qua mạch bảo vệ 74LVC14A - inverter)
)(
    input  wire                    i_clk,       // Clock hệ thống
    input  wire                    i_rstn,      // Nút reset bất đồng bộ ( active low )
    input  wire                    i_rx,        // Chân nhận UART
    input  wire [DATA_WIDTH-1:0]   i_probe,     // 16 kênh đo logic đầu vào
    output wire                    o_tx         // Chân phát UART
);

    // ---------------------------------------------------------
    // Khai báo kết nối nội bộ (Internal Wires)
    // ---------------------------------------------------------
    wire w_rstn_sync;
    wire [DATA_WIDTH-1:0] w_synced_data;
    
    // Tín hiệu Clock Divider & Timestamp
    wire w_sample_clk_posedge;
    wire w_time_rollover;
    wire [7:0] w_time;
    
    // Tín hiệu Trigger
    wire w_triggered_state;
    wire w_event_pulse;
    wire w_write_event;
    
    // Tín hiệu UART
    wire w_rx_dv;
    wire [7:0] w_rx_byte;
    wire w_tx_dv;
    wire [7:0] w_tx_byte;
    wire w_tx_done;
    
    // Tín hiệu từ FSM điều khiển
    wire [15:0] w_scaler;
    wire [$clog2(MEM_DEPTH)-1:0] w_precap_depth;
    wire [$clog2(DATA_WIDTH)-1:0] w_channel_select;
    wire w_trigger_type;
    wire w_trigger_delay_en;
    wire [7:0] w_trigger_delay;
    wire w_enable;
    wire w_start_read;
    wire w_stop;
    wire w_r_ack;
    
    // Tín hiệu Circular Buffer
    wire [PACKET_WIDTH-1:0] w_packet_data;
    wire [PACKET_WIDTH-1:0] w_cb_data;
    wire w_post_read;
    wire w_buffer_full;
    wire w_finished_read;
    wire w_t_rdy;

    // Tín hiệu từ PLL
    wire w_clk_100mhz; // Dây mang xung 100MHz
    wire w_pll_locked;

    // Ghép dữ liệu từ các kênh đo và timestamp thành gói 24-bit
    assign w_packet_data = {w_synced_data, w_time};
    
    // Ghi vào bộ đệm khi có sự thay đổi (event) hoặc khi bộ đếm thời gian tràn (rollover)
    assign w_write_event = w_event_pulse | w_time_rollover;

    // ---------------------------------------------------------
    // Khởi tạo các module con (Instances)
    // ---------------------------------------------------------

    // Khối PLL
    pll_sys u_pll (
        .areset (~i_rstn),      // active high reset
        .inclk0 (i_clk),        // 50MHz
        .c0     (w_clk_100mhz), // 100MHz
        .locked (w_pll_locked)
    );

    // Đồng bộ hóa nút reset để tránh lỗi metastability
    reset_sync u_reset_sync (
        .i_clk(w_clk_100mhz),
        .i_rstn(i_rstn & w_pll_locked),
        .o_rstsync(w_rstn_sync) // active low reset
    );

    // Đảo lại 16 kênh nếu đi qua mạch bảo vệ dùng 74LVC14A (Schmitt inverter): NOT + NOT = tín hiệu thật
    wire [DATA_WIDTH-1:0] w_probe_in = INVERT_PROBE ? ~i_probe : i_probe;

    // Lấy mẫu và đồng bộ hóa tín hiệu từ 16 kênh đo
    sampler_sync #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_sampler_sync (
        .i_clk(w_clk_100mhz),
        .i_rstn(w_rstn_sync),
        .i_async(w_probe_in),
        .o_sync(w_synced_data)
    );

    // Bộ chia tần số để quyết định tốc độ lấy mẫu (Sample Rate)
    clock_divider u_clock_divider (
        .i_clk(w_clk_100mhz),
        .i_enable(w_enable),
        .i_scaler(w_scaler),
        .o_sample_clk_posedge(w_sample_clk_posedge)
    );

    // Bộ đếm thời gian giữa các lần có sự kiện thay đổi tín hiệu
    timestamp_counter u_timestamp_counter (
        .i_clk(w_clk_100mhz),
        .i_enable(w_enable),
        .i_incr(w_sample_clk_posedge),
        .i_event(w_event_pulse),
        .o_rollover(w_time_rollover),
        .o_time(w_time)
    );

    // Khối phát hiện điều kiện kích hoạt (Trigger)
    trigger_engine #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_trigger_engine (
        .i_clk(w_clk_100mhz),
        .i_data(w_synced_data),
        .i_channel_select(w_channel_select),
        .i_trigger_type(w_trigger_type),
        .i_enable(w_enable),
        .i_sample_clk_posedge(w_sample_clk_posedge),
        .i_trigger_delay_en(w_trigger_delay_en),
        .i_trigger_delay(w_trigger_delay),
        .o_triggered_state(w_triggered_state),
        .o_event_pulse(w_event_pulse)
    );

    // Bộ đệm vòng lưu trữ dữ liệu đo đạc (Circular Buffer SRAM) 
    circular_buffer_controller #(
        .PACKET_WIDTH(PACKET_WIDTH),
        .MEM_DEPTH(MEM_DEPTH)
    ) u_circular_buffer (
        .i_clk(w_clk_100mhz),
        .i_rstn(w_rstn_sync),
        .i_enable(w_enable),
        .i_stop(w_stop),
        .i_precap_depth(w_precap_depth),
        .i_triggered_state(w_triggered_state),
        .i_event(w_write_event),
        .i_r_ack(w_r_ack),
        .i_start_read(w_start_read),
        .i_data(w_packet_data),
        .o_post_read(w_post_read),
        .o_buffer_full(w_buffer_full),
        .o_finished_read(w_finished_read),
        .o_data(w_cb_data),
        .o_t_rdy(w_t_rdy)
    );

    // FSM controller trung tâm xử lý lệnh từ máy tính và gửi dữ liệu đi
    // Module này đã bao gồm khối data_width_divider bên trong
    fsm_controller #(
        .DATA_WIDTH(DATA_WIDTH),
        .PACKET_WIDTH(PACKET_WIDTH),
        .MEM_DEPTH(MEM_DEPTH)
    ) u_fsm_controller (
        .i_clk(w_clk_100mhz),
        .i_rstn(w_rstn_sync),
        .i_triggered_state(w_triggered_state),
        .i_post_read(w_post_read),
        .i_buffer_full(w_buffer_full),
        .i_finished_read(w_finished_read),
        .i_t_rdy(w_t_rdy),
        .i_rx_DV(w_rx_dv),
        .i_tx_done(w_tx_done),
        .i_rx_byte(w_rx_byte),
        .i_data(w_cb_data),
        
        .o_scaler(w_scaler),
        .o_precap_depth(w_precap_depth),
        .o_channel_select(w_channel_select),
        .o_trigger_type(w_trigger_type),
        .o_trigger_delay_en(w_trigger_delay_en),
        .o_trigger_delay(w_trigger_delay),
        .o_enable(w_enable),
        .o_r_ack(w_r_ack),
        .o_start_read(w_start_read),
        .o_stop(w_stop),
        .o_tx_DV(w_tx_dv),
        .o_tx_byte(w_tx_byte)
    );

    // Giao tiếp UART với máy tính (Baudrate thiết lập bởi CLKS_PER_BIT)
    uart #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart (
        .i_clk(w_clk_100mhz),
        .i_Rx_Serial(i_rx),
        .i_Tx_DV(w_tx_dv),
        .i_Tx_Byte(w_tx_byte),
        .o_Tx_Serial(o_tx),
        .o_Rx_DV(w_rx_dv),
        .o_Rx_Byte(w_rx_byte),
        .o_Tx_Done(w_tx_done)
    );

endmodule