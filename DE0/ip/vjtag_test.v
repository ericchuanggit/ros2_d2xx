/*
 ============================================================================
 Module Name: vjtag_test

 Description:
 - This module interfaces with a virtual JTAG (VJTAG) for data communication.
 - It provides functionalities to send and receive data using the VJTAG protocol.
 - The module also interfaces with switches, LEDs, and seven-segment displays for 
   user interactions and data visualization.

 Functionalities:
 1. Data Reception: When new data arrives through VJTAG, it's stored in an internal RAM.
 2. Data Transmission: Data can be sent through VJTAG by reading it from internal RAM.
 3. Display:
    - The seven-segment displays show either:
      a) The current send and receive address.
      b) The difference of the send sum from the last sent data and the total sum of received data.
    - The selection between the two display modes is controlled by the SW[0] switch.
 4. LEDs indicate the higher bits of send and receive addresses.

 Operation:
 1. Upon a system or module reset, all internal registers and states are initialized to default values.
 2. The module waits for a receive or send initiation signal.
 3. For data reception:
    - Upon receiving a `recv_init` signal, it prepares to receive new data.
    - When data arrives (`recv` signal is high), it's stored in RAM and the address and sum are updated.
 4. For data transmission:
    - Upon a `send_init` signal, it prepares to send data.
    - When the system is ready to send (`send_ready` signal is high), data from RAM is loaded and transmitted.
    - The send address, sum of sent data, and last sent data are updated accordingly.
 5. Switches, LEDs, and seven-segment displays provide user interaction and data visualization.

 ============================================================================
*/
module vjtag_test (
    input           iRst_n,            // Global reset signal
    input           iClk50M,           // Main system clock
    input [9:0]     SW,                // Input switches
    output [6:0]    HEX0,              // Seven-segment display HEX0 outputs
    output [6:0]    HEX1,              // Seven-segment display HEX1 outputs
    output [6:0]    HEX2,              // Seven-segment display HEX2 outputs
    output [6:0]    HEX3,              // Seven-segment display HEX3 outputs
    output [9:0]    LEDG               // 10 green LEDs output
);

    // Internal signals and registers
    wire [6:0] oSEG_0, oSEG_1, oSEG_2, oSEG_3; // Seven-segment display internal signals
    reg [15:0] seg;                   // 16-bit register for segment control
    wire       recv_init, recv, send_init, send_ready; // Signals for JTAG communication control
    wire [7:0] recv_data;             // 8-bit data received from JTAG
    reg [7:0]  ram[32768];            // Memory storage array
    reg [7:0]  ram_dout;              // Memory data output register
    reg [14:0] recv_adrs, send_adrs;  // Address registers for receiving and sending data
    reg [7:0]  recv_sum, send_sum, send_data_last; // Summation registers and last sent data
    reg        vjtag_send;                 // JTAG send control signal

    // Instantiation of seven-segment control modules
    seg7_ctrl seg7_0 (.iDIG(seg[15:12]), .oSEG(oSEG_0)); // Control module for HEX3
    seg7_ctrl seg7_1 (.iDIG(seg[11: 8]), .oSEG(oSEG_1)); // Control module for HEX2
    seg7_ctrl seg7_2 (.iDIG(seg[ 7: 4]), .oSEG(oSEG_2)); // Control module for HEX1
    seg7_ctrl seg7_3 (.iDIG(seg[ 3: 0]), .oSEG(oSEG_3)); // Control module for HEX0
    
    // Segment output assignments
    assign HEX3 = oSEG_0;           // HEX3 display output
    assign HEX2 = oSEG_1;           // HEX2 display output
    assign HEX1 = oSEG_2;           // HEX1 display output
    assign HEX0 = oSEG_3;           // HEX0 display output

    // JTAG UART instantiation for data communication
    vjtag_uart vjtag (
        /* clock& reset interface  */
        .iClk50M    (iClk50M),        // Main clock
        .iRst_n     (iRst_n),        // Global reset
          /* receive interface */
        .oRecv_init  (recv_init),      // Signal initialization for receiving
        .oRecv_p     (recv),           // Signal for receiving data
        .oRecv_data  (recv_data),      // Data being received
        /* send interface */
        .oSend_init  (send_init),      // Signal initialization for sending
        .oSend_ready (send_ready),     // Signal indicating ready to send
        .iSend       (vjtag_send),     // Signal to start sending
        .iSend_data  (ram_dout)        // Data to send
    );

    // Control logic for segment display based on switch position
    always @(posedge iClk50M or negedge iRst_n) begin
        if (!iRst_n) begin
            seg <= 0;                // Reset segment display control
        end else begin
            if(SW[0]) begin
                seg <= {send_adrs[7:0], recv_adrs[7:0]}; // Assign addresses to segment
            end else begin
                seg <= {(send_sum-send_data_last), recv_sum}; // Assign sum difference to segment
            end
        end
    end

    // Control logic for JTAG data receiving
    always @(posedge iClk50M or negedge iRst_n) begin
        if (!iRst_n) begin
            recv_adrs <= 15'd0;      // Reset receive address
            recv_sum <= 8'd0;        // Reset receive sum
        end else begin
            if (recv_init) begin
                recv_adrs <= 15'd0;  // Initialize receive address
                recv_sum <= 8'd0;    // Initialize receive sum
            end else if (recv) begin
                ram[recv_adrs] <= recv_data; // Store received data to memory
                recv_adrs <= recv_adrs + 1; // Increment receive address
                recv_sum <= recv_sum + recv_data; // Update the sum of received data
            end
        end
    end

    // Control logic for JTAG data sending
    always @(posedge iClk50M or negedge iRst_n) begin
        if (!iRst_n) begin
            send_adrs <= 15'd0;      // Reset send address
            send_sum <= 8'd0;        // Reset send sum
            send_data_last <= 8'd0;  // Reset last sent data
        end else begin
            if (send_init) begin
                send_adrs <= 15'd0;  // Initialize send address
                send_sum <= 8'd0;    // Initialize send sum
                send_data_last <= 8'd0; // Initialize last sent data
            end else if (send_ready) begin
                ram_dout <= ram[send_adrs]; // Load data from memory to send
                send_adrs <= send_adrs + 1; // Increment send address
                vjtag_send <= 1'b1;  // Set JTAG send signal
            end else if (vjtag_send) begin
                send_sum <= send_sum + ram_dout; // Update the sum of sent data
                send_data_last <= ram_dout;     // Store the last sent data
                vjtag_send <= 1'b0;  // Reset JTAG send signal
            end
        end
    end

    // LED display logic
    assign LEDG = {send_adrs[10:1], recv_adrs[10:1]}; // Display send and receive addresses on LEDs

endmodule
