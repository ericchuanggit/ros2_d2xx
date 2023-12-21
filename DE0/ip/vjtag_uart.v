/*
    Data transmission and reception with VirtualJTAG
*/



module vjtag_uart (
    // Define inputs and outputs for the module
    input               iRst_n,         // Global reset
    input               iClk50M,        // Main clock
    output              oRecv_init,     // Signal initialization for receiving
    output              oRecv_p,        // trigger for receiving data
    output     [7:0]    oRecv_data,     // Data being received
    output              oSend_init,     // Signal initialization for sending
    output              oSend_ready,    // Signal indicating ready to send
    input               iSend,          // Signal to start sending
    input       [7:0]   iSend_data  // Data to send
);

`include "vjtag_uart.vh"

    // Internal wires for VirtualJTAG connection
    wire [7:0]      ir_in;
    wire            tck, tdi, tdo;
    wire            virtual_state_sdr, virtual_state_uir;

    // Instance of VirtualJTAG module
    VirtualJTAG vjtag (
         .ir_in   (ir_in),                       // Input register to load JTAG instructions.
         .tdo     (tdo),                         // TDO (Test Data Output) is the output data line from the JTAG module.
         .tck     (tck),                         // TCK (Test Clock) is the clock signal used by the JTAG interface.
         .tdi     (tdi),                         // TDI (Test Data Input) is the input data line to the JTAG module.
         .virtual_state_sdr(virtual_state_sdr),  // A signal that indicates if the JTAG state machine is in the Shift-DR state.
         .virtual_state_uir(virtual_state_uir)   // A signal that indicates if the JTAG state machine is in the Update-IR state.
    );



    // Internal registers for control and data
    reg t_init, t_recv, t_send;
    reg [7:0] ir, dr, ds;
    reg [2:0] count;

    // Main state machine logic
    always @(negedge iRst_n or posedge tck) begin
        if(!iRst_n) begin  // Reset all states
            t_init <= 0;
            t_recv <= 0;
            t_send <= 0;
        end
        else if(virtual_state_uir) begin  // Update the instruction register
            ir <= ir_in;
            t_init <= 1;
        end	
        else if(t_init) begin  // If instruction is recognized, prepare for data phase
            count <= 0;
            if(ir == COMMAND_SEND) t_send <= 1;
            t_init <= 0;
        end
        else if(virtual_state_sdr) begin  // Shift-DR state
            dr[count] <= tdi;  // Load data from TDI
            count <= count + 1;
            if(count==7) begin  // Once all data is received
                if(ir==COMMAND_RECV) t_recv <= 1;
                if(ir==COMMAND_SEND) t_send <= 1;
            end
            else begin  // Not all data received, continue to wait
                if(ir==COMMAND_RECV) t_recv <= 0;
                if(ir==COMMAND_SEND) t_send <= 0;
            end
        end
    end

    // Synchronization for initialization signal with main clock domain
    reg [2:0] m_init;
    always @(negedge iRst_n or posedge iClk50M) begin
        if(!iRst_n)
            m_init <= 3'b000;
        else
            m_init <= {m_init[1:0], t_init};
    end

    // Detect rising edge of initialization signals
    assign oRecv_init = (ir==COMMAND_RECV && m_init[2:1]==2'b01) ? 1 : 0;
    assign oSend_init = (ir==COMMAND_SEND && m_init[2:1]==2'b01) ? 1 : 0;

    // Synchronization for receiving signal with main clock domain
    reg [2:0] m_recv;
    always @(negedge iRst_n or posedge iClk50M) begin
        if(!iRst_n)
            m_recv <= 3'b000;
        else
            m_recv <= {m_recv[1:0], t_recv};
    end

    // Assigning received data and signal
    assign oRecv_data = dr;
    assign oRecv_p = m_recv[2:1]==2'b01 ? 1 : 0;

    // Synchronization for sending signal with main clock domain
    reg [2:0] m_send;
    always @(negedge iRst_n or posedge iClk50M) begin
        if(!iRst_n)
            m_send <= 3'b000;
        else
            m_send <= {m_send[1:0], t_send};
    end

    // Indicate when ready to send data
    assign oSend_ready = m_send[2:1]==2'b01 ? 1 : 0;

    // Load data to send when send signal is asserted
    always @(negedge iRst_n or posedge iClk50M) begin
        if(!iRst_n)
            ds <= 8'h00;
        else if(iSend)
            ds <= iSend_data;
    end

    // Assign data out to TDO pin for transmission
    assign tdo = ds[count];

endmodule
