`include "util.v"

module uart_tx(
	input mclk,
	input reset,
	input baud_x1,
	output serial,
	output reg ready,
	input [7:0] data,
	input data_strobe
);

   reg [7+1+1:0]   shiftreg;

   reg         serial_r;
   assign      serial = !serial_r;

   always @(posedge mclk)
     if (reset) begin
        shiftreg <= 0;
        serial_r <= 0;
     end
     else if (data_strobe) begin
        shiftreg <= {
		1'b1, // stop bit
		data,
		1'b0  // start bit (inverted)
	};
	ready <= 0;
     end
     else if (baud_x1) begin
        if (shiftreg == 0)
	begin
          /* Idle state is idle high, serial_r is inverted */
          serial_r <= 0;
	  ready <= 1;
	end else
          serial_r <= !shiftreg[0];
  	// shift the output register down
        shiftreg <= {1'b0, shiftreg[7+1+1:1]};
    end else
    	ready <= (shiftreg == 0);

endmodule


/*
 * Byte receiver, RS-232 8-N-1
 *
 * Receives on 'serial'. When a properly framed byte is
 * received, 'data_strobe' pulses while the byte is on 'data'.
 *
 * Error bytes are ignored.
 */

module uart_rx(mclk, reset, baud_x4,
                      serial, data, data_strobe);

   input        mclk, reset, baud_x4, serial;
   output [7:0] data;
   output       data_strobe;

   /*
    * Synchronize the serial input to this clock domain
    */
   wire         serial_sync;
   d_flipflop_pair input_dff(mclk, reset, serial, serial_sync);

   /*
    * State machine: Four clocks per bit, 10 total bits.
    */
   reg [8:0]    shiftreg;
   reg [5:0]    state;
   reg          data_strobe;
   wire [3:0]   bit_count = state[5:2];
   wire [1:0]   bit_phase = state[1:0];

   wire         sampling_phase = (bit_phase == 1);
   wire         start_bit = (bit_count == 0 && sampling_phase);
   wire         stop_bit = (bit_count == 9 && sampling_phase);

   wire         waiting_for_start = (state == 0 && serial_sync == 1);

   wire         error = ( (start_bit && serial_sync == 1) ||
                          (stop_bit && serial_sync == 0) );

   assign       data = shiftreg[7:0];

   always @(posedge mclk or posedge reset)
     if (reset) begin
        state <= 0;
        data_strobe <= 0;
     end
     else if (baud_x4) begin

        if (waiting_for_start || error || stop_bit)
          state <= 0;
        else
          state <= state + 1;

        if (bit_phase == 1)
          shiftreg <= { serial_sync, shiftreg[8:1] };

        data_strobe <= stop_bit && !error;

     end
     else begin
        data_strobe <= 0;
     end

endmodule


/*
 * Output UART with a block RAM FIFO queue.
 *
 * Add bytes to the queue and they will be printed when the line is idle.
 */
module uart_tx_fifo(
	input clk,
	input reset,
	input baud_x1,
	input [7:0] data,
	input data_strobe,
	output serial
);
	parameter NUM = 32;

	wire uart_txd_ready; // high the UART is ready to take a new byte
	reg uart_txd_strobe; // pulse when we have a new byte to transmit
	reg [7:0] uart_txd;

	uart_tx txd(
		.mclk(clk),
		.reset(reset),
		.baud_x1(baud_x1),
		.serial(serial),
		.ready(uart_txd_ready),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe)
	);

	wire fifo_available;
	wire fifo_read_strobe;

	fifo #(.NUM(NUM), .WIDTH(8)) buffer(
		.clk(clk),
		.reset(reset),
		.write_data(data),
		.write_strobe(data_strobe),
		.data_available(fifo_available),
		.read_data(uart_txd),
		.read_strobe(fifo_read_strobe)
	);

	// drain the fifo into the serial port
	always @(posedge clk)
	begin
		uart_txd_strobe <= 0;
		fifo_read_strobe <= 0;

		if (fifo_available
		&&  uart_txd_ready
		&& !data_strobe // avoid dual port RAM if possible
		&& !uart_txd_strobe // don't TX twice on one byte
		) begin
			fifo_read_strobe <= 1;
			uart_txd_strobe <= 1;
		end
	end
endmodule
