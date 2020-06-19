`define FILE_PATH "../../rv32-benchmarks-master/individual-instructions/rv32ui-p-xor.x"
`define FILE_LINES 357
`define VCD "main-mem-rv32ui-p-xor.vcd"

`define MEM_DEPTH_BYTES 'h0100000

`define READ 0
`define WRITE 1

`define HALF_CYCLE 10

/*========================================================================================*/

// main memory component
module mainmem (
	input clock,
	input [31:0] address,
	input [31:0] data_in,
	output [31:0] data_out,
	input read_write); 

    parameter STARTING_ADDR = 'h01000000;

	wire read_write;
	wire [31:0] address;
	wire [31:0] data_in;
	reg [31:0] data_out;

	reg [7:0] mem[0:`MEM_DEPTH_BYTES - 1];

    // read mode
	always @(clock or address or read_write) begin
		if (read_write == `READ)
			data_out <= {
				mem[address - STARTING_ADDR + 3],
				mem[address - STARTING_ADDR + 2],
				mem[address - STARTING_ADDR + 1],
				mem[address - STARTING_ADDR]
			};
		else
			data_out <= 32'hXXXXXXXX;
	end

	// write mode
    always @(posedge clock) begin
        if (read_write == `WRITE) begin
            mem[address - STARTING_ADDR + 3] <= data_in[31:24];
            mem[address - STARTING_ADDR + 2] <= data_in[23:16];
            mem[address - STARTING_ADDR + 1] <= data_in[15:8];
            mem[address - STARTING_ADDR] <= data_in[7:0];
        end
	end

	// testbench
	reg [31:0] tmp_mem[0:(`MEM_DEPTH_BYTES / 4) - 1];
	integer i;
    
    initial begin
        $readmemh(`FILE_PATH, tmp_mem);

		// copy to mem
		i = 0;
		while (tmp_mem[i] !== 32'hX) begin
			mem[(4 * i) + 3] <= tmp_mem[i][31:24];
            mem[(4 * i) + 2] <= tmp_mem[i][23:16];
            mem[(4 * i) + 1] <= tmp_mem[i][15:8];
            mem[(4 * i)] <= tmp_mem[i][7:0];
			i += 1;
		end
    end
endmodule // mainmem

/*========================================================================================*/

// testbench

module dut;
    parameter STARTING_ADDR = 'h01000000;
	
	reg clock = 0;
	reg [31:0] pc;
	reg read_write = 0;
	reg [31:0] data_in;
    wire [31:0] data_out;

    initial begin
        $dumpfile(`VCD);
        $dumpvars(0, memory);

		#(`HALF_CYCLE / 2) pc = STARTING_ADDR;

		#(`FILE_LINES * 4 * `HALF_CYCLE) $finish;
    end
	
	mainmem memory(
		.clock(clock),
		.address(pc),
		.data_in(data_in),
		.data_out(data_out),
		.read_write(read_write));
	
	always @(posedge clock) begin
        $display("time=%t, address=%8h, data_out=%8h", $time, pc, data_out);
        #(`HALF_CYCLE / 2)  pc <= pc + 4;
	end
	
	always begin                 
		#`HALF_CYCLE clock = ~clock;
	end
endmodule
