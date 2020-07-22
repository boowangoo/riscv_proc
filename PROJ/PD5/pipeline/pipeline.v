`define FILE_PATH "../../rv32-benchmarks-master/simple-programs/SimpleAdd.x"
// `define FILE_PATH "../../rv32-benchmarks-master/individual-instructions/rv32ui-p-xori.x"
`define FILE_LINES 39
`define VCD "pipeline-SimpleAdd.vcd"
// `define VCD "pipeline-rv32ui-p-xori.vcd"

`define HALF_CYCLE 10

/*========================================================================================*/

// interstage flipflop
module flipflop(
	input wire [31:0]	data,
	input wire			clock,
	input wire			reset,
	output reg [31:0]	data_out);

	always @ (posedge clock or reset) begin
	// always @ (negedge clock or reset) begin
		if (reset) begin
			data_out <= 32'h00000000;
		end
		else begin
			data_out <= data;
		end
	end
endmodule

/*========================================================================================*/

`define MEM_DEPTH_BYTES 'h0100000
`define STARTING_ADDR 'h01000000

`define READ 0
`define WRITE 1

`define WR_WORD		2'b00
`define WR_HALFW	2'b01
`define WR_BYTE		2'b10

// main memory component
module mainmem (
	input clock,
	input wire [31:0] address,
	input wire [31:0] data_in,
	output reg [31:0] data_out,
	input wire read_write,
	input load_instrs,
	input [1:0] wr_width); 

	reg [7:0] mem[0:`MEM_DEPTH_BYTES - 1];

    // read mode
	always @(address or read_write) begin
		if (read_write == `READ)
			data_out <= {
				mem[address - `STARTING_ADDR + 3],
				mem[address - `STARTING_ADDR + 2],
				mem[address - `STARTING_ADDR + 1],
				mem[address - `STARTING_ADDR]
			};
		else
			data_out <= 32'hXXXXXXXX;
	end

	always @(data_out) begin
		$display("read 0x%8h from memory slot 0x%8h", data_out, address);
	end
	// always @(data_in) begin
	// 	$display("new data_in: 0x%8h", data_in);
	// end
	always @(address) begin
		$display("new address: 0x%8h", address);
	end

	// always @(posedge clock) begin
	always @(negedge clock) begin
        if (read_write == `WRITE) begin
			$display("writing 0x%8h into memory slot 0x%8h", data_in, address);
			if (wr_width == `WR_WORD) begin
				mem[address - `STARTING_ADDR + 3] <= data_in[31:24];
				mem[address - `STARTING_ADDR + 2] <= data_in[23:16];
			end

			if (wr_width == `WR_WORD || wr_width == `WR_HALFW) begin
				mem[address - `STARTING_ADDR + 1] <= data_in[15:8];
			end

            mem[address - `STARTING_ADDR] <= data_in[7:0];
        end
	end

	// testbench
	reg [31:0] tmp_mem[0:(`MEM_DEPTH_BYTES / 4) - 1];
	integer i;
    
    initial begin
		if (load_instrs) begin
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
    end
endmodule // mainmem

/*========================================================================================*/

// register component
module register(
	input clock,
	input [31:0] instr,
	input [4:0] addr_rs1,
	input [4:0] addr_rs2,
	input [4:0] addr_rd,
	input [31:0] data_rd,
	output reg [31:0] data_rs1,
	output reg [31:0] data_rs2,
	input write_enable,
	input ecall);

	reg [31:0] regfile [0:31];

	// assign data_rs1 = regfile[addr_rs1];
	// assign data_rs2 = regfile[addr_rs2];
	// read rs1
	always @(instr or addr_rs1 or regfile[addr_rs1]) begin
		data_rs1 = regfile[addr_rs1];
		$display("rs1: x%0d is 0x%8h", addr_rs1, data_rs1);
	end

	// read rs1
	always @(instr or addr_rs2 or regfile[addr_rs2]) begin
		data_rs2 = regfile[addr_rs2];
	end

	// always @(data_rs1) begin
	// 	$display("rs1: 0x%8h", data_rs1);
	// end

	// always @(data_rd) begin
	// 	$display("data_rd: 0x%8h", data_rd);
	// end

	// always @(write_enable) begin
	// 	if (!write_enable)
	// 		$display("register write off");
	// end

	always @(negedge clock) begin
		#1;
		$display("x00:0x%8h  x01:0x%8h  x02:0x%8h  x03:0x%8h  x04:0x%8h  x05:0x%8h  x06:0x%8h  x07:0x%8h",
				regfile[0], regfile[1], regfile[2], regfile[3], regfile[4], regfile[5], regfile[6], regfile[7]);
		$display("x08:0x%8h  x09:0x%8h  x10:0x%8h  x11:0x%8h  x12:0x%8h  x13:0x%8h  x14:0x%8h  x15:0x%8h",
				regfile[8], regfile[9], regfile[10], regfile[11], regfile[12], regfile[13], regfile[14], regfile[15]);
		$display("x16:0x%8h  x17:0x%8h  x18:0x%8h  x19:0x%8h  x20:0x%8h  x21:0x%8h  x22:0x%8h  x23:0x%8h",
				regfile[16], regfile[17], regfile[18], regfile[19], regfile[20], regfile[21], regfile[22], regfile[23]);
		$display("x24:0x%8h  x25:0x%8h  x26:0x%8h  x27:0x%8h  x28:0x%8h  x29:0x%8h  x30:0x%8h  x31:0x%8h",
				regfile[24], regfile[25], regfile[26], regfile[27], regfile[28], regfile[29], regfile[30], regfile[31]);
		$display("");
	end

	// always @(posedge clock) begin
	always @(negedge clock) begin
		if (write_enable && addr_rd != 0) begin
			regfile[addr_rd] = data_rd;
			// $display("write 0x%8h into x%2d", data_rd, addr_rd);
		end
	end

	integer i;
	integer sp_initial_occurences = 0;

	always @(regfile[2]) begin
		if (regfile[2] == `MEM_DEPTH_BYTES + `STARTING_ADDR - 4)
			sp_initial_occurences += 1;
	end

	// end conditions
	always @(sp_initial_occurences) begin
		if (sp_initial_occurences == 1) begin
			$display("x00:0x%8h  x01:0x%8h  x02:0x%8h  x03:0x%8h  x04:0x%8h  x05:0x%8h  x06:0x%8h  x07:0x%8h",
				regfile[0], regfile[1], regfile[2], regfile[3], regfile[4], regfile[5], regfile[6], regfile[7]);
			$display("x08:0x%8h  x09:0x%8h  x10:0x%8h  x11:0x%8h  x12:0x%8h  x13:0x%8h  x14:0x%8h  x15:0x%8h",
					regfile[8], regfile[9], regfile[10], regfile[11], regfile[12], regfile[13], regfile[14], regfile[15]);
			$display("x16:0x%8h  x17:0x%8h  x18:0x%8h  x19:0x%8h  x20:0x%8h  x21:0x%8h  x22:0x%8h  x23:0x%8h",
					regfile[16], regfile[17], regfile[18], regfile[19], regfile[20], regfile[21], regfile[22], regfile[23]);
			$display("x24:0x%8h  x25:0x%8h  x26:0x%8h  x27:0x%8h  x28:0x%8h  x29:0x%8h  x30:0x%8h  x31:0x%8h",
					regfile[24], regfile[25], regfile[26], regfile[27], regfile[28], regfile[29], regfile[30], regfile[31]);
			$display("");
		end
		else if (sp_initial_occurences == 2) begin
			$display("x00:0x%8h  x01:0x%8h  x02:0x%8h  x03:0x%8h  x04:0x%8h  x05:0x%8h  x06:0x%8h  x07:0x%8h",
				regfile[0], regfile[1], regfile[2], regfile[3], regfile[4], regfile[5], regfile[6], regfile[7]);
			$display("x08:0x%8h  x09:0x%8h  x10:0x%8h  x11:0x%8h  x12:0x%8h  x13:0x%8h  x14:0x%8h  x15:0x%8h",
					regfile[8], regfile[9], regfile[10], regfile[11], regfile[12], regfile[13], regfile[14], regfile[15]);
			$display("x16:0x%8h  x17:0x%8h  x18:0x%8h  x19:0x%8h  x20:0x%8h  x21:0x%8h  x22:0x%8h  x23:0x%8h",
					regfile[16], regfile[17], regfile[18], regfile[19], regfile[20], regfile[21], regfile[22], regfile[23]);
			$display("x24:0x%8h  x25:0x%8h  x26:0x%8h  x27:0x%8h  x28:0x%8h  x29:0x%8h  x30:0x%8h  x31:0x%8h",
					regfile[24], regfile[25], regfile[26], regfile[27], regfile[28], regfile[29], regfile[30], regfile[31]);
			$display("");
			$display("end: sp");
			$finish;
		end
	end

	always @(ecall) begin
		if (ecall) begin
			#1
			for (i = 0; i < 32; i = i + 1) begin
				$display("register x%0d:\t 0x%8h", i, regfile[i]);
			end
			$display("end: ecall");
			$finish;
		end
	end

	initial begin
		for (i = 0; i < 32; i = i + 1) begin
			regfile[i] = 32'h00000000;
		end
		regfile[2] = `MEM_DEPTH_BYTES + `STARTING_ADDR - 4; // sp at end of memory
    end
endmodule // register

/*========================================================================================*/
`define IMM_SEL_I 0
`define IMM_SEL_S 1
`define IMM_SEL_U 2
`define IMM_SEL_B 3
`define IMM_SEL_J 4

module immediategen(
	input [31:0] instr,
	input [2:0] imm_sel,
	output wire [31:0] imm);

	wire [31:0] imm_i;
	wire [31:0] imm_s;
	wire [31:0] imm_u;
	wire [31:0] imm_b;
	wire [31:0] imm_j;

	assign imm_i = { {20{instr[31]}}, instr[31:20] };
	assign imm_s = { {20{instr[31]}}, instr[31:25], instr[11:7] };
	// assign imm_u = { {12{instr[31]}}, instr[31:12] };
	assign imm_u = { instr[31:12], 12'h000 };
	assign imm_b = { {19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
	assign imm_j = { {11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0 };

	// always @(imm_u) begin
	// 	$display("imm_u:\t0x%8h", imm_u);
	// end

	assign imm = (imm_sel == `IMM_SEL_I) ? imm_i :
				 (imm_sel == `IMM_SEL_S) ? imm_s :
				 (imm_sel == `IMM_SEL_U) ? imm_u :
				 (imm_sel == `IMM_SEL_B) ? imm_b :
				 (imm_sel == `IMM_SEL_J) ? imm_j : 32'bX;
endmodule // immediategen

/*========================================================================================*/

module branchcomp(
	input [31:0] rs1,
	input [31:0] rs2,
	input br_un,
	output wire br_eq,
	output wire br_lt);

	wire signed [31:0] rs1_s, rs2_s;
	assign rs1_s = rs1;
	assign rs2_s = rs2; 

	assign br_eq = (rs1 == rs2);
	assign br_lt = br_un ? (rs1 < rs2) : (rs1_s < rs2_s);
endmodule

/*========================================================================================*/

`define ALU_SEL_ADD		0
`define ALU_SEL_SUB		1
`define ALU_SEL_SLL		2
`define ALU_SEL_SLT		3
`define ALU_SEL_SLTU	4
`define ALU_SEL_XOR		5
`define ALU_SEL_SRL		6
`define ALU_SEL_SRA		7
`define ALU_SEL_OR		8
`define ALU_SEL_AND		9
`define ALU_SEL_LUI		10

`define A_SEL_REG	0;
`define A_SEL_PC	1;

`define B_SEL_REG	0;
`define B_SEL_IMM	1;

module alu(
	input signed [31:0] in_a,	
	input signed [31:0] in_b,
	input [3:0] alu_sel,
	output reg signed [31:0] alu_out);
	
	always @(alu_sel or in_a or in_b) begin
		if (alu_sel == `ALU_SEL_ADD) begin
			// $display("alu_se': ALU_SEL_ADD");
			alu_out <= in_a + in_b;
			$display("0x%8h + 0x%8h = 0x%8h", in_a, in_b, in_a + in_b);
		end
		else if (alu_sel == `ALU_SEL_SUB) begin
			// $display("alu_se': ALU_SEL_SUB");
			alu_out <= in_a - in_b;
		end
		else if (alu_sel == `ALU_SEL_SLL) begin
			// $display("alu_se': ALU_SEL_SLL");
			alu_out <= in_a << in_b;
		end
		else if (alu_sel == `ALU_SEL_SLT) begin
			// $display("alu_se': ALU_SEL_SLT");
			alu_out <= in_a < in_b;
		end
		else if (alu_sel == `ALU_SEL_SLTU) begin
			// $display("alu_se': ALU_SEL_SLTU");
			alu_out <= { 1'b0, in_a } < { 1'b0, in_b };
		end
		else if (alu_sel == `ALU_SEL_XOR) begin
			// $display("alu_se': ALU_SEL_XOR");
			alu_out <= in_a ^ in_b;
		end
		else if (alu_sel == `ALU_SEL_SRL) begin
			// $display("alu_se': ALU_SEL_SRL");
			alu_out <= in_a >> in_b;
		end
		else if (alu_sel == `ALU_SEL_SRA) begin
			// $display("alu_se': ALU_SEL_SRA");
			alu_out <= in_a >>> in_b;
		end
		else if (alu_sel == `ALU_SEL_OR) begin
			// $display("alu_se': ALU_SEL_OR");
			alu_out <= in_a | in_b;
		end
		else if (alu_sel == `ALU_SEL_AND) begin
			// $display("alu_se': ALU_SEL_AND");
			alu_out <= in_a & in_b;
		end
		else if (alu_sel == `ALU_SEL_LUI) begin
			alu_out <= in_b;
		end
		else begin
			alu_out <= 32'bX;
		end
	end
endmodule //alu

/*========================================================================================*/

`define A_SEL_DEFAULT	2'b00
`define A_SEL_ALU		2'b01
`define A_SEL_WB		2'b10

`define B_SEL_DEFAULT	2'b00
`define B_SEL_ALU		2'b01
`define B_SEL_WB		2'b10

`define DMEM_DATA_SEL_DEFAULT	1'b0
`define DMEM_DATA_SEL_WB	1'b1

// forwarding control logic
module control_forwarding(
	input [31:0]		instr_ex,
	input [31:0]		instr_mem,
	input [31:0]		instr_wb,
	output reg [1:0]	a_sel,
	output reg [1:0]	b_sel,
	output reg			dmem_data_sel);

	wire [4:0]	rs1_ex;
	wire [4:0]	rs2_ex;

	wire [4:0]	rs2_mem;
	wire [4:0]	rd_mem;

	wire [4:0]	rd_wb;
	
	wire [6:0] opcode_ex;
	wire [6:0] opcode_mem;
	wire [6:0] opcode_wb;

	assign rs1_ex = instr_ex[19:15];
	assign rs2_ex = instr_ex[24:20];

	assign rs2_mem = instr_mem[24:20];
	assign rd_mem = instr_mem[11:7]; 

	assign rd_wb = instr_wb[11:7];

	assign opcode_ex = instr_ex[6:0];
	assign opcode_mem = instr_mem[6:0];
	assign opcode_wb = instr_wb[6:0];

	always @(rd_mem or rd_wb or rs1_ex or rs2_ex or opcode_ex or opcode_mem or opcode_wb or rs2_mem) begin
		// input a
		// rd_mem is meaningless if instr_mem is a store or branch or nop
		// rs1_ex is meaningless if instr_ex is a lui, auipc, or jal or nop
		if (rd_mem == rs1_ex &&
				opcode_mem != 7'b0100011 && opcode_mem != 7'b1100011 && opcode_mem != 7'b0000000 &&
				opcode_ex != 7'b0110111 && opcode_ex != 7'b0010111 && opcode_ex != 7'b1101111 && opcode_mem != 7'b0000000) begin
			// MX bypassing
			a_sel <= `A_SEL_ALU;
		end
		// rd_wb is meaningless if instr_wb is a store or branch or nop
		// rs1_ex is meaningless if instr_ex is a lui, auipc, or jal or nop
		else if (rd_wb == rs1_ex &&
				opcode_wb != 7'b0100011 && opcode_wb != 7'b1100011 && opcode_wb != 7'b0000000 &&
				opcode_ex != 7'b0110111 && opcode_ex != 7'b0010111 && opcode_ex != 7'b1101111 && opcode_mem != 7'b0000000) begin
			// WX bypassing
			a_sel <= `A_SEL_WB;
		end
		else begin
			a_sel <= `A_SEL_DEFAULT;
		end

		// input b
		// rd_mem is meaningless if instr_mem is a store or branch or nop
		// rs2_ex is only meaningful if instr_ex is (ADD to AND)
		if (rd_mem == rs2_ex &&
				opcode_mem != 7'b0100011 && opcode_mem != 7'b1100011 && opcode_mem != 7'b0000000 &&
				opcode_ex == 7'b0110011) begin
			// MX bypassing
			b_sel <= `B_SEL_ALU;
		end
		// rd_wb is meaningless if instr_wb is a store or branch or nop
		// rs2_ex is only meaningful if instr_ex is (ADD to AND)
		else if (rd_wb == rs2_ex &&
				opcode_wb != 7'b0100011 && opcode_wb != 7'b1100011 && opcode_wb != 7'b0000000 &&
				opcode_ex == 7'b0110011) begin
			// WX bypassing
			b_sel <= `B_SEL_WB;
		end
		else begin
			b_sel <= `B_SEL_DEFAULT;
		end

		// dmem dataW
		// rd_wb is meaningless if instr_wb is a store or branch or nop
		// instr_mem has to be a store
		if (rd_wb == rs2_mem &&
				opcode_wb != 7'b0100011 && opcode_wb != 7'b1100011 && opcode_wb != 7'b0000000 &&
				opcode_mem == 7'b0100011) begin
			// WM bypassing
			dmem_data_sel <= `DMEM_DATA_SEL_WB;
		end
		else begin
			dmem_data_sel <= `DMEM_DATA_SEL_DEFAULT;
		end
	end
endmodule

/*========================================================================================*/

module control_stalling (
	input [31:0]		instr_dec,
	input [31:0]		instr_ex,
	output				stall,
	output				stallbj);

	wire [4:0]	rs1_dec;
	wire [4:0]	rs2_dec;

	wire [4:0]	rd_ex;
	
	wire [6:0] opcode_dec;
	wire [6:0] opcode_ex;

	assign rs1_dec = instr_dec[19:15];
	assign rs2_dec = instr_dec[24:20];

	assign rd_ex = instr_ex[11:7];

	assign opcode_dec = instr_dec[6:0];
	assign opcode_ex = instr_ex[6:0];

	// load-to-use case
	assign stall = opcode_ex == 7'b0000011 && (
			(rs1_dec == rd_ex && opcode_dec != 7'b0100011 && opcode_dec != 7'b1100011 && opcode_dec != 7'b0000000) || // instr_dec has rs1
			(rs2_dec == rd_ex && (opcode_dec == 7'b1100011 || opcode_dec == 7'b0110011))); //instr_dec has rs2 but is not store
	
	assign stallbj = opcode_ex == 7'b1100011 || opcode_ex == 7'b1101111 || opcode_ex == 7'b1100111; // jal* and b*
	
	// initial begin
	// 	stall = 1'b0;
	// end
endmodule

/*========================================================================================*/

`define PC_SEL_4	1'b0
`define PC_SEL_ALU	1'b1

// fetch component
module stage_fetch (
	input 				clock,
	input				pc_sel,
	input [31:0]		alu,
	input				stall,
	output reg [31:0]	pc_out,
	output [31:0]		instr_out);

	reg [31:0]	pc_next;

	initial begin
		pc_next = `STARTING_ADDR;
    end

	always @(posedge clock) begin
		#1;
		pc_out = pc_next;
		// #1;
		if (stall) begin
			pc_next = pc_next;
		end
		else if (pc_sel == `PC_SEL_ALU) begin
			pc_next = alu;
		end
		else begin
			pc_next = pc_next + 4;
		end
	end

	// for debugging
	decodedisplay decdisp_dec(
		.clock(clock),
		.pc(pc_out),
		.decode_in(instr_out),
		.stage(3'd0));

	// temp
	// always @(instr_out) begin
	// 	if (instr_out === 32'hX) begin
	// 		$display("bad instr at pc: 0x%8h", pc_out);
	// 		$finish;
	// 	end
	// end
	
	mainmem imem(
		.clock(clock),
		.address(pc_out),
		.data_in(32'hXXXXXXXX),
		.data_out(instr_out),
		.read_write(1'b0),
		.load_instrs(1'b1),
		.wr_width(`WR_WORD));
endmodule // stage_fetch

/*========================================================================================*/

// decodedisplay component (only for printing to console, does not do anything to the simulation)
module decodedisplay (
	input clock,
	input wire [31:0] pc,
	input [31:0] decode_in,
	input [2:0] stage);

	reg [6:0] opcode;
	reg [2:0] funct3;
	reg [6:0] funct7;

	reg [4:0] rd;
	reg [4:0] rs1;
	reg [4:0] rs2;

	reg signed [31:0] imm_i;
	reg signed [31:0] imm_s;
	reg signed [31:0] imm_u;

	reg 		[4:0] shamt;
	reg signed [31:0] addr_b;
	reg signed [31:0] addr_j;

	integer pc_changed = 0;
	integer decode_in_changed = 0;
	integer printed = 0;
	// integer print_stuff = 0; 

	always @(pc) begin
		// $display("stage %d: pc changes to 0x%8h", stage, pc);
		pc_changed = 1;
	end

	always @(decode_in) begin
		// $display("stage %d: instr changes to 0x%8h", stage, decode_in);
		decode_in_changed = 1;
	end
	
	always @(posedge clock) begin
		if (stage == 0) begin
			#7;
		end
		else if (stage == 1) begin
			#6;
		end
		else if (stage == 2) begin
			#5;
		end
		else if (stage == 3) begin
			#4;
		end
		else if (stage == 4) begin
			#3;
		end


		$display("time: %0t\tstage %d", $time, stage);
		opcode = decode_in[6:0];
		funct3 = decode_in[14:12];
		funct7 = decode_in[31:25];

		rd = decode_in[11:7];
		rs1 = decode_in[19:15];
		rs2 = decode_in[24:20];

		imm_i = { {20{decode_in[31]}}, decode_in[31:20] };
		imm_s = { {20{decode_in[31]}}, decode_in[31:25], decode_in[11:7] };
		imm_u = { {12{decode_in[31]}}, decode_in[31:12] };

		shamt = rs2;
		addr_b = pc + { {19{decode_in[31]}}, decode_in[31], decode_in[7], decode_in[30:25], decode_in[11:8], 1'b0 };
		addr_j = pc + { {11{decode_in[31]}}, decode_in[31], decode_in[19:12], decode_in[20], decode_in[30:21], 1'b0 };

		if (decode_in == 32'b000000000000_00000_000_00000_0000000)
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnop",
							$time, pc, decode_in); // NOP

		else if (opcode == 7'b0110111)
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tlui\tx%0d, 0x%0h",
							$time, pc, decode_in, rd, imm_u); // LUI

		else if (opcode == 7'b0010111)
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tauipc\tx%0d, 0x%0h",
							$time, pc, decode_in, rd, imm_u); // AUIPC

		else if (opcode == 7'b1101111) begin
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tjal\tx%0d, 0x%8h",
							$time, pc, decode_in, rd, addr_j); // JAL
		end

		else if (opcode == 7'b1100111)
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tjalr\tx%0d, %0d(x%0d)",
							$time, pc, decode_in, rd, imm_i, rs1); // JALR

		else if (opcode == 7'b1100011) begin
			if (funct3 == 3'b000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tbeq\tx%0d, x%0d, 0x%8h",
								$time, pc, decode_in, rs1, rs2, addr_b); // BEQ
			else if (funct3 == 3'b001)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tbne\tx%0d, x%0d, 0x%8h",
								$time, pc, decode_in, rs1, rs2, addr_b); // BNE
			else if (funct3 == 3'b100)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tblt\tx%0d, x%0d, 0x%8h",
								$time, pc, decode_in, rs1, rs2, addr_b); // BLT
			else if (funct3 == 3'b101)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tbge\tx%0d, x%0d, 0x%8h",
								$time, pc, decode_in, rs1, rs2, addr_b); // BGE
			else if (funct3 == 3'b110)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tbltu\tx%0d, x%0d, 0x%8h",
								$time, pc, decode_in, rs1, rs2, addr_b); // BLTU
			else if (funct3 == 3'b111)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tbgeu\tx%0d, x%0d, 0x%8h",
								$time, pc, decode_in, rs1, rs2, addr_b); // BGEU
			else begin
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnot instr",
								$time, pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (opcode == 7'b0000011) begin
			if (funct3 == 3'b000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tlb\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rd, imm_i, rs1); // LB
			else if (funct3 == 3'b001)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tlh\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rd, imm_i, rs1); // LH
			else if (funct3 == 3'b010)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tlw\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rd, imm_i, rs1); // LW
			else if (funct3 == 3'b100)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tlbu\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rd, imm_i, rs1); // LBU
			else if (funct3 == 3'b101)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tlhu\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rd, imm_i, rs1); // LHU
			else begin
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnot instr",
								$time, pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (opcode == 7'b0100011) begin
			if (funct3 == 3'b000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsb\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rs2, imm_s, rs1); // SB
			else if (funct3 == 3'b001)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsh\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rs2, imm_s, rs1); // SH
			else if (funct3 == 3'b010)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsw\tx%0d, %0d(x%0d)",
								$time, pc, decode_in, rs2, imm_s, rs1); // SW
			else begin
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnot instr",
								$time, pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (opcode == 7'b0010011) begin
			if (funct3 == 3'b000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\taddi\tx%0d, x%0d, %0d",
								$time, pc, decode_in, rd, rs1, imm_i); // ADDI
			else if (funct3 == 3'b010)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tslti\tx%0d, x%0d, %0d",
								$time, pc, decode_in, rd, rs1, imm_i); // SLTI
			else if (funct3 == 3'b011)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsltiu\tx%0d, x%0d, %0d",
								$time, pc, decode_in, rd, rs1, imm_i); // SLTIU
			else if (funct3 == 3'b100)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\txori\tx%0d, x%0d, %0d",
								$time, pc, decode_in, rd, rs1, imm_i); // XORI
			else if (funct3 == 3'b110)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tori\tx%0d, x%0d, %0d",
								$time, pc, decode_in, rd, rs1, imm_i); // ORI
			else if (funct3 == 3'b111)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tandi\tx%0d, x%0d, %0d",
								$time, pc, decode_in, rd, rs1, imm_i); // ANDI
			else if (funct3 == 3'b001 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tslli\tx%0d, x%0d, 0x%0h",
								$time, pc, decode_in, rd, rs1, shamt); // SLLI
			else if (funct3 == 3'b101 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsrli\tx%0d, x%0d, 0x%0h",
								$time, pc, decode_in, rd, rs1, shamt); // SRLI
			else if (funct3 == 3'b101 && funct7 == 7'b0100000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsrai\tx%0d, x%0d, 0x%0h",
								$time, pc, decode_in, rd, rs1, shamt); // SRAI
			else begin
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnot instr",
								$time, pc, decode_in); // NOT INSTRUCTION
			end
		end
		
		else if (opcode == 7'b0110011) begin
			if (funct3 == 3'b000 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tadd\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // ADD
			else if (funct3 == 3'b000 && funct7 == 7'b0100000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsub\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // SUB
			else if (funct3 == 3'b001 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsll\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // SLL
			else if (funct3 == 3'b010 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tslt\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // SLT
			else if (funct3 == 3'b011 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsltu\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // SLTU
			else if (funct3 == 3'b100 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\txor\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // XOR
			else if (funct3 == 3'b101 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsrl\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // SRL
			else if (funct3 == 3'b101 && funct7 == 7'b0100000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tsra\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // SRA
			else if (funct3 == 3'b110 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tor\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // OR
			else if (funct3 == 3'b111 && funct7 == 7'b0000000)
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tand\tx%0d, x%0d, x%0d",
								$time, pc, decode_in, rd, rs1, rs2); // AND
			else begin
				$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnot instr",
								$time, pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (decode_in == 32'b000000000000_00000_000_00000_1110011) begin
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tecall",
							$time, pc, decode_in,); // ECALL
		end

		else begin
			// not instruction
			$display("time: %0t\tpc: 0x%8h\tinstr: 0x%8h\tnot instr",
							$time, pc, decode_in); // NOT INSTRUCTION
		end
	end
endmodule // decodedisplay

/*========================================================================================*/

// decode component
module stage_decode (
	input			clock,
	input [31:0] 	pc,
	input [31:0] 	instr,
	input [31:0]	wb,
	input [31:0]	instr_wb,
	input			stall,
	input			stallbj,
	output reg [31:0] 	pc_out,
	output [31:0]	data_rs1,
	output [31:0]	data_rs2,
	output reg [31:0] 	instr_out);

	wire [31:0] pc_out_nonstall;
	wire [31:0] instr_out_nonstall;

	// wire rs1

	reg reg_wen;

	reg [4:0] rd;
	reg [4:0] rs1;
	reg [4:0] rs2;

	reg [6:0] opcode_wb;
	reg [6:0] opcode;
	reg [2:0] funct3;
	reg [6:0] funct7;

	reg ecall = 0;
	
	// inter-stage flipflops
	flipflop pc_ff(
		.data(pc),
		.clock(clock),
		.reset(1'b0),
		.data_out(pc_out_nonstall));
		// .data_out(pc_out));
	flipflop instr_ff(
		.data(instr),
		.clock(clock),
		.reset(1'b0),
		.data_out(instr_out_nonstall));
		// .data_out(instr_out));

	// for debugging
	decodedisplay decdisp_dec(
		.clock(clock),
		.pc(pc_out),
		.decode_in(instr_out),
		.stage(3'd1));
	
	// stalls
	// assign instr_out = instr_out_nonstall;
	always @(stallbj) begin
		$display("decode: stallbj %0d", stallbj);
	end
	always @(stall) begin
		$display("decode: stall %0d", stall);
	end

	always @(instr_out_nonstall) begin
		if (stallbj == 1'b1) begin
			instr_out <= 32'h00000000;
		end
		else if (stall == 1'b1) begin
			instr_out <= instr_out;
		end
		else begin
			instr_out <= instr_out_nonstall;
		end
	end
	always @(pc_out_nonstall) begin
		if (stall == 1'b1) begin
			pc_out <= pc_out;
		end
		else begin
			pc_out <= pc_out_nonstall;
		end
	end

	// control
	always @(instr_wb) begin
		// writeback destination address
		rd = instr_wb[11:7];

		// reg_wen
		opcode_wb = instr_wb[6:0];
		if (opcode_wb == 7'b1100011 || opcode_wb == 7'b0100011 || opcode_wb == 7'b0000000)
			reg_wen <= 0;
		else
			reg_wen <= 1;
	end

	always @(instr_out) begin
		rs1 = instr_out[19:15];
		rs2 = instr_out[24:20];

		opcode = instr_out[6:0];
		funct3 = instr_out[14:12];
		funct7 = instr_out[31:25];
		// ECALL -- terminate
		if (opcode == 7'b1110011 && funct3 == 3'b000 && funct7 == 7'b0000000)
			ecall = 1;
	end

	// register component
	register reggie(
		.clock(clock),
		.instr(instr_out),
		.addr_rs1(rs1),
		.addr_rs2(rs2),
		.addr_rd(rd),
		.data_rd(wb),
		.data_rs1(data_rs1),
		.data_rs2(data_rs2),
		.write_enable(reg_wen),
		.ecall(ecall));

	initial begin
        $dumpvars(0, reggie);
    end
endmodule // stage_decode

/*========================================================================================*/

module stage_execute(
	input 			clock,
	input [31:0]	pc,
	input [31:0]	instr,
	input [31:0]	data_rs1,
	input [31:0]	data_rs2,
	input [1:0]		a_sel_fwd,
	input [1:0]		b_sel_fwd,
	input [31:0]	alu_fwd,
	input [31:0]	wb_fwd,
	input 			stall,
	input 			stallbj,
	output [31:0]	pc_out,
	output [31:0]	alu_out,
	output reg [31:0]	data_rs2_out,
	// output [31:0]	data_rs2_out,
	output reg [31:0]	instr_out,
	output reg		pc_sel_out);

	// always @(instr) begin
	// 	$display("execute: instr 0x%8h", instr);
	// end
	// always @(instr_out) begin
	// 	$display("execute: instr_out 0x%8h", instr_out);
	// end

	wire [31:0] instr_out_nonstall;

	reg [2:0] imm_sel;
	wire [31:0] imm;

	reg [3:0] alu_sel;
	reg a_sel, b_sel;

	reg [31:0] a_sel_out, b_sel_out;
	
	reg br_un;
	wire br_eq;
	wire br_lt;

	reg [6:0] opcode;
	reg [2:0] funct3;
	reg [6:0] funct7;

	wire [31:0] data_rs1_out_tmp;
	wire [31:0] data_rs2_out_tmp;

	reg [31:0] data_rs1_out;

	// inter-stage flipflops
	flipflop pc_ff(
		.data(pc),
		.clock(clock),
		.reset(1'b0),
		.data_out(pc_out));
	flipflop instr_ff(
		.data(instr),
		.clock(clock),
		.reset(1'b0),
		.data_out(instr_out_nonstall));
		// .data_out(instr_out));
	flipflop rs1_ff(
		.data(data_rs1),
		.clock(clock),
		.reset(1'b0),
		.data_out(data_rs1_out_tmp));
	flipflop rs2_ff(
		.data(data_rs2),
		.clock(clock),
		.reset(1'b0),
		.data_out(data_rs2_out_tmp));

	// for debugging
	decodedisplay decdisp_dec(
		.clock(clock),
		.pc(pc_out),
		.decode_in(instr_out),
		.stage(3'd2));

	// stage componenets
	branchcomp bcmp(
		.rs1(data_rs1_out),
		.rs2(data_rs2_out),
		.br_un(br_un),
		.br_eq(br_eq),
		.br_lt(br_lt));

	immediategen igen(
		// .instr(instr_out_nonstall),
		.instr(instr_out),
		.imm_sel(imm_sel),
		.imm(imm));

	alu alu(
		.in_a(a_sel_out),
		.in_b(b_sel_out),
		.alu_sel(alu_sel),
		.alu_out(alu_out));
	
	// stalls
	// always @(stall) begin
	// 	$display("execute: stall %0d", stall);
	// end
	
	always @(instr_out_nonstall) begin
		if (stall == 1'b1 || stallbj) begin
			instr_out <= 32'h00000000;
			// $display("execute: stall this");
		end
		else begin
			instr_out <= instr_out_nonstall;
		end
	end

	// rs1 selection
	always @(a_sel_fwd or alu_fwd or wb_fwd) begin
		if (a_sel_fwd == `A_SEL_ALU) begin
			$display("mx bypass rs1");
			data_rs1_out <= alu_fwd;
		end
		else if (a_sel_fwd == `A_SEL_WB) begin
			$display("wx bypass rs1");
			data_rs1_out <= wb_fwd;
		end
		else begin
			$display("rs1 is data_rs1");
			data_rs1_out <= data_rs1_out_tmp;
		end
	end
	
	// rs2 selection

	// input a selection
	always @(a_sel or pc_out or data_rs1_out) begin
		if (a_sel) begin
			$display("input a is pc");
			a_sel_out <= pc_out;
		end
		else begin
			$display("input a is data_rs1");
			a_sel_out <= data_rs1_out;
		end
	end
	// always @(a_sel_fwd or alu_fwd or wb_fwd or a_sel or pc_out or data_rs1_out) begin
	// 	if (a_sel_fwd == `A_SEL_ALU) begin
	// 		$display("mx bypass a");
	// 		a_sel_out <= alu_fwd;
	// 	end
	// 	else if (a_sel_fwd == `A_SEL_WB) begin
	// 		$display("wx bypass a");
	// 		a_sel_out <= wb_fwd;
	// 	end
	// 	else if (a_sel) begin
	// 		$display("input a is pc");
	// 		a_sel_out <= pc_out;
	// 	end
	// 	else begin
	// 		$display("input a is data_rs1");
	// 		a_sel_out <= data_rs1_out;
	// 	end
	// end

	// rs2 selection
	always @(b_sel_fwd or alu_fwd or wb_fwd) begin
		if (b_sel_fwd == `B_SEL_ALU) begin
			$display("mx bypass rs2");
			data_rs2_out <= alu_fwd;
		end
		else if (b_sel_fwd == `B_SEL_WB) begin
			$display("wx bypass rs2");
			data_rs2_out <= wb_fwd;
		end
		else begin
			$display("rs2 is data_rs2");
			data_rs2_out <= data_rs2_out_tmp;
		end
	end

	// input b selection
	always @(b_sel or imm or data_rs2_out) begin
		if (b_sel) begin
			b_sel_out <= imm;
		end
		else begin
			b_sel_out <= data_rs2_out;
		end
	end
	// always @(b_sel_fwd or alu_fwd or wb_fwd or b_sel or imm or data_rs2_out) begin
	// 	if (b_sel_fwd == `B_SEL_ALU) begin
	// 		$display("mx bypass b");
	// 		b_sel_out <= alu_fwd;
	// 	end
	// 	else if (b_sel_fwd == `B_SEL_WB) begin
	// 		$display("wx bypass b");
	// 		b_sel_out <= wb_fwd;
	// 	end
	// 	else if (b_sel) begin
	// 		b_sel_out <= imm;
	// 	end
	// 	else begin
	// 		b_sel_out <= data_rs2_out;
	// 	end
	// end


	// control
	always @(instr_out) begin
		opcode = instr_out[6:0];
		funct3 = instr_out[14:12];
		funct7 = instr_out[31:25];
	// always @(instr_out_nonstall) begin
	// 	opcode = instr_out_nonstall[6:0];
	// 	funct3 = instr_out_nonstall[14:12];
	// 	funct7 = instr_out_nonstall[31:25];

		// imm_sel
		if (opcode[4:0] == 5'b10111) begin
			imm_sel <= `IMM_SEL_U;
		end
		else if (opcode == 7'b1101111) begin
			imm_sel <= `IMM_SEL_J;
		end
		else if (opcode == 7'b1100011) begin
			imm_sel <= `IMM_SEL_B;
		end
		else if (opcode == 7'b0100011) begin
			imm_sel <= `IMM_SEL_S;
		end
		else begin
			imm_sel <= `IMM_SEL_I;
		end

		// br_un
		if (opcode == 7'b1100011)
			br_un <= (funct3[2:1] == 2'b11);
		
		// a_sel
		// $display("instr_out %32b", instr_out);
		// $display("opcode %7b", opcode);
		if (opcode == 7'b1100011 || opcode == 7'b1101111 || opcode == 7'b0010111) begin
			a_sel <= `A_SEL_PC;
			// $display("a_sel: %s", "A_SEL_PC");
		end
		else begin
			a_sel <= `A_SEL_REG;
			// $display("a_sel: %s", "A_SEL_REG");
		end

		// b_sel
		if (opcode == 7'b0110011) begin
			b_sel <= `B_SEL_REG;
			// $display("b_sel: %s", "B_SEL_REG");
		end
		else begin
			b_sel <= `B_SEL_IMM;
			// $display("b_sel: %s", "B_SEL_IMM");
		end

		// alu_sel
		if (opcode == 7'b0110111) begin
			alu_sel <= `ALU_SEL_LUI;
		end
		else if ({ funct7[5], funct3, opcode } == 11'b1_000_0110011) begin
			alu_sel <= `ALU_SEL_SUB;
		end
		else if (opcode == 7'b0010011 || opcode == 7'b0110011) begin
			if (funct3 == 3'b001) begin
				alu_sel <= `ALU_SEL_SLL;
			end
			else if (funct3 == 3'b010) begin
				alu_sel <= `ALU_SEL_SLT;
			end
			else if (funct3 == 3'b011) begin
				alu_sel <= `ALU_SEL_SLTU;
			end
			else if (funct3 == 3'b100) begin
				alu_sel <= `ALU_SEL_XOR;
			end
			else if ({ funct7[5], funct3 } == 4'b1_101) begin
				alu_sel <= `ALU_SEL_SRA;
			end
			else if ({ funct7[5], funct3 } == 4'b0_101) begin
				alu_sel <= `ALU_SEL_SRL;
			end
			else if (funct3 == 3'b110) begin
				alu_sel <= `ALU_SEL_OR;
			end
			else if (funct3 == 3'b111) begin
				alu_sel <= `ALU_SEL_AND;
			end
			else begin
				alu_sel <= `ALU_SEL_ADD;
			end
		end
		else
			alu_sel <= `ALU_SEL_ADD;
	end

	always @(instr_out or br_eq or br_lt) begin
		opcode = instr_out[6:0];
		funct3 = instr_out[14:12];
		funct7 = instr_out[31:25];
	// always @(instr_out_nonstall or br_eq or br_lt) begin
	// 	opcode = instr_out_nonstall[6:0];
	// 	funct3 = instr_out_nonstall[14:12];
	// 	funct7 = instr_out_nonstall[31:25];

		// pc_sel
		if (opcode == 7'b1101111 || opcode == 7'b1100111) begin
			// JAL and JALR
			pc_sel_out <= `PC_SEL_ALU;
		end
		else if (opcode == 7'b1100011) begin
			if (funct3[2] == 0 && funct3[0] == ~br_eq) begin
				pc_sel_out <= `PC_SEL_ALU;
			end
			else if (funct3[2] == 1 && funct3[0] == ~br_lt) begin
				pc_sel_out <= `PC_SEL_ALU;
			end
			else begin
				pc_sel_out <= `PC_SEL_4;
			end
		end
		else begin
			pc_sel_out <= `PC_SEL_4;
		end
	end
endmodule // stage_execute

/*========================================================================================*/

`define WB_MEM			2;
`define WB_ALU			1;
`define WB_PC_PLUS_4	0;
module stage_memory(
	input			clock,
	input [31:0]	pc,
	input [31:0]	alu,
	input [31:0]	data_rs2,
	input [31:0]	instr,
	input [31:0]	wb_fwd,
	input 			dmem_data_sel_fwd,
	output [31:0]	alu_fwd_out,
	output reg [31:0]	wb_out,
	output [31:0]	pc_out,
	output [31:0]	instr_out);

	reg [6:0] opcode;
	reg [2:0] funct3;

	wire [31:0] alu_out;
	wire [31:0] data_rs2_out;

	reg [31:0] dmem_data_in;

	wire [31:0] mem_out;
	reg [31:0] mem_out_masked;

	reg [1:0] wr_width;
	reg [1:0] wb_sel;
	reg 		mem_rw;

	assign alu_fwd_out = alu_out;

	// inter-stage flipflops
	flipflop pc_ff(
		.data(pc),
		.clock(clock),
		.reset(1'b0),
		.data_out(pc_out));
	flipflop instr_ff(
		.data(instr),
		.clock(clock),
		.reset(1'b0),
		.data_out(instr_out));
	flipflop alu_ff(
		.data(alu),
		.clock(clock),
		.reset(1'b0),
		.data_out(alu_out));
	flipflop rs2_ff(
		.data(data_rs2),
		.clock(clock),
		.reset(1'b0),
		.data_out(data_rs2_out));

	// for debugging
	decodedisplay decdisp_dec(
		.clock(clock),
		.pc(pc_out),
		.decode_in(instr_out),
		.stage(3'd3));

	// components
	mainmem dmem(
		.clock(clock),
		.address(alu_out),
		.data_in(dmem_data_in),
		.data_out(mem_out),
		.read_write(mem_rw),
		.load_instrs(1'b1),
		.wr_width(wr_width));

	// DMEM data input condition
	always @(dmem_data_sel_fwd or wb_fwd or data_rs2_out) begin
		if (dmem_data_sel_fwd == `DMEM_DATA_SEL_WB) begin
			dmem_data_in <= wb_fwd;
			$display("wm bypass");
		end
		else begin
			dmem_data_in <= data_rs2_out;
		end
	end

	// always @(alu_out) begin
	// 	$display("alu_out: 0x%8h", alu_out);
	// end

	// wb mux
	always @(pc_out or alu_out or mem_out_masked or wb_sel) begin
		if (wb_sel == 2) begin
			wb_out <= mem_out_masked;
		end
		else if (wb_sel == 1) begin
			wb_out <= alu_out;
		end
		else begin
			wb_out <= pc_out + 4;
		end
	end

	// always @(data_rs2_out) begin
	// 	$display("memin: 0x%8h", data_rs2_out);
	// end
	// always @(mem_out) begin
	// 	$display("memout: 0x%8h", mem_out);
	// end

	//control
	// adjust for LB, LH, LW, LBU, LHU, SB, SH, SW
	always @(instr_out or data_rs2_out or mem_out) begin
		opcode = instr_out[6:0];
		funct3 = instr_out[14:12];

		// mem_rw
		if (opcode == 7'b0100011)
			mem_rw <= `WRITE;
		else
			mem_rw <= `READ;

		// wb_sel
		// `define WB_MEM			2;
		// `define WB_ALU			1;
		// `define WB_PC_PLUS_4		0;
		if (opcode == 7'b0000011)
			wb_sel <= 2;
		else if (opcode == 7'b1101111 || opcode == 7'b1100111)
			wb_sel <= 0;
		else
			wb_sel <= 1;

		// wr_width
		if (opcode == 7'b0000011) begin
			wr_width <= `WR_WORD;
			if (funct3 == 3'b000) begin
				// LB
				mem_out_masked <= { {24{mem_out[7]}}, mem_out[7:0] };
			end
			else if (funct3 == 3'b001) begin
				// LH
				mem_out_masked <= { {16{mem_out[15]}}, mem_out[15:0] };
			end
			else if (funct3 == 3'b100) begin
				// LBU
				mem_out_masked <= { 24'h000000, mem_out[7:0] };
			end
			else if (funct3 == 3'b101) begin
				// LHU
				mem_out_masked <= { 16'h0000, mem_out[15:0] };
			end
			else begin
				// LW
				mem_out_masked <= mem_out;
			end
		end
		else if (opcode == 7'b0100011) begin
			mem_out_masked <= mem_out;
			if (funct3 == 3'b000)		// SB
				wr_width <= `WR_BYTE;
			else if (funct3 == 3'b001)	// SH
				wr_width <= `WR_HALFW;
			else						// SW
				wr_width <= `WR_WORD;
		end
		else begin
			wr_width <= `WR_WORD;
			mem_out_masked <= mem_out;
		end
	end
endmodule // stage_memory

/*========================================================================================*/

module stage_writeback(
	input 			clock,
	input [31:0]	pc,
	input [31:0]	wb,
	input [31:0]	instr,
	output [31:0]	wb_out,
	output [31:0]	instr_wb_out);

	wire [31:0] pc_out;

	// inter-stage flipflops
	flipflop wb_ff(
		.data(wb),
		.clock(clock),
		.reset(1'b0),
		.data_out(wb_out));
	flipflop instr_ff(
		.data(instr),
		.clock(clock),
		.reset(1'b0),
		.data_out(instr_wb_out));
	flipflop pc_ff(
		.data(pc),
		.clock(clock),
		.reset(1'b0),
		.data_out(pc_out));

	// always @(pc_out) begin
	// 	$display("wb pc: 0x%8h", pc_out);
	// end

	always @(instr_wb_out) begin
		$display("mem: instr_wb_out: 0x%8h", instr_wb_out);
	end

	// for debugging
	decodedisplay decdisp_dec(
		.clock(clock),
		.pc(pc_out),
		.decode_in(instr_wb_out),
		.stage(3'd4));

	// always @(wb_out) begin
	// 	$display("wb output: 0x%8h", wb_out);
	// end
endmodule // stage_writeback

/*========================================================================================*/

// testbench
module dut;
	// system clock
	reg clock = 0;

	// pc select
	wire pc_sel;

	// stall signal
	wire stall_sig;
	wire stallbj_sig; // for branches and jumps

	// bypass selects
	wire [1:0]	a_sel_fwd;
	wire [1:0]	b_sel_fwd;
	wire		dmem_data_sel_fwd;

	// program counters
	wire [31:0] pc_dec;
	wire [31:0] pc_ex;
	wire [31:0] pc_mem;
	wire [31:0] pc_wb;

	// instructions
	wire [31:0] instr_fet_dec;
	wire [31:0] instr_ex;
	wire [31:0] instr_mem;
	wire [31:0] instr_wb;
	wire [31:0] instr_wb_dec;

	// register out
	wire [31:0] data_rs1_ex;
	wire [31:0] data_rs2_ex;
	wire [31:0] data_rs2_mem;
	
	// alu
	wire [31:0]	alu_mem;
	wire [31:0]	alu_fwd;

	// writeback
	wire [31:0] wb_wb;
	wire [31:0] wb_dec;

	control_forwarding fwd(
		.instr_ex(instr_mem),
		.instr_mem(instr_wb),
		.instr_wb(instr_wb_dec),
		.a_sel(a_sel_fwd),
		.b_sel(b_sel_fwd),
		.dmem_data_sel(dmem_data_sel_fwd));

	control_stalling stall(
		.instr_dec(instr_ex),
		.instr_ex(instr_mem),
		.stall(stall_sig),
		.stallbj(stallbj_sig));
	
	always @(stall_sig) begin
		$display("load to use stall sig: %0d", stall_sig);
	end
    
	stage_fetch stage_fet (
		.clock(clock),
		.pc_sel(pc_sel),
		.alu(alu_mem),
		.stall(stall_sig),
		.pc_out(pc_dec),
		.instr_out(instr_fet_dec));

	stage_decode stage_dec (
		.clock(clock),
		.pc(pc_dec),
		.instr(instr_fet_dec),
		.wb(wb_dec),
		.instr_wb(instr_wb_dec),
		.stall(stall_sig),
		.stallbj(stallbj_sig),
		.pc_out(pc_ex),
		.data_rs1(data_rs1_ex),
		.data_rs2(data_rs2_ex),
		.instr_out(instr_ex));

	stage_execute stage_ex (
		.clock(clock),
		.pc(pc_ex),
		.instr(instr_ex),
		.data_rs1(data_rs1_ex),
		.data_rs2(data_rs2_ex),
		.a_sel_fwd(a_sel_fwd),
		.b_sel_fwd(b_sel_fwd),
		.wb_fwd(wb_dec),
		.alu_fwd(alu_fwd),
		.stall(stall_sig),
		.stallbj(stallbj_sig),
		.pc_out(pc_mem),
		.alu_out(alu_mem),
		.data_rs2_out(data_rs2_mem),
		.instr_out(instr_mem),
		.pc_sel_out(pc_sel));
	
	stage_memory stage_mem(
		.clock(clock),
		.pc(pc_mem),
		.alu(alu_mem),
		.data_rs2(data_rs2_mem),
		.instr(instr_mem),
		.wb_fwd(wb_dec),
		.dmem_data_sel_fwd(dmem_data_sel_fwd),
		.alu_fwd_out(alu_fwd),
		.wb_out(wb_wb),
		.pc_out(pc_wb),
		.instr_out(instr_wb));

	stage_writeback stage_wb(
		.clock(clock),
		.pc(pc_wb),
		.wb(wb_wb),
		.instr(instr_wb),
		.wb_out(wb_dec),
		.instr_wb_out(instr_wb_dec));

	initial begin
		$dumpfile(`VCD);
        $dumpvars(0, fwd);
        $dumpvars(0, stage_fet);
        $dumpvars(0, stage_dec);
        $dumpvars(0, stage_ex);
        $dumpvars(0, stage_mem);
        $dumpvars(0, stage_wb);

		// #(1503211170 * 4 * `HALF_CYCLE) $finish;
		#(1000 * 4 * `HALF_CYCLE) $display("out of time!");
		$finish;
    end

	// always @(alu) begin
	// 	$display("alu: 0x%8h", alu);
	// end

	always begin                 
		#`HALF_CYCLE clock = ~clock;
	end
endmodule
