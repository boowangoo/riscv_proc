`define FILE_PATH "../../rv32-benchmarks-master/simple-programs/SumArray.x"
`define FILE_LINES 39
`define VCD "mem-wb-SumArray.vcd"

`define HALF_CYCLE 10

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
	input [31:0] address,
	input [31:0] data_in,
	output [31:0] data_out,
	input read_write,
	input load_instrs,
	input [1:0] wr_width); 

	wire read_write;
	wire [31:0] address;
	wire [31:0] data_in;
	reg [31:0] data_out;

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

	// write mode
    // always @(posedge clock) begin
    //     if (read_write == `WRITE) begin
    //         mem[address - `STARTING_ADDR + 3] <= data_in[31:24];
    //         mem[address - `STARTING_ADDR + 2] <= data_in[23:16];
    //         mem[address - `STARTING_ADDR + 1] <= data_in[15:8];
    //         mem[address - `STARTING_ADDR] <= data_in[7:0];
    //     end
	// end
	always @(posedge clock) begin
        if (read_write == `WRITE) begin
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
	input [4:0] addr_rs1,
	input [4:0] addr_rs2,
	input [4:0] addr_rd,
	input [31:0] data_rd,
	output [31:0] data_rs1,
	output [31:0] data_rs2,
	input write_enable);

	reg [31:0] regfile [0:31];

	assign data_rs1 = regfile[addr_rs1];
	assign data_rs2 = regfile[addr_rs2];

	integer sp_initial_occurences = 0;
	always @(regfile[2]) begin
		$display("sp:\t0x%8h", regfile[2]);

		if (regfile[2] == `MEM_DEPTH_BYTES + `STARTING_ADDR - 1)
			sp_initial_occurences += 1;
		if (sp_initial_occurences == 2)
			$finish; 
	end

	always @(posedge clock) begin
		if (write_enable && addr_rd != 0)
			regfile[addr_rd] <= data_rd;
	end

	integer i;
	initial begin
		for (i = 0; i < 32; i = i + 1) begin
			if (i == 2)
				regfile[2] = `MEM_DEPTH_BYTES + `STARTING_ADDR - 1; // sp at end
			else
				regfile[i] = 32'h00000000;
		end
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
	output [31:0] imm);

	wire [31:0] imm_i;
	wire [31:0] imm_s;
	wire [31:0] imm_u;
	wire [31:0] imm_b;
	wire [31:0] imm_j;
	wire imm;

	assign imm_i = { {20{instr[31]}}, instr[31:20] };
	assign imm_s = { {20{instr[31]}}, instr[31:25], instr[11:7] };
	assign imm_u = { {12{instr[31]}}, instr[31:12] };
	assign imm_b = { {19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
	assign imm_j = { {11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0 };

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
	output br_eq,
	output br_lt);

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
	output signed [31:0] alu_out);

	reg signed [31:0] alu_out;

	// always @(in_a) begin
	// 	$display("in_a: %8h", in_a);
	// end

	// always @(in_b) begin
	// 	$display("in_b: %8h", in_b);
	// end
	
	always @(alu_sel or in_a or in_b) begin
		if (alu_sel == `ALU_SEL_ADD) begin
			// $display("alu_se': ALU_SEL_ADD");
			alu_out <= in_a + in_b;
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
			alu_out[31:12] <= in_b[19:0];
		end
		else begin
			alu_out <= 32'bX;
		end
	end
endmodule //alu

/*========================================================================================*/

`define PC_SEL_4	1'b0
`define PC_SEL_ALU	1'b1

// fetch component
module stage_fetch (
	input 			clock,
	input 			pc_sel,
	input [31:0]	alu,
	output [31:0]	pc_out,
	output [31:0]	instr_out);

	reg [31:0]	pc_next;
	reg [31:0]	pc_out;

	initial begin
		pc_next = `STARTING_ADDR;
    end

	always @(posedge clock) begin
		pc_out = pc_next;
		#1
		if (pc_sel == `PC_SEL_ALU) begin
			pc_next = alu;
		end
		else begin
			pc_next = pc_next + 4;
		end
	end

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
	input [31:0] pc,
	input [31:0] decode_in);

	wire [31:0] pc;

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

	always @(decode_in) begin
		$display(" ");
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

		if (opcode == 7'b0110111)
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
	input [31:0] 	instr,
	input			br_eq,
	input			br_lt,
	input [31:0]	wb,
	output 			pc_sel,
	output [2:0] 	imm_sel,
	output 			reg_wen,
	output 			br_un,
	output 			a_sel,
	output 			b_sel,
	output [3:0]	alu_sel,
	output			mem_rw,
	output [1:0]	wb_sel,
	output [31:0]	data_rs1,
	output [31:0]	data_rs2,
	output [31:0] 	instr_out);

	wire [4:0] rd;
	wire [4:0] rs1;
	wire [4:0] rs2;

	assign rd = instr[11:7];
	assign rs1 = instr[19:15];
	assign rs2 = instr[24:20];
	assign instr_out = instr;

	reg 		pc_sel;
	reg [2:0] 	imm_sel;
	reg 		reg_wen;
	reg 		br_un;
	reg 		a_sel;
	reg 		b_sel;
	reg [3:0]	alu_sel;
	reg			mem_rw;
	reg [1:0]	wb_sel;

	register reggie(
		.clock(clock),
		.addr_rs1(rs1),
		.addr_rs2(rs2),
		.addr_rd(rd),
		.data_rd(wb),
		.data_rs1(data_rs1),
		.data_rs2(data_rs2),
		.write_enable(reg_wen));

	reg [6:0] opcode;
	reg [2:0] funct3;
	reg [6:0] funct7;

	always @(instr or br_eq or br_lt) begin
		// $display("instr: %32b", instr);

		opcode = instr[6:0];
		funct3 = instr[14:12];
		funct7 = instr[31:25];

		// pc_sel
		if (opcode == 7'b1101111 || opcode == 7'b1100111) begin
			// JAL and JALR
			pc_sel <= `PC_SEL_ALU;
		end
		else if (opcode == 7'b1100011) begin
			if (funct3[2] == 0 && funct3[0] == ~br_eq) begin
				pc_sel <= `PC_SEL_ALU;
			end
			else if (funct3[2] == 1 && funct3[0] == ~br_lt) begin
				pc_sel <= `PC_SEL_ALU;
			end
			else begin
				pc_sel <= `PC_SEL_4;
			end
		end
		else begin
			pc_sel <= `PC_SEL_4;
		end
	end
	
	always @(instr) begin
		// imm_sel
		if (opcode[3:0] == 4'b0111) begin
			// $display("time=%0t\tu-type", $time);
			imm_sel <= `IMM_SEL_U;
		end
		else if (opcode == 7'b1101111) begin
			// $display("time=%0t\tj-type", $time);
			imm_sel <= `IMM_SEL_J;
		end
		else if (opcode == 7'b1100011) begin
			// $display("time=%0t\tb-type", $time);
			imm_sel <= `IMM_SEL_B;
		end
		else if (opcode == 7'b0100011) begin
			// $display("time=%0t\ts-type", $time);
			imm_sel <= `IMM_SEL_S;
		end
		else begin
			// $display("time=%0t\ti-type", $time);
			imm_sel <= `IMM_SEL_I;
		end

		// br_un
		if (opcode == 7'b1100011)
			br_un <= (funct3[2:1] == 2'b11);
		
		// a_sel
		if (opcode == 7'b1100011 || opcode == 7'b1101111 || opcode == 7'b0010111) begin
			a_sel <= `A_SEL_PC;
		end
		else begin
			a_sel <= `A_SEL_REG;
		end

		// b_sel
		if (opcode == 7'b0110011) begin
			b_sel <= `B_SEL_REG;
		end
		else begin
			b_sel <= `B_SEL_IMM;
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
		
		// mem_rw
		if (opcode == 7'b0100011)
			mem_rw <= 1;
		else
			mem_rw <= 0; 

		// reg_wen
		if (opcode == 7'b1100011 || opcode == 7'b0100011)
			reg_wen <= 0;
		else
			reg_wen <= 1;

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
	end
endmodule // stage_decode

/*========================================================================================*/

module stage_execute(
	input [31:0]	pc,
	input [31:0]	instr,
	input [31:0]	data_rs1,
	input [31:0]	data_rs2,
	input [2:0]		imm_sel,
	input			br_un,
	output			br_eq,
	output 			br_lt,
	input			a_sel,
	input			b_sel,
	input [3:0]		alu_sel,
	output [31:0]	alu_out,
	output [31:0]	instr_out);

	wire [31:0] imm;
	wire [31:0] a_sel_out, b_sel_out;

	assign a_sel_out = a_sel ? pc : data_rs1;
	assign b_sel_out = b_sel ? imm : data_rs2;

	assign instr_out = instr;

	branchcomp bcmp(
		.rs1(data_rs1),
		.rs2(data_rs2),
		.br_un(br_un),
		.br_eq(br_eq),
		.br_lt(br_lt));

	immediategen igen(
		.instr(instr),
		.imm_sel(imm_sel),
		.imm(imm));

	alu alu(
		.in_a(a_sel_out),
		.in_b(b_sel_out),
		.alu_sel(alu_sel),
		.alu_out(alu_out));
endmodule // stage_execute

/*========================================================================================*/

`define WB_MEM			2;
`define WB_ALU			1;
`define WB_PC_PLUS_4	0;
module stage_memory(
	input			clock,
	input [31:0]	pc,
	input [31:0]	alu,
	input [31:0]	mem_in,
	input			mem_rw,
	input [1:0]		wb_sel,
	input [31:0]	instr,
	output [31:0]	wb,
	output [31:0]	instr_out);

	reg [31:0] wb;
	wire [31:0] mem_out;

	reg [1:0] wr_width;
	reg [31:0] mem_out_masked;

	assign instr_out = instr;
	
	// always @(pc or alu or mem_out or wb_sel) begin
	always @(pc or alu or mem_out_masked or wb_sel) begin
		if (wb_sel == 2) begin
			// wb <= mem_out;
			wb <= mem_out_masked;
		end
		else if (wb_sel == 1) begin
			wb <= alu;
		end
		else begin
			wb <= pc + 4;
		end
	end

	// adjust for LB, LH, LW, LBU, LHU, SB, SH, SW
	wire [6:0] opcode;
	wire [2:0] funct3;

	assign opcode = instr[6:0];
	assign funct3 = instr[14:12];

	always @(instr or mem_out) begin
		if (opcode == 7'b0000011) begin
			wr_width <= `WR_WORD;
			if (funct3 == 3'b000)		// LB
				mem_out_masked <= { {24{mem_out[7]}}, mem_out[7:0] };
			else if (funct3 == 3'b001)	// LH
				mem_out_masked <= { {16{mem_out[7]}}, mem_out[7:0] };
			else if (funct3 == 3'b100)	// LBU
				mem_out_masked <= { {24{1'b0}}, mem_out[7:0] };
			else if (funct3 == 3'b101)	// LHU
				mem_out_masked <= { {16{1'b0}}, mem_out[15:0] };
			else						// LW
				mem_out_masked <= mem_out;
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

	mainmem dmem(
		.clock(clock),
		.address(alu),
		.data_in(mem_in),
		.data_out(mem_out),
		.read_write(mem_rw),
		.load_instrs(1'b0),
		.wr_width(wr_width));
		// .wr_width(`WR_WORD));
endmodule // stage_memory

/*========================================================================================*/

module stage_writeback(
	input [31:0]	wb_in,
	input [31:0]	instr,
	output [31:0]	wb_out,
	output [31:0]	instr_out);

	assign wb_out = wb_in;
	assign instr_out = instr;
endmodule // stage_writeback

/*========================================================================================*/

// testbench
module dut;
	reg clock = 0;
	wire [31:0] pc;

	// instructions
	// wire [31:0] instr_fet;
	wire [31:0] instr_dec;
	wire [31:0] instr_ex;
	wire [31:0] instr_mem;
	wire [31:0] instr_wb;

	// control signals
	wire 		pc_sel;
	wire [2:0] 	imm_sel;
	wire		reg_wen;
	wire 		a_sel;
	wire 		b_sel;
	wire [3:0]	alu_sel;
	wire		mem_rw;
	wire [1:0]	wb_sel;

	// branch comp
	wire 		br_un;
	wire 		br_eq;
	wire 		br_lt;

	// register out
	wire [31:0] data_rs1;
	wire [31:0] data_rs2;
	
	// alu
	wire [31:0]	alu;

	// writeback
	wire [31:0] wb_wb;
	wire [31:0] wb_dec;
	// assign wb = alu; // temporary

	decodedisplay decdisp(
		.pc(pc),
		.decode_in(instr_dec));
    
	stage_fetch stage_fet (
		.clock(clock),
		.pc_sel(pc_sel),
		.alu(alu),
		.pc_out(pc),
		.instr_out(instr_dec));

	stage_decode stage_dec (
		.clock(clock),
		.instr(instr_dec),
		.br_eq(br_eq),
		.br_lt(br_lt),
		.wb(wb_dec),
		.pc_sel(pc_sel),
		.imm_sel(imm_sel),
		.reg_wen(reg_wen),
		.br_un(br_un),
		.a_sel(a_sel),
		.b_sel(b_sel),
		.alu_sel(alu_sel),
		.mem_rw(mem_rw),
		.wb_sel(wb_sel),
		.data_rs1(data_rs1),
		.data_rs2(data_rs2),
		.instr_out(instr_ex));

	stage_execute stage_ex (
		.pc(pc),
		.instr(instr_ex),
		.data_rs1(data_rs1),
		.data_rs2(data_rs2),
		.imm_sel(imm_sel),
		.br_un(br_un),
		.br_eq(br_eq),
		.br_lt(br_lt),
		.a_sel(a_sel),
		.b_sel(b_sel),
		.alu_sel(alu_sel),
		.alu_out(alu),
		.instr_out(instr_mem));
	
	stage_memory stage_mem(
		.clock(clock),
		.pc(pc),
		.alu(alu),
		.mem_in(data_rs2),
		.mem_rw(mem_rw),
		.wb_sel(wb_sel),
		.instr(instr_mem),
		.wb(wb_wb),
		.instr_out(instr_wb));

	stage_writeback stage_wb(
		.wb_in(wb_wb),
		.wb_out(wb_dec));

	initial begin
		$dumpfile(`VCD);
        $dumpvars(0, stage_fet);
        $dumpvars(0, stage_dec);
        $dumpvars(0, stage_ex);
        $dumpvars(0, stage_mem);
        $dumpvars(0, stage_wb);

		#(300 * 4 * `HALF_CYCLE) $finish;
    end

	always @(posedge clock) begin                 
		#1
		$display("data_rs1: 0x%8h\tdata_rs2: 0x%8h", data_rs1, data_rs2);
		$display("alu: 0x%8h", alu);
	end
	
	
	always begin                 
		#`HALF_CYCLE clock = ~clock;
	end
endmodule
