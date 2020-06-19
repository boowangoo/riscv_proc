`define FILE_PATH "../../rv32-benchmarks-master/simple-programs/SumArray.x"
`define FILE_LINES 39
`define VCD "fetch-decode-SumArray.vcd"

`define MEM_DEPTH_BYTES 'h0100000
`define STARTING_ADDR 'h01000000

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

	wire read_write;
	wire [31:0] address;
	wire [31:0] data_in;
	reg [31:0] data_out;

	reg [7:0] mem[0:`MEM_DEPTH_BYTES - 1];

    // read mode
	always @(clock or address or read_write) begin
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
    always @(posedge clock) begin
        if (read_write == `WRITE) begin
            mem[address - `STARTING_ADDR + 3] <= data_in[31:24];
            mem[address - `STARTING_ADDR + 2] <= data_in[23:16];
            mem[address - `STARTING_ADDR + 1] <= data_in[15:8];
            mem[address - `STARTING_ADDR] <= data_in[7:0];
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

// fetch component
module fetch (
	input clock,
	input fetch_en,
	input [31:0] pc_in,
	output [31:0] pc_out,
	output [31:0] fetch_out);

	wire [31:0] pc_out;

	initial begin
		$dumpfile(`VCD);
        $dumpvars(0, mem);
    end

	assign pc_out = pc_in + ({32{fetch_en}} & 4);

	mainmem mem(
		.clock(clock),
		.address(pc_in),
		.data_in(32'hXXXXXXXX),
		.data_out(fetch_out),
		.read_write(~fetch_en));

endmodule // fetch

/*========================================================================================*/

// decode component
module decode (
	input clock,
	input [31:0] pc,
	input [31:0] decode_in);

	wire [31:0] pc;

	wire [6:0] opcode;
	wire [2:0] funct3;
	wire [6:0] funct7;

	wire [4:0] rd;
	wire [4:0] rs1;
	wire [4:0] rs2;

	// wire signed [11:0] imm_i;
	// wire signed [11:0] imm_s;
	// wire signed [4:0] imm_u;
	wire signed [31:0] imm_i;
	wire signed [31:0] imm_s;
	wire signed [31:0] imm_u;

	wire 		[4:0] shamt;
	// wire signed [12:0] addr_b;
	// wire signed [31:0] addr_j;
	wire signed [31:0] addr_b;
	wire signed [31:0] addr_j;

	assign opcode = decode_in[6:0];
	assign funct3 = decode_in[14:12];
	assign funct7 = decode_in[31:25];

	assign rd = decode_in[11:7];
	assign rs1 = decode_in[19:15];
	assign rs2 = decode_in[24:20];

	assign imm_i = { {20{decode_in[31]}}, decode_in[31:20] };
	assign imm_s = { {20{decode_in[31]}}, decode_in[31:25], decode_in[11:7] };
	assign imm_u = { {12{decode_in[31]}}, decode_in[31:12] };

	assign shamt = rs2;
	assign addr_b = pc + { {19{decode_in[31]}}, decode_in[31], decode_in[7], decode_in[30:25], decode_in[11:8], 1'b0 };
								//   1             1                 6                4     1
	assign addr_j = pc + { {11{decode_in[31]}}, decode_in[31], decode_in[19:12], decode_in[20], decode_in[30:21], 1'b0 };

	always @(decode_in) begin
		#1
		if (opcode == 7'b0110111)
			#1 $display("pc: 0x%8h\tinstr: 0x%8h\tlui\tx%0d, 0x%0h",
							pc, decode_in, rd, imm_u); // LUI

		else if (opcode == 7'b0010111)
			#1 $display("pc: 0x%8h\tinstr: 0x%8h\tauipc\tx%0d, 0x%0h",
							pc, decode_in, rd, imm_u); // AUIPC

		else if (opcode == 7'b1101111) begin
			#1 $display("pc: 0x%8h\tinstr: 0x%8h\tjal\tx%0d, 0x%8h",
							pc, decode_in, rd, addr_j); // JAL
		end

		else if (opcode == 7'b1100111)
			#1 $display("pc: 0x%8h\tinstr: 0x%8h\tjalr\tx%0d, %0d(x%0d)",
							pc, decode_in, rd, imm_i, rs1); // JALR

		else if (opcode == 7'b1100011) begin
			if (funct3 == 3'b000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tbeq\tx%0d, x%0d, 0x%8h",
								pc, decode_in, rs1, rs2, addr_b); // BEQ
			else if (funct3 == 3'b001)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tbne\tx%0d, x%0d, 0x%8h",
								pc, decode_in, rs1, rs2, addr_b); // BNE
			else if (funct3 == 3'b100)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tblt\tx%0d, x%0d, 0x%8h",
								pc, decode_in, rs1, rs2, addr_b); // BLT
			else if (funct3 == 3'b101)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tbge\tx%0d, x%0d, 0x%8h",
								pc, decode_in, rs1, rs2, addr_b); // BGE
			else if (funct3 == 3'b110)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tbltu\tx%0d, x%0d, 0x%8h",
								pc, decode_in, rs1, rs2, addr_b); // BLTU
			else if (funct3 == 3'b111)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tbgeu\tx%0d, x%0d, 0x%8h",
								pc, decode_in, rs1, rs2, addr_b); // BGEU
			else begin
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tnot instr",
								pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (opcode == 7'b0000011) begin
			if (funct3 == 3'b000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tlb\tx%0d, %0d(x%0d)",
								pc, decode_in, rd, imm_i, rs1); // LB
			else if (funct3 == 3'b001)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tlh\tx%0d, %0d(x%0d)",
								pc, decode_in, rd, imm_i, rs1); // LH
			else if (funct3 == 3'b010)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tlw\tx%0d, %0d(x%0d)",
								pc, decode_in, rd, imm_i, rs1); // LW
			else if (funct3 == 3'b100)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tlbu\tx%0d, %0d(x%0d)",
								pc, decode_in, rd, imm_i, rs1); // LBU
			else if (funct3 == 3'b101)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tlhu\tx%0d, %0d(x%0d)",
								pc, decode_in, rd, imm_i, rs1); // LHU
			else begin
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tnot instr",
								pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (opcode == 7'b0100011) begin
			if (funct3 == 3'b000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsb\tx%0d, %0d(x%0d)",
								pc, decode_in, rs2, imm_s, rs1); // SB
			else if (funct3 == 3'b001)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsh\tx%0d, %0d(x%0d)",
								pc, decode_in, rs2, imm_s, rs1); // SH
			else if (funct3 == 3'b010)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsw\tx%0d, %0d(x%0d)",
								pc, decode_in, rs2, imm_s, rs1); // SW
			else begin
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tnot instr",
								pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (opcode == 7'b0010011) begin
			if (funct3 == 3'b000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\taddi\tx%0d, x%0d, %0d",
								pc, decode_in, rd, rs1, imm_i); // ADDI
			else if (funct3 == 3'b010)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tslti\tx%0d, x%0d, %0d",
								pc, decode_in, rd, rs1, imm_i); // SLTI
			else if (funct3 == 3'b011)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsltiu\tx%0d, x%0d, %0d",
								pc, decode_in, rd, rs1, imm_i); // SLTIU
			else if (funct3 == 3'b100)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\txori\tx%0d, x%0d, %0d",
								pc, decode_in, rd, rs1, imm_i); // XORI
			else if (funct3 == 3'b110)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tori\tx%0d, x%0d, %0d",
								pc, decode_in, rd, rs1, imm_i); // ORI
			else if (funct3 == 3'b111)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tandi\tx%0d, x%0d, %0d",
								pc, decode_in, rd, rs1, imm_i); // ANDI
			else if (funct3 == 3'b001 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tslli\tx%0d, x%0d, 0x%0h",
								pc, decode_in, rd, rs1, shamt); // SLLI
			else if (funct3 == 3'b101 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsrli\tx%0d, x%0d, 0x%0h",
								pc, decode_in, rd, rs1, shamt); // SRLI
			else if (funct3 == 3'b101 && funct7 == 7'b0100000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsrai\tx%0d, x%0d, 0x%0h",
								pc, decode_in, rd, rs1, shamt); // SRAI
			else begin
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tnot instr",
								pc, decode_in); // NOT INSTRUCTION
			end
		end
		
		else if (opcode == 7'b0110011) begin
			if (funct3 == 3'b000 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tadd\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // ADD
			else if (funct3 == 3'b000 && funct7 == 7'b0100000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsub\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // SUB
			else if (funct3 == 3'b001 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsll\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // SLL
			else if (funct3 == 3'b010 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tslt\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // SLT
			else if (funct3 == 3'b011 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsltu\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // SLTU
			else if (funct3 == 3'b100 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\txor\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // XOR
			else if (funct3 == 3'b101 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsrl\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // SRL
			else if (funct3 == 3'b101 && funct7 == 7'b0100000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tsra\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // SRA
			else if (funct3 == 3'b110 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tor\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // OR
			else if (funct3 == 3'b111 && funct7 == 7'b0000000)
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tand\tx%0d, x%0d, x%0d",
								pc, decode_in, rd, rs1, rs2); // AND
			else begin
				#1 $display("pc: 0x%8h\tinstr: 0x%8h\tnot instr",
								pc, decode_in); // NOT INSTRUCTION
			end
		end

		else if (decode_in == 32'b000000000000_00000_000_00000_1110011) begin
			#1 $display("pc: 0x%8h\tinstr: 0x%8h\tecall",
							pc, decode_in,); // ECALL
		end

		else begin
			// not instruction
			#1 $display("pc: 0x%8h\tinstr: 0x%8h\tnot instr",
							pc, decode_in); // NOT INSTRUCTION
		end
	end
endmodule // decode

/*========================================================================================*/

// testbench
module dut;
	reg clock = 0;
	reg [31:0] pc;
	wire [31:0] pc_next;
	wire [31:0] fet_out_dec_in;

    initial begin
        $dumpvars(0, fet);
        $dumpvars(0, dec);

		#(`HALF_CYCLE / 2) 
		pc = `STARTING_ADDR;

		#(`FILE_LINES * 4 * `HALF_CYCLE) $finish;
    end
	
	fetch fet (
		.clock(clock),
		.fetch_en(1'h1),
		.pc_in(pc),
		.pc_out(pc_next),
		.fetch_out(fet_out_dec_in));
	
	decode dec (
		.clock(clock),
		.pc(pc),
		.decode_in(fet_out_dec_in));

	always @(posedge clock) begin
		pc = pc_next;
	end
	
	always begin                 
		#`HALF_CYCLE clock = ~clock;
	end
endmodule
