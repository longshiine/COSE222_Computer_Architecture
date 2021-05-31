//
//  Author: Prof. Taeweon Suh
//          Computer Science & Engineering
//          Korea University
//  Date: July 14, 2020
//  Description: Skeleton design of RV32I Single-cycle CPU
//

// 1. xori: controller=> aludec, datapath=> inside alu module, part of assign result change
// 2. bgeu: pc logic update=> add f3bgeu, bgeu_taken(if cflag set): pc <= branch_dest
// 3. jalr: controller=> maindec jalr=1, aludec=>OP_I_JALR --> 5'b00000(addition), pc <= aluout(rs1_data + imm[11:0]), rd_data[31:0] = pc+4
// 4. slli:

`timescale 1ns/1ns
`define simdelay 1

module rv32i_cpu (
		      input         clk, reset,
            output [31:0] pc,		  		// program counter for instruction fetch
            input  [31:0] inst, 			// incoming instruction
            output        Memwrite, 	// 'memory write' control signal
            output [31:0] Memaddr,  	// memory address 
            output [31:0] MemWdata, 	// data to write to memory
            input  [31:0] MemRdata); 	// data read from memory

  wire        auipc, lui;
  wire        alusrc, regwrite;
  wire [4:0]  alucontrol;
  wire        memtoreg, memwrite;
  wire        branch, jal, jalr;
  
  // ###### Jangyoung Kim: Start ######   //
  reg [31:0]  IFID_inst;
  
  reg 		  IDEX_auipc, IDEX_lui;
  reg 		  IDEX_alusrc, IDEX_regwrite;
  reg [4:0]   IDEX_alucontrol;
  reg         IDEX_memtoreg, IDEX_memwrite;
  reg         IDEX_branch, IDEX_jal, IDEX_jalr;
  
  reg         EXMEM_memwrite;
  wire        stop;
  wire        flush;
  
  always @(posedge clk)
  begin
	  // Stalling when stop == 1, flushing when flush == 1
	  if (stop | flush)
	  begin
		  if (flush) IFID_inst <= #`simdelay 32'b0;
		  else IFID_inst <= #`simdelay IFID_inst;
		  
		  IDEX_auipc <= #`simdelay 0;
		  IDEX_lui <= #`simdelay 0;
		  IDEX_alusrc <= #`simdelay 0;
		  IDEX_regwrite <= #`simdelay 0;
		  IDEX_alucontrol <= #`simdelay 0;
		  IDEX_memtoreg <= #`simdelay 0;
		  IDEX_memwrite <= #`simdelay 0;
		  IDEX_branch <= #`simdelay 0;
		  IDEX_jal <= #`simdelay 0;
		  IDEX_jalr <= #`simdelay 0;
	  end
	  else
	  begin
		  IFID_inst <= #`simdelay inst;
		  
		  IDEX_auipc <= #`simdelay auipc;
		  IDEX_lui <= #`simdelay lui;
		  IDEX_alusrc <= #`simdelay alusrc;
		  IDEX_regwrite <= #`simdelay regwrite;
		  IDEX_alucontrol <= #`simdelay alucontrol;
		  IDEX_memtoreg <= #`simdelay memtoreg;
		  IDEX_memwrite <= #`simdelay memwrite;
		  IDEX_branch <= #`simdelay branch;
		  IDEX_jal <= #`simdelay jal;
		  IDEX_jalr <= #`simdelay jalr;
	  end
	  
	  EXMEM_memwrite <= #`simdelay IDEX_memwrite;
  end
  
  assign Memwrite = EXMEM_memwrite;
  // ###### Jangyoung Kim: End ###### 
  
  
  
  
  // Instantiate Controller
  controller i_controller(
		// ###### Jangyoung Kim: Start ###### 
      .opcode		(IFID_inst[6:0]), 
		.funct7		(IFID_inst[31:25]), 
		.funct3		(IFID_inst[14:12]), 
		// ###### Jangyoung Kim: End ###### 
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr),
		.alucontrol	(alucontrol));

  // Instantiate Datapath
  datapath i_datapath(
		.clk				(clk),
		.reset			(reset),
		// ###### Jangyoung Kim: Start ###### 
		.IDEX_auipc				(IDEX_auipc),
		.IDEX_lui				(IDEX_lui),
		.IDEX_memtoreg			(IDEX_memtoreg),
		.IDEX_memwrite			(IDEX_memwrite),
		.IDEX_branch			(IDEX_branch),
		.IDEX_alusrc			(IDEX_alusrc),
		.IDEX_regwrite			(IDEX_regwrite),
		.IDEX_jal				(IDEX_jal),
		.IDEX_jalr				(IDEX_jalr),
		.IDEX_alucontrol		(IDEX_alucontrol),
		.pc						(pc),
		.inst						(IFID_inst),
		.aluout					(Memaddr), 
		.MemWdata				(MemWdata),
		.MemRdata				(MemRdata),
		.stop  					(stop),
		.flush					(flush));
		// ###### Jangyoung Kim: End ###### 
		
		
endmodule


//
// Instruction Decoder 
// to generate control signals for datapath
//
module controller(input  [6:0] opcode,
                  input  [6:0] funct7,
                  input  [2:0] funct3,
                  output       auipc,
                  output       lui,
                  output       alusrc,
                  output [4:0] alucontrol,
                  output       branch,
                  output       jal,
                  output       jalr,
                  output       memtoreg,
                  output       memwrite,
                  output       regwrite);

	maindec i_maindec(
		.opcode		(opcode),
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr));
		
	aludec i_aludec( 
		.opcode     (opcode),
		.funct7     (funct7),
		.funct3     (funct3),
		.alucontrol (alucontrol));


endmodule


//
// RV32I Opcode map = Inst[6:0]
//
`define OP_R			7'b0110011
`define OP_I_Arith	7'b0010011
`define OP_I_Load  	7'b0000011
`define OP_I_JALR  	7'b1100111
`define OP_S			7'b0100011
`define OP_B			7'b1100011
`define OP_U_LUI		7'b0110111
`define OP_J_JAL		7'b1101111

//
// Main decoder generates all control signals except alucontrol 
//
//
module maindec(input  [6:0] opcode,
               output       auipc,
               output       lui,
               output       regwrite,
               output       alusrc,
               output       memtoreg, memwrite,
               output       branch, 
               output       jal,
               output       jalr);

  reg [8:0] controls;
  assign {auipc, lui, regwrite, alusrc, memtoreg, memwrite, branch, jal, jalr} = controls;
  
  always @(*)
  begin
    case(opcode)
      `OP_R: 			controls <= #`simdelay 9'b0010_0000_0; // R-type
      `OP_I_Arith: 	controls <= #`simdelay 9'b0011_0000_0; // I-type Arithmetic
      `OP_I_Load: 	controls <= #`simdelay 9'b0011_1000_0; // I-type Load
		`OP_I_JALR:    controls <= #`simdelay 9'b0011_0000_1; // JALR
      `OP_S: 			controls <= #`simdelay 9'b0001_0100_0; // S-type Store
      `OP_B: 			controls <= #`simdelay 9'b0000_0010_0; // B-type Branch
      `OP_U_LUI: 		controls <= #`simdelay 9'b0111_0000_0; // LUI
      `OP_J_JAL: 		controls <= #`simdelay 9'b0011_0001_0; // JAL
      default:    	controls <= #`simdelay 9'b0000_0000_0; // ???
    endcase
  end

endmodule

//
// ALU decoder generates ALU control signal (alucontrol)
//
module aludec(input      [6:0] opcode,
              input      [6:0] funct7,
              input      [2:0] funct3,
              output reg [4:0] alucontrol);

  always @(*)
    case(opcode)
      `OP_R:   		// R-type
		begin
			case({funct7,funct3})
			 10'b0000000_000: alucontrol <= #`simdelay 5'b00000; // addition (add)
			 10'b0100000_000: alucontrol <= #`simdelay 5'b10000; // subtraction (sub)
			 10'b0000000_111: alucontrol <= #`simdelay 5'b00001; // and (and)
			 10'b0000000_110: alucontrol <= #`simdelay 5'b00010; // or (or)
			 10'b0000000_100: alucontrol <= #`simdelay 5'b00011; // xor (xor)
			 10'b0000000_001: alucontrol <= #`simdelay 5'b00100; // sll (sll)
			 10'b0000000_101: alucontrol <= #`simdelay 5'b00101; // srl (srl)
			 // sra
			 // slt, sltu
          default:         alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end

      `OP_I_Arith:   // I-type Arithmetic
		begin
			case(funct3)
			 3'b000:  alucontrol <= #`simdelay 5'b00000; // addition (addi)
			 3'b111:  alucontrol <= #`simdelay 5'b00001; // and (andi)
			 3'b110:  alucontrol <= #`simdelay 5'b00010; // or (ori)
			 3'b100:  alucontrol <= #`simdelay 5'b00011; // xor (xori)
			 3'b001:  alucontrol <= #`simdelay 5'b00100; // sll (slli)
			 3'b101:  alucontrol <= #`simdelay 5'b00101; // srl (srli)
			 // sra
          default: alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end
		`OP_I_JALR:
			alucontrol <= #`simdelay 5'b00000;  // addition

      `OP_I_Load: 	// I-type Load (LW, LH, LB...)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_B:   		// B-type Branch (BEQ, BNE, ...)
      	alucontrol <= #`simdelay 5'b10000;  // subtraction 

      `OP_S:   		// S-type Store (SW, SH, SB)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_U_LUI: 		// U-type (LUI)
      	alucontrol <= #`simdelay 5'b00000;  // addition

      default: 
      	alucontrol <= #`simdelay 5'b00000;  // 

    endcase
    
endmodule


//
// CPU datapath
//
module datapath(input         clk, reset,
                input  [31:0] inst, 					// IFID_inst
                input         IDEX_auipc, 			// IDEX_auipc
                input         IDEX_lui,   			// IDEX_lui
                input         IDEX_regwrite,			// IDEX_regwrite
                input         IDEX_memtoreg,			// IDEX_memtoreg
                input         IDEX_memwrite,			// IDEX_memwrite
                input         IDEX_alusrc, 			// IDEX_alusrc
                input  [4:0]  IDEX_alucontrol,		// IDEX_alucontrol
                input         IDEX_branch,			// IDEX_branch
                input         IDEX_jal,				// IDEX_jal
                input         IDEX_jalr,				// IDEX_jalr
					 
                output reg [31:0] pc,
                output [31:0] aluout,
                output [31:0] MemWdata,
                input  [31:0] MemRdata,
					 
					 // ###### Jangyoung Kim: Start ######
					 output reg stop,
					 output reg flush
					 // ###### Jangyoung Kim: End ######
					 
					 );

  wire [4:0]  rs1, rs2, rd;
  wire [2:0]  funct3;
  wire [31:0] rs1_data, rs2_data;
  reg  [31:0] rd_data;
  wire [20:1] jal_imm;
  wire [12:1] br_imm;
  wire [31:0] se_br_imm, se_jal_imm, se_imm_itype, se_imm_stype, auipc_lui_imm;
  reg  [31:0] alusrc1, alusrc2;
  
  
  // ###### Jangyoung Kim: Start ######
  reg [31:0]  IFID_pc;
  
  reg [31:0]  IDEX_pc;
  reg [2:0]   IDEX_funct3;
  reg [4:0]   IDEX_rd;
  reg [4:0]   IDEX_rs1, IDEX_rs2;
  reg [31:0]  IDEX_rs1_data, IDEX_rs2_data;
  wire[31:0]  IDEX_aluout;
  reg [31:0]  IDEX_se_imm_itype, IDEX_se_imm_stype, IDEX_se_auipc_lui_imm, IDEX_se_br_imm, IDEX_se_jal_imm;
  wire [31:0] IDEX_branch_dest, IDEX_jal_dest, IDEX_jalr_dest;
  wire		  IDEX_Nflag, IDEX_Zflag, IDEX_Cflag, IDEX_Vflag;
  wire		  IDEX_f3beq, IDEX_f3bne, IDEX_f3blt, IDEX_f3bge, IDEX_f3bltu, IDEX_f3bgeu;
  wire		  IDEX_beq_taken, IDEX_bne_taken, IDEX_blt_taken, IDEX_bltu_taken, IDEX_bge_taken, IDEX_bgeu_taken;
  
  reg [31:0]  EXMEM_pc;
  reg [4:0]   EXMEM_rd, EXMEM_rs2;
  reg [31:0]  EXMEM_rs2_data;
  reg [31:0]  EXMEM_aluout;
  reg         EXMEM_memwrite, EXMEM_memtoreg, EXMEM_regwrite,EXMEM_jal, EXMEM_jalr;
  
  reg [31:0]  MEMWB_pc;
  reg [4:0]   MEMWB_rd;
  reg [31:0]  MEMWB_aluout;
  reg [31:0]  MEMWB_MemRdata;
  reg         MEMWB_memtoreg, MEMWB_regwrite, MEMWB_jalr, MEMWB_jal;

  always @(posedge clk)
  begin
	  
	  // IF/ID stage
	  if (stop)
			IFID_pc[31:0] <= #`simdelay IFID_pc;
	  else
			IFID_pc[31:0] <= #`simdelay pc;
	  // ID/EX stage
	  if (flush)
	  begin
	  	IDEX_pc[31:0] <= #`simdelay 32'b0;
		  IDEX_funct3[2:0] <= #`simdelay 3'b0;
		  IDEX_rd[4:0] <= #`simdelay 5'b0;
		  IDEX_rs1[4:0] <= #`simdelay 5'b0;
		  IDEX_rs2[4:0] <= #`simdelay 5'b0;
		  IDEX_rs1_data[31:0] <= #`simdelay 32'b0;
		  IDEX_rs2_data[31:0] <= #`simdelay 32'b0;
		  IDEX_se_imm_itype[31:0] <= #`simdelay 32'b0;
		  IDEX_se_imm_stype[31:0] <= #`simdelay 32'b0;
		  IDEX_se_auipc_lui_imm[31:0] <= #`simdelay 32'b0;
		  IDEX_se_br_imm[31:0] <= #`simdelay 32'b0;
		  IDEX_se_jal_imm[31:0] <= #`simdelay 32'b0;
	  end
	  else
	  begin
	  	  IDEX_pc[31:0] <= #`simdelay IFID_pc[31:0];
		  IDEX_rd[4:0] <= #`simdelay rd;
		  IDEX_funct3[2:0] <= #`simdelay funct3;
		  IDEX_rs1[4:0] <= #`simdelay rs1[4:0];
		  IDEX_rs2[4:0] <= #`simdelay rs2[4:0];
		  
		  // IDEX_rs1_data forwarding
		  begin
			  if ((MEMWB_rd == rs1) & MEMWB_regwrite & (MEMWB_rd !=0))
					 IDEX_rs1_data[31:0] <= #`simdelay rd_data[31:0];
			  else IDEX_rs1_data[31:0] <= #`simdelay rs1_data[31:0];
		  end
		  
		  // IDEX_rs2_data_forwarding
		  begin
			  if ((MEMWB_rd == rs2) & MEMWB_regwrite & (MEMWB_rd !=0))
					 IDEX_rs2_data[31:0] <= #`simdelay rd_data[31:0];
			  else IDEX_rs2_data[31:0] <= #`simdelay rs2_data[31:0]; 
		  end
		  
		  IDEX_se_imm_itype[31:0] <= #`simdelay se_imm_itype[31:0];
		  IDEX_se_imm_stype[31:0] <= #`simdelay se_imm_stype[31:0];
		  IDEX_se_auipc_lui_imm[31:0] <= #`simdelay auipc_lui_imm[31:0];
		  IDEX_se_br_imm[31:0] <= #`simdelay se_br_imm;
		  IDEX_se_jal_imm[31:0] <= #`simdelay se_jal_imm;
	  end
	  
	  // EX/MEM stage
	  EXMEM_pc[31:0] <= #`simdelay IDEX_pc[31:0];
	  EXMEM_rd[4:0] <= #`simdelay IDEX_rd[4:0];
	  EXMEM_rs2[4:0] <= #`simdelay IDEX_rs2[4:0];
	  EXMEM_aluout[31:0] <= #`simdelay IDEX_aluout[31:0];
	  
	  // (sw) EXMEM_rs2_data forwarding
	  begin
		  if ((EXMEM_rd == IDEX_rs2) & (IDEX_memwrite == 1) & (EXMEM_rd != 0)) EXMEM_rs2_data[31:0] <= #`simdelay EXMEM_aluout[31:0];
		  else if ((MEMWB_rd == IDEX_rs2) & (IDEX_memwrite == 1) & (MEMWB_rd != 0)) EXMEM_rs2_data[31:0] <= #`simdelay rd_data[31:0];
		  else EXMEM_rs2_data[31:0] <= #`simdelay IDEX_rs2_data[31:0];
	  end
	  
	  EXMEM_memwrite <= #`simdelay IDEX_memwrite;
	  EXMEM_memtoreg <= #`simdelay IDEX_memtoreg;
	  EXMEM_regwrite <= #`simdelay IDEX_regwrite;
	  EXMEM_jal <= #`simdelay IDEX_jal;
	  EXMEM_jalr <= #`simdelay IDEX_jalr;
	  
	  // MEM/WB stage
	  MEMWB_pc[31:0] <= #`simdelay EXMEM_pc[31:0];
	  MEMWB_rd[4:0] <= #`simdelay EXMEM_rd[4:0];
	  MEMWB_aluout[31:0] <= #`simdelay EXMEM_aluout[31:0];
	  MEMWB_MemRdata[31:0] <= #`simdelay MemRdata[31:0];
	  MEMWB_memtoreg <= #`simdelay EXMEM_memtoreg;
	  MEMWB_regwrite <= #`simdelay EXMEM_regwrite;
	  MEMWB_jalr <= #`simdelay EXMEM_jalr;
	  MEMWB_jal <= #`simdelay EXMEM_jal;
  end
  
  // assign rs1,rs2,rd, funct3 from IFID_inst
  assign rs1 = inst[19:15];
  assign rs2 = inst[24:20];
  assign rd  = inst[11:7];
  assign funct3  = inst[14:12];
  
  // JAL immediate
  assign jal_imm[20:1] = {inst[31],inst[19:12],inst[20],inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}},jal_imm[20:1],1'b0};

  // Branch immediate
  assign br_imm[12:1] = {inst[31],inst[7],inst[30:25],inst[11:8]};
  assign se_br_imm[31:0] = {{19{br_imm[12]}},br_imm[12:1],1'b0};

  // SE immediate
  assign se_imm_itype[31:0] = {{20{inst[31]}},inst[31:20]};
  assign se_imm_stype[31:0] = {{20{inst[31]}},inst[31:25],inst[11:7]};
  assign auipc_lui_imm[31:0] = {inst[31:12],12'b0};
  
  // i_datapath module output
  assign aluout = EXMEM_aluout;
  assign MemWdata = EXMEM_rs2_data;

  
  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
    .we			(MEMWB_regwrite),
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			(MEMWB_rd),
    .rd_data	(rd_data),
    .rs1_data	(rs1_data),
    .rs2_data	(rs2_data));

  //
  // ALU 
  //
  alu i_alu(
	 .a			(alusrc1),
	 .b			(alusrc2),
	 .alucont	(IDEX_alucontrol),
	 .result		(IDEX_aluout),
	 .N			(IDEX_Nflag),
	 .Z			(IDEX_Zflag),
	 .C			(IDEX_Cflag),
	 .V			(IDEX_Vflag));

  
  //
  // PC (Program Counter) logic 
  //
  always @(posedge clk, posedge reset)
  begin
	  if (reset)  pc <= 32'b0;
	  else
	  begin
			if (IDEX_beq_taken | IDEX_bne_taken | IDEX_blt_taken | IDEX_bge_taken | IDEX_bltu_taken | IDEX_bgeu_taken) // IDEX_branch_taken
				pc <= #`simdelay IDEX_branch_dest;
			else if (IDEX_jal) // IDEX_jal
				pc <= #`simdelay IDEX_jal_dest;
			else if (IDEX_jalr) // IDEX_jalr
				pc <= #`simdelay IDEX_jalr_dest;
			else if (stop)
				pc <= #`simdelay pc;
			else
				pc <= #`simdelay (pc + 4);
	  end
  end
  
 
  always@(*)
  begin
	  // 1st source to ALU (alusrc1)
	  begin
		  if ((EXMEM_rd == IDEX_rs1) & EXMEM_regwrite & (EXMEM_rd != 0))    alusrc1[31:0] = EXMEM_aluout[31:0]; // EX harzard forwarding unit
		  else if ((MEMWB_rd == IDEX_rs1) & MEMWB_regwrite & (MEMWB_rd != 0))    alusrc1[31:0] = rd_data[31:0]; //MEM harzard forwarding unit        
		  else if (IDEX_auipc)		alusrc1[31:0]  =  IDEX_pc;
		  else if (IDEX_lui) 		alusrc1[31:0]  =  32'b0;
		  else          				alusrc1[31:0]  =  IDEX_rs1_data[31:0];
	  end
  end
  always@(*)
  begin
	  // 2nd source to ALU (alusrc2)		
	  begin
		  if ((EXMEM_rd == IDEX_rs2) & EXMEM_regwrite & (EXMEM_rd != 0) & (IDEX_alusrc != 1))    alusrc2[31:0] = EXMEM_aluout[31:0]; //EX_harzard forwarding unit
		  else if ((MEMWB_rd == IDEX_rs2) & MEMWB_regwrite & (MEMWB_rd != 0) & (IDEX_alusrc != 1))    alusrc2[31:0] = rd_data[31:0]; //MEM_harzard forwarding unit
		  else if (IDEX_auipc | IDEX_lui)			alusrc2[31:0] = IDEX_se_auipc_lui_imm[31:0];
		  else if (IDEX_alusrc & IDEX_memwrite)	alusrc2[31:0] = IDEX_se_imm_stype[31:0];
		  else if (IDEX_alusrc)							alusrc2[31:0] = IDEX_se_imm_itype[31:0];
		  else												alusrc2[31:0] = IDEX_rs2_data[31:0];
	  end
  end
  always@(*)
  begin
	  // Data selection for writing to RF
	  begin
		  if	    (MEMWB_jal | MEMWB_jalr)			rd_data[31:0] = MEMWB_pc + 4;
		  else if (MEMWB_memtoreg)					rd_data[31:0] = MEMWB_MemRdata;
		  else												rd_data[31:0] = MEMWB_aluout;
	  end
  end
  always@(*)
  begin 
	  // Hazard Detection Unit
	  begin
		  if((IDEX_memtoreg == 1) & ((IDEX_rd == rs1) | (IDEX_rd == rs2))) 
			    stop = 1;
		  else stop = 0;
	  end
		// Flushing Logic
	  begin
		 if(IDEX_beq_taken | IDEX_bne_taken | IDEX_blt_taken | IDEX_bge_taken | IDEX_bltu_taken | IDEX_bgeu_taken | IDEX_jal | IDEX_jalr)
			   flush = 1;
       else flush = 0;
	  end
  end
  
  // Branch_taken calc in EX stage
  // branch_dest, jal_dest, jalr_dest calculate in EX stage
  assign IDEX_f3beq  = (IDEX_funct3 == 3'b000);
  assign IDEX_f3bne  = (IDEX_funct3 == 3'b001);
  assign IDEX_f3blt  = (IDEX_funct3 == 3'b100);
  assign IDEX_f3bge  = (IDEX_funct3 == 3'b101);
  assign IDEX_f3bltu  = (IDEX_funct3 == 3'b110);
  assign IDEX_f3bgeu  = (IDEX_funct3 == 3'b111);

  assign IDEX_beq_taken  =  IDEX_branch & IDEX_f3beq & IDEX_Zflag;
  assign IDEX_bne_taken  =  IDEX_branch & IDEX_f3bne & ~IDEX_Zflag;
  assign IDEX_blt_taken  =  IDEX_branch & IDEX_f3blt & (IDEX_Nflag != IDEX_Vflag);
  assign IDEX_bge_taken  =  IDEX_branch & IDEX_f3bge & (IDEX_Nflag == IDEX_Vflag);
  assign IDEX_bltu_taken =  IDEX_branch & IDEX_f3bltu & ~IDEX_Cflag;
  assign IDEX_bgeu_taken =  IDEX_branch & IDEX_f3bgeu & IDEX_Cflag;
 
  assign IDEX_branch_dest = (IDEX_pc + IDEX_se_br_imm);
  assign IDEX_jal_dest 	  = (IDEX_pc + IDEX_se_jal_imm);
  assign IDEX_jalr_dest   = (IDEX_aluout);
	
	  
  // ###### Jangyoung Kim: End ######
	
endmodule
