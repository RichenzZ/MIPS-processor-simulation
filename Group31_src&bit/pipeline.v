
module pipeline(clk, reset, flush, t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7,
Imem_out, PC_out, PC_Write, rdreg1, rdreg2, rddata1, rddata2, MEM_MemWrite, MEM_MemRead, MEM_Dmem_wrdata,MEM_rddata,WB_wraddr, MEM_ALU_result, WB_res,forwardA, forwardB, WB_RegWrite, WB_MemtoReg);
	input clk, reset;
	output [31:0]t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7;
	output flush;
	
	// IF stage
	wire [31:0] branch_or_PC4, PC_in, IF_PC_4;
	output [31:0] Imem_out, PC_out;
	wire IF_Flush, IF_ID_Write, PC_Src;
	output PC_Write;

	//ID stage
	output [31:0] rddata1, rddata2;
	output [4:0] rdreg1, rdreg2;
	wire [31:0] ID_instruction, ID_instrction_extended32, ID_jump_addr,ID_PC_4;
	wire [25:0] ID_instruction25_0;
	wire [27:0] ID_instruction_shifted28;
	wire [15:0] ID_instruction15_0;
	wire [4:0] ID_rs, ID_rt, ID_rd;
	wire [5:0] op;
	wire ID_Flush, control_mux;
	//******signals*************
	wire ID_jump, ID_MemRead, ID_RegDst, ID_Branch, ID_BranchNeq, ID_MemtoReg,  ID_MemWrite, ID_ALUSrc, ID_RegWrite;
	wire [2:0] ID_ALUOp;
	
	//EX stage
	//******signals*************
	wire EX_jump, EX_RegWrite, EX_MemtoReg, EX_Branch, EX_BranchNeq,EX_MemRead, EX_MemWrite, EX_RegDst, EX_ALUSrc;
	wire [2:0] EX_ALUOp;

	wire EX_Flush, EX_zero;
	assign flush=EX_Flush;
	wire [31:0] ALUin1, ALUin2;
	wire [31:0] EX_jump_addr, EX_PC_4, EX_instruction_extended32, EX_shifted_branch, EX_branch_addr, EX_ALU_result, EX_Dmem_wrdata;
	wire [31:0] EX_rddata1, EX_rddata2;
	wire [4:0] EX_rs, EX_rt, EX_rd, EX_wraddr;
	wire [5:0] func;
	wire [3:0] control;
	output [1:0] forwardA, forwardB;

	//MEM stage
	//******signals*************
	output MEM_MemWrite, MEM_MemRead; 
	wire MEM_jump, MEM_RegWrite, MEM_MemtoReg, MEM_Branch, MEM_BranchNeq;

	wire MEM_zero;
	assign PC_Src=(MEM_Branch && MEM_zero)|| (MEM_BranchNeq && !(MEM_zero));

	wire [4:0] MEM_wraddr;
	wire [31:0] MEM_jump_addr, MEM_branch_addr;
	output [31:0] MEM_ALU_result, MEM_rddata,MEM_Dmem_wrdata;

	//WB stage
	//******signals*************
	output WB_RegWrite, WB_MemtoReg;

	output [4:0] WB_wraddr;
	wire [31:0] WB_rddata;
	output [31:0] WB_res;
	wire [31:0] WB_ALU_result, WB_res;

	
	// IF stage
	mux_2to1_32 mux1(IF_PC_4, MEM_branch_addr, branch_or_PC4, PC_Src);
	mux_2to1_32 mux2(branch_or_PC4, MEM_jump_addr, PC_in, MEM_jump);
	ALU_add ALU_add(PC_out, 4, IF_PC_4);
	PC PC(clk, reset, PC_in, PC_out, PC_Write);
	instruction_memory instruction_memory(reset, PC_out, Imem_out);

	IF_ID_reg IF_ID_reg(clk, reset, IF_PC_4, ID_PC_4, Imem_out, ID_instruction, IF_ID_Write, IF_Flush);

	//ID stage
	assign op[5:0]=ID_instruction[31:26];
	assign ID_instruction25_0[25:0]=ID_instruction[25:0];
	assign ID_instruction15_0[15:0]=ID_instruction[15:0];
	assign ID_rs[4:0]=ID_instruction[25:21];
	assign ID_rt[4:0]=ID_instruction[20:16];
	assign ID_rd[4:0]=ID_instruction[15:11];
	assign rdreg1=ID_rs;
	assign rdreg2=ID_rt;
	regfile regfile(clk, reset, ID_rs,ID_rt,WB_wraddr,WB_res, WB_RegWrite, rddata1,rddata2,t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7);
	shift_left_2_jump shift_left_2_jump (ID_instruction25_0, ID_instruction_shifted28);
	sign_extend_16to32 sign_extend_16to32(ID_instruction15_0,ID_instrction_extended32);
	hazard_unit hazard_unit(ID_rs, ID_rt, EX_rt , EX_rd, MEM_wraddr , EX_MemRead, MEM_MemRead, EX_ALUSrc, EX_RegWrite, op, PC_Write, IF_ID_Write, control_mux);
	Control Control(op, control_mux, ID_jump, ID_RegDst, ID_Branch, ID_BranchNeq, ID_MemRead, ID_MemtoReg, ID_ALUOp ,ID_MemWrite, ID_ALUSrc, ID_RegWrite);
	assign ID_jump_addr={ID_PC_4[31:28], ID_instruction_shifted28[27:0]};

	ID_EX_reg ID_EX_reg(clk, reset, ID_Flush, ID_jump_addr, ID_PC_4, rddata1, rddata2, ID_instrction_extended32, ID_rs, ID_rt, ID_rd, EX_jump_addr, EX_PC_4, EX_rddata1, EX_rddata2, EX_instruction_extended32, EX_rs, EX_rt, EX_rd, ID_jump, ID_RegWrite, ID_MemtoReg, ID_Branch, ID_BranchNeq, ID_MemRead, ID_MemWrite, ID_RegDst, ID_ALUSrc, ID_ALUOp, EX_jump, EX_RegWrite, EX_MemtoReg, EX_Branch,EX_BranchNeq, EX_MemRead, EX_MemWrite, EX_RegDst, EX_ALUSrc, EX_ALUOp);


	//EX stage
	assign func[5:0]=EX_instruction_extended32[5:0];
	mux_3to1_32 mux3(EX_rddata1, WB_res, MEM_ALU_result, ALUin1, forwardA);
	mux_3to1_32 mux4(EX_rddata2, WB_res, MEM_ALU_result, EX_Dmem_wrdata, forwardB);
	mux_2to1_32 mux5(EX_Dmem_wrdata, EX_instruction_extended32, ALUin2, EX_ALUSrc);
	mux_2to1_5 mux6(EX_rt, EX_rd, EX_wraddr, EX_RegDst);
	shift_left_2_branch shift_left_2_branch(EX_instruction_extended32, EX_shifted_branch);
	ALU_add ALU_add2(EX_PC_4,EX_shifted_branch, EX_branch_addr);
	ALUcontrol ALUcontrol(func, EX_ALUOp, control);
	ALU ALU(ALUin1, ALUin2, control, EX_ALU_result, EX_zero);
	forward_unit forward_unit(EX_rs, EX_rt, MEM_wraddr, WB_wraddr, MEM_RegWrite, WB_RegWrite, forwardA, forwardB);

	EX_MEM_reg EX_MEM_reg(clk, reset, EX_Flush, EX_jump, EX_RegWrite, EX_MemtoReg, EX_MemWrite, EX_MemRead, EX_Branch, EX_BranchNeq,EX_jump_addr, EX_branch_addr, EX_zero, EX_ALU_result, EX_Dmem_wrdata, EX_wraddr, MEM_jump, MEM_RegWrite, MEM_MemtoReg, MEM_MemWrite, MEM_MemRead, MEM_Branch, MEM_BranchNeq, MEM_jump_addr, MEM_branch_addr, MEM_zero, MEM_ALU_result, MEM_Dmem_wrdata, MEM_wraddr);


	//MEM stage
	flush_unit flush_unit(PC_Src, MEM_jump, IF_Flush, ID_Flush, EX_Flush);
	data_memory data_memory(clk, reset, MEM_ALU_result, MEM_Dmem_wrdata, MEM_MemWrite, MEM_MemRead, MEM_rddata);

	MEM_WB_reg MEM_WB_reg(clk, reset, MEM_RegWrite, MEM_MemtoReg, WB_RegWrite, WB_MemtoReg, MEM_rddata, MEM_ALU_result, MEM_wraddr, WB_rddata, WB_ALU_result, WB_wraddr);
	
	//WB stage
	mux_2to1_32 mux7(WB_ALU_result, WB_rddata, WB_res, WB_MemtoReg);

endmodule

//IF/ID state register
//input: pc_4_in, instruction_in
//output: pc_4_out, instruction_out
//hazard signal: flush
// control signal: IF_ID_Write
module IF_ID_reg(clk, reset, pc_4_in, pc_4_out, instruction_in, instruction_out, IF_ID_Write, flush);
	input clk, reset;
	input [31:0] pc_4_in;
	output reg [31:0] pc_4_out;
	input [31:0] instruction_in;
	output reg [31:0] instruction_out;
	input IF_ID_Write, flush;
	always @(posedge clk or posedge reset or posedge flush)
	begin
		if(reset==1)
		begin
			pc_4_out= 0;
			instruction_out=0;
		end
		else if(flush==1)
		begin
			pc_4_out= 0;
			instruction_out=0;
		end
		else if (IF_ID_Write==1)
		begin
			pc_4_out=pc_4_in;
			instruction_out=instruction_in;
		end
	end
endmodule

//input: pc_4_in, rddata1_in, rddata2_in, extended_32_instrction_in, IF_ID_rs_in, IF_ID_rt_in, IF_ID_rd_in
//output: pc_4_out, rddata1_out, rddata2_out, dextended_32_instrction_out, IF_ID_rs_out, IF_ID_rt_out, IF_ID_rd_out
//hazard signal: flush
// WB signal: RegWrite, MemtoReg
// MEM signal: jump, Branch, MemRead, MemWrite
// EX signal: RegDst, ALUSrc, AlUOp

module ID_EX_reg(clk, reset, flush, jump_addr_in, pc_4_in, rddata1_in, rddata2_in, extended_32_instrction_in, IF_ID_rs_in, IF_ID_rt_in, IF_ID_rd_in, jump_addr_out, pc_4_out, rddata1_out, rddata2_out, extended_32_instrction_out, IF_ID_rs_out, IF_ID_rt_out, IF_ID_rd_out,jump_in, RegWrite_in, MemtoReg_in,Branch_in, BranchNeq_in, MemRead_in, MemWrite_in,RegDst_in, ALUSrc_in, ALUOp_in, jump_out,RegWrite_out, MemtoReg_out,Branch_out, BranchNeq_out, MemRead_out, MemWrite_out,RegDst_out, ALUSrc_out, ALUOp_out);
	input clk, reset,flush;
	input [31:0] pc_4_in, jump_addr_in, rddata1_in, rddata2_in, extended_32_instrction_in;
	output reg [31:0] pc_4_out, jump_addr_out, rddata1_out, rddata2_out, extended_32_instrction_out;
	input [4:0] IF_ID_rs_in, IF_ID_rt_in, IF_ID_rd_in;
	output reg [4:0] IF_ID_rs_out, IF_ID_rt_out, IF_ID_rd_out;
	//signals
	input jump_in, RegWrite_in, MemtoReg_in,Branch_in, BranchNeq_in, MemRead_in, MemWrite_in,RegDst_in, ALUSrc_in;
	output reg jump_out, RegWrite_out, MemtoReg_out,Branch_out, BranchNeq_out, MemRead_out, MemWrite_out,RegDst_out, ALUSrc_out;
	input [2:0] ALUOp_in;
	output reg [2:0] ALUOp_out;
	always @(posedge clk or posedge reset or posedge flush)
	begin
		if (reset==1)
		begin
			jump_out=0;
			RegWrite_out=0;
			MemtoReg_out=0;
			Branch_out=0;
			BranchNeq_out=0;
			MemRead_out=0;
			MemWrite_out=0;
			RegDst_out=0;
			ALUSrc_out=0;
			ALUOp_out=0;
			rddata1_out=0;
			rddata2_out=0;
			extended_32_instrction_out=0;
			IF_ID_rt_out=0;
			IF_ID_rs_out=0;
			IF_ID_rd_out=0;
			pc_4_out=0;
			jump_addr_out=0;
		end
		else if(flush==1)
		begin
			jump_out=0;
			RegWrite_out=0;
			MemtoReg_out=0;
			Branch_out=0;
			BranchNeq_out=0;
			MemRead_out=0;
			MemWrite_out=0;
			RegDst_out=0;
			ALUSrc_out=0;
			ALUOp_out=0;
			rddata1_out=0;
			rddata2_out=0;
			extended_32_instrction_out=0;
			IF_ID_rt_out=0;
			IF_ID_rs_out=0;
			IF_ID_rd_out=0;
			pc_4_out=0;
			jump_addr_out=0;
		end
		else
		begin
			rddata1_out=rddata1_in;
			rddata2_out=rddata2_in;
			extended_32_instrction_out=extended_32_instrction_in;
			IF_ID_rt_out=IF_ID_rt_in;
			IF_ID_rs_out=IF_ID_rs_in;
			IF_ID_rd_out=IF_ID_rd_in;
			pc_4_out=pc_4_in;
			jump_out=jump_in;
			RegWrite_out=RegWrite_in;
			MemtoReg_out=MemtoReg_in;
			Branch_out=Branch_in;
			BranchNeq_out=BranchNeq_in;
			MemRead_out=MemRead_in;
			MemWrite_out=MemWrite_in;
			RegDst_out= RegDst_in;
			ALUSrc_out= ALUSrc_in;
			ALUOp_out= ALUOp_in;
			jump_addr_out=jump_addr_in;
		end
	end
endmodule

module EX_MEM_reg(clk, reset, flush, jump_in, RegWrite_in, MemtoReg_in, MemWrite_in, MemRead_in, Branch_in, BranchNeq_in, jump_addr_in, add_result_in, zero_in, ALU_result_in, rddata2_in, wraddr_in, jump_out, RegWrite_out, MemtoReg_out, MemWrite_out, MemRead_out, Branch_out, BranchNeq_out, jump_addr_out, add_result_out, zero_out, ALU_result_out, rddata2_out, wraddr_out);
	input clk, reset;
	//signals
	input flush, jump_in, RegWrite_in, MemtoReg_in, MemWrite_in, MemRead_in, Branch_in, BranchNeq_in, zero_in;
	output reg jump_out, RegWrite_out, MemtoReg_out, MemWrite_out, MemRead_out, Branch_out, BranchNeq_out,zero_out;
	input [31:0] jump_addr_in, add_result_in, ALU_result_in, rddata2_in;
	output reg [31:0] jump_addr_out, add_result_out, ALU_result_out, rddata2_out;
	input [4:0] wraddr_in;
	output reg [4:0] wraddr_out;
	always @(posedge clk)//why?why?why?
	begin
		if (reset==1)
		begin
			jump_out=0;
			RegWrite_out=0;
			MemtoReg_out=0;
			Branch_out=0;
			BranchNeq_out=0;
			MemRead_out=0;
			MemWrite_out=0;
			zero_out=0;
			ALU_result_out=0;
			add_result_out=0;
			rddata2_out=0;
			wraddr_out=0;
			jump_addr_out=0;
		end
		else if (flush ==1)
		begin
			jump_out=0;
			RegWrite_out=0;
			MemtoReg_out=0;
			Branch_out=0;
			BranchNeq_out=0;
			MemRead_out=0;
			MemWrite_out=0;
			zero_out=0;
			ALU_result_out=0;
			add_result_out=0;
			rddata2_out=0;
			wraddr_out=0;
			jump_addr_out=0;
		end
		else
		begin
			jump_out=jump_in;
			RegWrite_out=RegWrite_in;
			MemtoReg_out=MemtoReg_in;
			Branch_out=Branch_in;
			BranchNeq_out=BranchNeq_in;
			MemRead_out=MemRead_in;
			MemWrite_out=MemWrite_in;
			zero_out=zero_in;
			ALU_result_out=ALU_result_in;
			add_result_out=add_result_in;
			rddata2_out=rddata2_in;
			wraddr_out=wraddr_in;
			jump_addr_out=jump_addr_in;
		end
	end
endmodule

//input: rddata_in, ALU_result_in, reg_dst_in
//output:rddata_out, ALU_result_out, reg_dst_out
// WB signal: RegWrite, MemtoReg

module MEM_WB_reg(clk, reset, RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out,rddata_in, ALU_result_in, reg_dst_in, rddata_out, ALU_result_out, reg_dst_out);
	input clk, reset;
	input RegWrite_in, MemtoReg_in;
	output reg RegWrite_out, MemtoReg_out; 
	input [31:0] rddata_in, ALU_result_in;
	output reg [31:0] rddata_out, ALU_result_out;
	input [4:0] reg_dst_in;
	output reg [4:0] reg_dst_out;
	always @(posedge clk or posedge reset)
	begin
		if (reset==1)
		begin
			RegWrite_out=0;
			MemtoReg_out=0;
			ALU_result_out=0;
			rddata_out=0;
			reg_dst_out=0;
		end
		else
		begin
			RegWrite_out=RegWrite_in;
			MemtoReg_out=MemtoReg_in;
			ALU_result_out=ALU_result_in;
			rddata_out=rddata_in;
			reg_dst_out=reg_dst_in;
		end
	end
endmodule

//add PCWrite to this module compared with single cycle version
module PC(clk, reset, in, out, PCWrite);
	input clk, reset, PCWrite;
	input [31:0] in;
	output reg [31:0] out;
	always@ (posedge clk or posedge reset)
	begin
		if(reset==1)
			out=0;
		else if(PCWrite==1)
			out=in;
	end
endmodule

module forward_unit(ID_EX_rs, ID_EX_rt, EX_MEM_rd, MEM_WB_rd, EX_MEM_RegWrite, MEM_WB_RegWrite,forwardA, forwardB);
	input [4:0] ID_EX_rs, ID_EX_rt, EX_MEM_rd, MEM_WB_rd;
	input EX_MEM_RegWrite, MEM_WB_RegWrite;
	output reg [1:0] forwardA, forwardB;
	initial
	begin
		forwardB=0;
		forwardA=0;
	end
	always @(ID_EX_rs or ID_EX_rt or EX_MEM_rd or MEM_WB_rd, EX_MEM_RegWrite or MEM_WB_RegWrite)
	begin
	//EX hazard
		if( (EX_MEM_RegWrite==1) && (EX_MEM_rd!=0)&&(EX_MEM_rd == ID_EX_rs))
		forwardA=2'b10;
	//MEM hazard
		else if( (MEM_WB_RegWrite==1) && (MEM_WB_rd!=0) && (MEM_WB_rd==ID_EX_rs) && !((EX_MEM_RegWrite==1) && (EX_MEM_rd !=0)&&(EX_MEM_rd==ID_EX_rs)) )
		forwardA=2'b01;
		else
		forwardA=2'b00;
	//EX hazard
		if( (EX_MEM_RegWrite==1) && (EX_MEM_rd!=0)&&(EX_MEM_rd == ID_EX_rt))
		forwardB=2'b10;
	//MEM hazard
		else if ((MEM_WB_RegWrite==1) && (MEM_WB_rd!=0) && (MEM_WB_rd==ID_EX_rt)&& !((EX_MEM_RegWrite==1) && (EX_MEM_rd !=0)&&(EX_MEM_rd==ID_EX_rt)) )
		forwardB=2'b01;
		else
		forwardB=2'b00;
	end
endmodule


module flush_unit(PCSrc, jump, IF_Flush, ID_Flush, EX_Flush);
	input PCSrc, jump;
	output reg IF_Flush, ID_Flush, EX_Flush;
	initial begin
		 IF_Flush=0;
		 ID_Flush=0;
		 EX_Flush=0;
	 end
	 always @(PCSrc or jump)
	 begin
	 	if( (PCSrc==1) || (jump==1))
	 	begin
	 		IF_Flush=1;
	 		ID_Flush=1;
	 		EX_Flush=1;
	 	end
	 	else 
	 	begin
	 		IF_Flush=0;
	 		ID_Flush=0;
	 		EX_Flush=0;
	 	end
	 end
endmodule

// handle data hazard for branches and load related 
// Stall
module hazard_unit(IF_ID_rs, IF_ID_rt, ID_EX_rt ,ID_EX_rd, EX_MEM_rt , ID_EX_MemRead, EX_MEM_MemRead, ID_EX_ALUSrc, ID_EX_RegWrite, op, PCWrite, IF_ID_Write, control_mux);
	input [4:0] IF_ID_rs, IF_ID_rt, ID_EX_rt ,ID_EX_rd ,EX_MEM_rt;
	input ID_EX_MemRead, EX_MEM_MemRead, ID_EX_ALUSrc, ID_EX_RegWrite;
	input [5:0] op;
	output reg PCWrite, IF_ID_Write, control_mux; 
	//assume when control_mux=1, control signals can pass through
	initial begin
		control_mux=1;
		PCWrite=1;
		IF_ID_Write=1;
	end

	always @(IF_ID_rs or IF_ID_rt or ID_EX_rt or ID_EX_rd or EX_MEM_rt or ID_EX_MemRead or EX_MEM_MemRead or ID_EX_ALUSrc or ID_EX_RegWrite or op)
	begin
	control_mux=1;
	PCWrite=1;
	IF_ID_Write=1;
	// lw immediately before sw
	if ((op==6'h2b) && (ID_EX_MemRead==1)&& (IF_ID_rt==ID_EX_rt) )
	begin
		control_mux=0;
		PCWrite=0;
		IF_ID_Write=0;
	end
	// lw 2nd before sw
	else if ((op==6'h2b) && (EX_MEM_MemRead==1)&& (IF_ID_rt==EX_MEM_rt) )
	begin
		control_mux=0;
		PCWrite=0;
		IF_ID_Write=0;
	end
	// load use hazard
	else if((ID_EX_MemRead==1)&&( (ID_EX_rt==IF_ID_rs) ||  ID_EX_rt==IF_ID_rt) )
	begin
		control_mux=0;
		PCWrite=0;
		IF_ID_Write=0;
	end
	else 
	begin
		control_mux=1;
		PCWrite=1;
		IF_ID_Write=1;
	end
end
endmodule

// add control_mux input signal
module Control(op, control_mux, jump, RegDst, Branch, BranchNeq, MemRead, MemtoReg, ALUOp ,MemWrite, ALUSrc, RegWrite);
	input [5:0] op;
	input control_mux;
	output reg jump, RegDst, Branch, BranchNeq, MemRead, MemtoReg ,MemWrite, ALUSrc, RegWrite;
	output reg [2:0] ALUOp;
	//lw 23 10 0011
	//sw 2b 10 1011
	//addi 8 00 1000
	//add sub and or slt 0
	//andi c 00  1100
	//beq 4 00 0100
	//bne 5 00 0101
	//j 2 00 0010
	initial 
	begin
		jump=0; RegDst=0;ALUSrc=0;MemtoReg=0;RegWrite=0;MemRead=0;MemWrite=0;Branch=0;ALUOp=3'b000;Branch=0;BranchNeq=0;
	end
	always @(op or control_mux)
	begin
		case(op)
		6'b100011: begin 
		jump=0; RegDst=0;ALUSrc=1;MemtoReg=1;RegWrite=1;MemRead=1;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'b000;end //lw
		6'b101011: begin  jump=0;RegDst=0;ALUSrc=1;MemtoReg=0;RegWrite=0;MemRead=0;MemWrite=1;Branch=0;BranchNeq=0;ALUOp=3'b000;end //sw
		6'b000100: begin  jump=0;RegDst=0;ALUSrc=0;MemtoReg=0;RegWrite=0;MemRead=0;MemWrite=0;Branch=1;BranchNeq=0;ALUOp=3'b001;end //beq
		6'b000101: begin  jump=0;RegDst=0;ALUSrc=0;MemtoReg=0;RegWrite=0;MemRead=0;MemWrite=0;Branch=0;BranchNeq=1;ALUOp=3'b001;end //bne
		6'b001000: begin  jump=0;RegDst=0;ALUSrc=1;MemtoReg=0;RegWrite=1;MemRead=0;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'b011;end //addi
		6'b001100: begin  jump=0;RegDst=0;ALUSrc=1;MemtoReg=0;RegWrite=1;MemRead=0;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'b100;end //andi
		6'b000010: begin  jump=1;RegDst=0;ALUSrc=0;MemtoReg=0;RegWrite=0;MemRead=0;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'bxxx;end //j
		6'b000000: begin  jump=0; RegDst=1;ALUSrc=0;MemtoReg=0;RegWrite=1;MemRead=0;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'b010;end //add
		default: begin  jump=0; RegDst=1;ALUSrc=0;MemtoReg=0;RegWrite=1;MemRead=0;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'b010;end //R type
		endcase

		if(control_mux==0)
		begin  jump=0;RegDst=0;ALUSrc=0;MemtoReg=0;RegWrite=0;MemRead=0;MemWrite=0;Branch=0;BranchNeq=0;ALUOp=3'b000;end
	end
endmodule

	


//********************************************************//
// modules from single cycle CPU
//********************************************************//

// N-bit 2 to 1 mux
//N can be 5, 32
module mux_2to1_32 (in0, in1, out, s0);
	parameter N =32;
	input [N-1:0] in0, in1; 
	input s0;
	output [N-1:0] out;
	assign out=s0?in1:in0;
endmodule

module mux_3to1_32 (in0, in1, in2, out, s0);
	parameter N =32;
	input [N-1:0] in0, in1, in2; 
	input [1:0] s0;
	output reg [N-1:0] out;
	always @(in0 or in1 or in2 or s0)
	begin
		case(s0)
		2'b00: out=in0;
		2'b01: out=in1;
		2'b10: out=in2;
		default: out=in0;
		endcase
	end
endmodule

//mux before the register file
module mux_2to1_5 (in0, in1, out, s0);
	parameter N =5;
	input [N-1:0] in0, in1; 
	input s0;
	output [N-1:0] out;
	assign out=s0?in1:in0;
endmodule

module sign_extend_16to32(in, out);
	input [15:0] in;
	output [31:0] out;
	assign out[15:0]=in[15:0];
	assign out[31:16]=in[15]?16'b1111111111111111:16'b0;
endmodule

//shift left for branch
module shift_left_2_branch(in, out);
	input [31:0] in;
	output [31:0] out;
	assign out[31:0]={in[29:0],2'b00};
endmodule

//shift left for jump
module shift_left_2_jump (in, out);
	input [25:0] in;
	output [27:0] out;
	assign out[27:0]={in[25:0],2'b00};
endmodule

module ALU_add(in0, in1, out);
	input [31:0] in0, in1;
	output reg [31:0] out;
	always @(in0 or in1)
	begin
		out=in0+in1;
	end 
endmodule

module ALU(in0, in1, control, out, zero);
	input [31:0] in0, in1;
	input [3:0] control;
	output reg [31:0] out;
	output reg zero;
	always @(control or in0 or in1)
	begin
		case (control)
		4'b0010: begin zero=0;out=in0+in1;end //add ##addi
		4'b0110: 								//sub beq #+bne
			begin 
				out=in0-in1;
				if (in0==in1) zero=1;
				else zero=0;
			end
		4'b0000: begin zero=0;out=in0&in1;end //and,andi
		4'b0001: begin zero=0;out=in0|in1;end //or
		4'b0111:                                //slt
		begin
			zero=0;
			if(in0<in1) out=1;
			else out=0;
		end
		default: begin zero=0; out=0; end
		endcase
	end
endmodule

//register file
module regfile(clk, reset, rdreg1,rdreg2,wrreg,wrdata, regwrite, rddata1,rddata2,t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7);
	input clk,reset;
	input [4:0] rdreg1;
	input [4:0] rdreg2;
	input [4:0] wrreg;
	input [31:0] wrdata;
	input regwrite;
	output [31:0] rddata1;
	output [31:0] rddata2;
	output [31:0] t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7;
	reg [31:0] data [0:31];
	integer i;
	initial
	begin
		for(i=0; i<32; i=i+1)
		data[i]=0;
	end
	always @(posedge reset or negedge clk)
	begin
		if(reset==1)
		begin
		for(i=0; i<32; i=i+1)
		data[i]=0;
		end
		else 
		begin
		if(regwrite==1) data[wrreg]=wrdata;
		end
	end

	assign rddata1[31:0]=data[rdreg1];
	assign rddata2[31:0]=data[rdreg2];
	assign t0=data[8];
	assign t1=data[9];
	assign t2=data[10];
	assign t3=data[11];
	assign t4=data[12];
	assign t5=data[13];
	assign t6=data[14];
	assign t7=data[15];
	assign t8=data[24];
	assign t9=data[25];
	assign s0=data[16];
	assign s1=data[17];
	assign s2=data[18];
	assign s3=data[19];
	assign s4=data[20];
	assign s5=data[21];
	assign s6=data[22];
	assign s7=data[23];
endmodule

module data_memory(clk, reset, addr, wrdata, memwrite, memread, rddata);
	input [31:0] addr;
	input [31:0] wrdata;
	input clk, reset, memread, memwrite;
	output [31:0] rddata;
	wire [31:0] shift_addr;
	reg [31:0] mem [0:63];
	integer i;
	assign shift_addr={2'b00, addr[31:2]}; 
	initial begin
		for(i=0;i<64;i=i+1)
		begin
			mem[i]=32'b0;
		end
	end
	always @(negedge clk) begin
		if(memwrite==1) mem[shift_addr]=wrdata;
	end
		assign rddata=(memread)?mem[shift_addr]:32'bx;
endmodule


// instruction memory can hold 32 instructions
// test case only has 30 instructions
module instruction_memory(reset, addr, instruction);
	input reset;
	input [31:0] addr;
	output [31:0] instruction;
	reg [31:0] memory [0:31];
	wire [4:0] shift_addr;
	integer k;
	assign shift_addr[4:0] = addr[6:2]; //divided by 4
	assign instruction = memory[shift_addr];
	initial
	begin
	for(k=0;k<32;k=k+1)begin
		memory[k]=32'b0;
	end
	memory[0] = 32'b00100000000010000000000000100000; //addi $t0, $zero, 32
	memory[1] = 32'b00100000000010010000000000110111; //addi $t1, $zero, 55
	memory[2] = 32'b00000001000010011000000000100100; //and $s0, $t0, $t1
	memory[3] = 32'b00000001000010011000000000100101; //or $s0, $t0, $t1
	memory[4] = 32'b10101100000100000000000000000100; //sw $s0, 4($zero)
	memory[5] = 32'b10101100000010000000000000001000; //sw $t0, 8($zero)
	memory[6] = 32'b00000001000010011000100000100000; //add $s1, $t0, $t1
	memory[7] = 32'b00000001000010011001000000100010; //sub $s2, $t0, $t1
	memory[8] = 32'b00010010001100100000000000001001; //beq $s1, $s2, error0
	memory[9] = 32'b10001100000100010000000000000100; //lw $s1, 4($zero)
	memory[10]= 32'b00110010001100100000000001001000; //andi $s2, $s1, 48
	memory[11] =32'b00010010001100100000000000001001; //beq $s1, $s2, error1
	memory[12] =32'b10001100000100110000000000001000; //lw $s3, 8($zero)
	memory[13] =32'b00010010000100110000000000001010; //beq $s0, $s3, error2
	memory[14] =32'b00000010010100011010000000101010; //slt $s4, $s2, $s1 (Last)
	memory[15] =32'b00010010100000000000000000001111; //beq $s4, $0, EXIT
	memory[16] =32'b00000010001000001001000000100000; //add $s2, $s1, $0
	memory[17] =32'b00001000000000000000000000001110; //j Last
	memory[18] =32'b00100000000010000000000000000000; //addi $t0, $0, 0(error0)
	memory[19] =32'b00100000000010010000000000000000; //addi $t1, $0, 0
	memory[20] =32'b00001000000000000000000000011111; //j EXIT
	memory[21] =32'b00100000000010000000000000000001; //addi $t0, $0, 1(error1)
	memory[22] =32'b00100000000010010000000000000001; //addi $t1, $0, 1
	memory[23] =32'b00001000000000000000000000011111; //j EXIT
	memory[24] =32'b00100000000010000000000000000010; //addi $t0, $0, 2(error2)
	memory[25] =32'b00100000000010010000000000000010; //addi $t1, $0, 2
	memory[26] =32'b00001000000000000000000000011111; //j EXIT
	memory[27] =32'b00100000000010000000000000000011; //addi $t0, $0, 3(error3)
	memory[28] =32'b00100000000010010000000000000011; //addi $t1, $0, 3
	memory[29] =32'b00001000000000000000000000011111; //j EXIT
	end
endmodule

module ALUcontrol(func, op, control);
	input [5:0] func;
	input [2:0] op;/////use 3-bit op
	output reg [3:0] control;

	always @(func, op)
	begin
		case(op)
		3'b000: control= 4'b0010; //lw sw
		3'b001: control= 4'b0110; //beq ##bne
		3'b011: control= 4'b0010; //addi
		3'b100: control= 4'b0000; //ANDi
		3'b010:
		begin
			case(func)
			6'b100000: control=4'b0010; //add
			6'b100010: control=4'b0110; //sub
			6'b100100: control=4'b0000; //AND
			6'b100101: control=4'b0001; //OR
			6'b101010: control=4'b0111; //slt
			default:   control=4'b0010;
			endcase
		end
		default: control=4'b0010;
		endcase
	end

endmodule
