`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:34:49 11/16/2016 
// Design Name: 
// Module Name:    test_p 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module test_p;
reg clk,reset;
wire [31:0]t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7;
wire flush,MEM_MemWrite, MEM_MemRead, PC_Write, WB_RegWrite, WB_MemtoReg;
wire [1:0] forwardA, forwardB;
wire [4:0] rdreg1, rdreg2, WB_wraddr;
wire [31:0] Imem_out, PC_out, rddata1, rddata2, MEM_ALU_result,MEM_Dmem_wrdata, MEM_rddata, WB_res;
pipeline UUT (clk, reset, flush, t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7,
Imem_out, PC_out, PC_Write, rdreg1, rdreg2, rddata1, rddata2, MEM_MemWrite, MEM_MemRead, MEM_Dmem_wrdata,MEM_rddata,WB_wraddr, MEM_ALU_result, WB_res,forwardA, forwardB, WB_RegWrite, WB_MemtoReg);

initial begin
$monitor("clk=%b flush=%b PC_Write=%b Imem_out=%b PC_out=%h rdreg1=%b rdreg2=%b rddata1=%d rddata2=%d  forwardA=%b forwardB=%b  MemRead=%b  MEM_ALU_result=%d Dmem_wrdata=%d MEM_rddata=%d WB_wraddr=%b WB_res=%d WB_RegWrite=%b WB_MemtoReg=%b\n[$s0] = %h  [$s1] = %h  [$s2] = %h [$s3] = %h  [$s4] = %h  [$t0] = %h  [$t1] = %h", clk, flush, PC_Write, Imem_out, PC_out,rdreg1, rdreg2,rddata1, rddata2,forwardA, forwardB, MEM_MemRead, MEM_ALU_result, MEM_Dmem_wrdata, MEM_rddata, WB_wraddr,  WB_res, WB_RegWrite, WB_MemtoReg,s0,s1,s2,s3,s4,t0,t1);
end

initial begin
#0 clk=0;
#50 reset=1;
#50 reset=0;
end
always #50 clk=~clk;
initial #4000 $stop;
endmodule
