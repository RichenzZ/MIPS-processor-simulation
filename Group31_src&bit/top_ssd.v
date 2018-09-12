`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:32:05 11/20/2016 
// Design Name: 
// Module Name:    test_p_ssd 
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
module test_p_ssd(clk, clk_fast, reset, r, pc_reg, an, ca);
input clk,clk_fast, reset;// Interface: clk and reset (clk_500 used in ssd)
wire clk_500;
wire [31:0]t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7;

wire MEM_MemWrite, MEM_MemRead, PC_Write, WB_RegWrite, WB_MemtoReg;
wire flush;
wire [1:0] forwardA, forwardB;
wire [4:0] rdreg1, rdreg2, WB_wraddr;
wire [31:0] Imem_out, PC_out, rddata1, rddata2, ALU_result,MEM_rddata, MEM_Dmem_wrdata,WB_res;

input pc_reg;//used to check whether to show pc or register
output [3:0] an;
output [6:0] ca;
input [4:0] r;//which reg to show on ssd
wire  [3:0] hun;
wire  [3:0] ten;
wire  [3:0] one;
wire [6:0] sign;
wire  [6:0] hun_ca;
wire  [6:0] ten_ca;
wire  [6:0] one_ca; 

reg [31:0] display_reg;
wire [31:0] value;
assign value=display_reg;

pipeline mypipeline (clk, reset,flush, t0,t1,t2,t3,t4,t5,t6,t7,t8,t9,s0,s1,s2,s3,s4,s5,s6,s7, Imem_out, PC_out, PC_Write, rdreg1, rdreg2,rddata1, rddata2, MEM_MemWrite, MEM_MemRead,MEM_Dmem_wrdata, MEM_rddata, WB_wraddr,ALU_result, WB_res, forwardA, forwardB, WB_RegWrite, WB_MemtoReg);

  always @(posedge clk_500)
    begin
      case(r)
      5'd8: display_reg=t0;
      5'd9: display_reg=t1;
      5'd10: display_reg=t2;
      5'd11: display_reg=t3;
      5'd12: display_reg=t4;
      5'd13: display_reg=t5;
      5'd14: display_reg=t6;
      5'd15: display_reg=t7;
      5'd24: display_reg=t8;
      5'd25: display_reg=t9;
      5'd16: display_reg=s0;
      5'd17: display_reg=s1;
      5'd18: display_reg=s2;
      5'd19: display_reg=s3;
      5'd20: display_reg=s4;
      5'd21: display_reg=s5;
      5'd22: display_reg=s6;
      5'd23: display_reg=s7;
      default: display_reg=0;
      endcase
    end

displayNum displayNum(clk_500, pc_reg, PC_out, value, tho, hun, ten , one, sign);
divide_by_100k clk_divider(clk_fast, reset, clk_500);
Ring_4_counter rc(clk_500, reset, an);
ssd_driver hun_ssd(hun, hun_ca);
ssd_driver ten_ssd(ten, ten_ca);
ssd_driver one_ssd(one, one_ca);
choose_chathode choose(sign, hun_ca, ten_ca, one_ca, an ,ca);

endmodule

module displayNum(clk_500, pc_reg, PC_out, value, tho, hun, ten , one, sign);
input clk_500, pc_reg;
input [31:0] PC_out, value;
output reg [6:0] sign;
output reg [3:0] tho, hun , ten, one;
reg [31:0] temp;
initial begin
  sign <= 7'b1000000;
end

always @(posedge clk_500)
begin
temp=value;
  if(pc_reg)
  begin
    tho=PC_out[15:12];
    hun=PC_out[11:8];
    ten=PC_out[7:4];
    one=PC_out[3:0];
  end
  else 
  begin
    if(temp[31]==1)
    begin
      temp=~temp;
      temp=temp+1;
      sign=7'b0111111;
    end
    else 
    begin
      sign=7'b1000000;
    end
    tho=temp[15:12];
    hun=temp[11:8];
    ten=temp[7:4];
    one=temp[3:0];
  end
end
endmodule

module Dff_asy (q, d, clk, rst);
  input d, clk, rst;
  output reg q;
  
  always @ (posedge clk or posedge rst)
    if (rst == 1) q <= 0;
    else q <= d;
endmodule

module divide_by_100k (clock, reset, clock_out);
  parameter N = 17;
  input clock, reset;
  wire  load, asyclock_out;
  wire  [N-1:0] Dat;
  output  clock_out;
  reg   [N-1:0] Q;
  assign  Dat = 0;
  assign  load = Q[16] & Q[15] & Q[10] & Q[9] & Q[7] & Q[4] & Q[3] & Q[2] & Q[1] & Q[0];
  initial begin
  Q=0;
  end
  always @ (posedge reset or posedge clock)
  begin
    if (reset == 1'b1) Q <= 0;
    else if (load == 1'b1) Q <= Dat;
    else Q <= Q + 1;
  end
  assign  asyclock_out = load;
  Dff_asy Unit_Dff (clock_out, asyclock_out, clock, reset);
endmodule



module Ring_4_counter(clock, reset, Q);
  input     clock, reset;
  output reg  [3:0]Q;
  initial 
  begin
  Q=4'b1110;
  end
  always @(posedge clock or posedge reset)
  begin
    if (reset == 1) Q <= 4'b1110;
    else
    begin
      Q[3] <= Q[0];
      Q[2] <= Q[3];
      Q[1] <= Q[2];
      Q[0] <= Q[1];
    end
  end
endmodule

module ssd_driver(nIn, ssOut);
  output reg [6:0] ssOut;
  input [3:0] nIn;
  // ssOut format {g, f, e, d, c, b, a}

  always @(nIn)
  begin
    case (nIn)
    4'h0: ssOut = 7'b1000000;
    4'h1: ssOut = 7'b1111001;
    4'h2: ssOut = 7'b0100100;
    4'h3: ssOut = 7'b0110000;
    4'h4: ssOut = 7'b0011001;
    4'h5: ssOut = 7'b0010010;
    4'h6: ssOut = 7'b0000010;
    4'h7: ssOut = 7'b1111000;
    4'h8: ssOut = 7'b0000000;
    4'h9: ssOut = 7'b0011000;
    4'hA: ssOut = 7'b0001000;
    4'hB: ssOut = 7'b0000011;
    4'hC: ssOut = 7'b1000110;
    4'hD: ssOut = 7'b0100001;
    4'hE: ssOut = 7'b0000110;
    4'hF: ssOut = 7'b0001110;
    endcase
end
endmodule

module choose_chathode(tho, hun, ten, one, an, ca);
  input [6:0]tho;
  input [6:0]hun;
  input [6:0]ten;
  input [6:0]one;
  input [3:0]an;
  output  reg [6:0]ca;
 always @(an or tho or hun or ten or one)
 begin
  case(an)
  4'b1110: ca=one;
  4'b1101: ca=ten;
  4'b1011: ca=hun;
  4'b0111: ca=tho;
  default: ca=7'b1111111;
  endcase
 end
endmodule