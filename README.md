# MIPS-processor-simulation

Design single cycle implementation of MIPS computer in Verilog. Design pipelined implementation of MIPS computer in Verilog.
Realize control and data hazard handling in the pipelined implementation of MIPS Computer.
Use Xilinx to synthesize and implement the design, and execute some simple instructions on FPGA board.

Central Processing Unit (CPU) plays the most important role in modern computer. It is used to process and excecute the instruction fromed by a series of compilers. Every CPU has its own corresponding instruction set. In this project, we are going to design a MIPS instruction processor which supports a subset of MIPS instruction set including:
1. The memory-reference instructions load and save (lw,sw)
2. The arithetic-logical instructions: add, addi, sub, andi and slt 
3. The jump instructions: beq, bne, j
