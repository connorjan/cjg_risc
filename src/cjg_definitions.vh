// Instruction word slices
`define OPCODE 31:27
`define REG_I 26:22
`define REG_J 21:17
`define REG_K 16:12
`define ALU_CONSTANT 16:1
`define ALU_CONSTANT_MSB 16
`define ALU_CONTROL 0
`define DT_CONTROL 16
`define DT_CONSTANT 15:0
`define DT_CONSTANT_MSB 15
`define JMP_CODE 21:18
`define JMP_ADDR 15:0
`define JMP_CONTROL 16
`define RS_CONTROL 0
`define RS_OPCODE 3:1
`define RS_CONSTANT 16:11

// Jump codes
`define JU  4'b0000
`define JC  4'b1000
`define JN  4'b0100
`define JV  4'b0010
`define JZ  4'b0001
`define JNC 4'b0111
`define JNN 4'b1011
`define JNV 4'b1101
`define JNZ 4'b1110
`define JGE 4'b0110
`define JL  4'b1001

// special register file registers
`define REG_SR 5'h0 // status register
`define REG_PC 5'h1 // program counter
`define REG_SP 5'h2 // stack pointer

// Status bit index in the status register / RF[0]
`define SR_C    5'd0
`define SR_N    5'd1
`define SR_V    5'd2
`define SR_Z    5'd3
`define SR_GE   5'd4
`define SR_L    5'd5

// MMIO
`define MMIO_START_ADDR 16'hFF00
`define MMIO_GPIO_OUT 16'hFFF0
`define MMIO_GPIO_IN 16'hFFF0
