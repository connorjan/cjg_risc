// Opcodes
`define LD_IC   5'h00   // Load
`define ST_IC   5'h01   // Store
`define CPY_IC  5'h02   // Copy
`define PUSH_IC 5'h03   // Push onto stack
`define POP_IC  5'h04   // Pop off of stack
`define JMP_IC  5'h05   // Jumps
`define CALL_IC 5'h06   // Call
`define RET_IC  5'h07   // Return and RETI
`define ADD_IC  5'h08   // Addition
`define SUB_IC  5'h09   // Subtract
`define CMP_IC  5'h0A   // Compare
`define NOT_IC  5'h0B   // Bitwise NOT
`define AND_IC  5'h0C   // Bitwise AND
`define BIC_IC  5'h0D   // Bit clear ~&=
`define OR_IC   5'h0E   // Bitwise OR
`define XOR_IC  5'h0F   // Bitwise XOR
`define RS_IC   5'h10   // Rotate/Shift

`define MUL_IC  5'h1A   // Signed multiplication
`define DIV_IC  5'h1B   // Unsigned division

`define INT_IC  5'h1F   // Interrupt

// ALU States
`define ADD_ALU 4'h0    // Signed Add
`define SUB_ALU 4'h1    // Signed Subtract
`define AND_ALU 4'h2    // Logical AND
`define BIC_ALU 4'h3    // Logical BIC
`define OR_ALU  4'h4    // Logical OR
`define NOT_ALU 4'h5    // Logical Invert
`define XOR_ALU 4'h6    // Logical XOR
`define NOP_ALU 4'h7    // No operation
`define MUL_ALU 4'h8    // Signed multiplication
`define DIV_ALU 4'h9    // Signed division

// Shifter states
`define SRL_SHIFT 3'h0  // shift right logical
`define SLL_SHIFT 3'h1  // shift left logical
`define SRA_SHIFT 3'h2  // shift right arithmetic
`define RTR_SHIFT 3'h4  // rotate right 
`define RTL_SHIFT 3'h5  // rotate left
`define RRC_SHIFT 3'h6  // rotate right through carry
`define RLC_SHIFT 3'h7  // rotate left through carry
