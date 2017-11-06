// Dynamic width combinational logic Shifter

`include "../cjg_risc/src/cjg_opcodes.vh"

// Whether or not to use the modifier shift logic
`define USE_MODIFIER

module cjg_shifter #(parameter WIDTH = 32, MOD_WIDTH = 6) (
    input reset,
    input clk,

    input signed [WIDTH-1:0] operand,
    input carry_in,
    input [2:0] opcode,
`ifdef USE_MODIFIER
    input [MOD_WIDTH-1:0] modifier,
`endif

    output reg [WIDTH-1:0] result,
    output reg carry_out,

    // dft
    input scan_in0,
    input scan_en,
    input test_mode,
    output scan_out0
);

`ifdef USE_MODIFIER
wire[WIDTH+WIDTH-1:0] temp_rotate_right = {operand, operand} >> modifier[MOD_WIDTH-2:0];
wire[WIDTH+WIDTH-1:0] temp_rotate_left = {operand, operand} << modifier[MOD_WIDTH-2:0];

wire[WIDTH+WIDTH+1:0] temp_rotate_right_c = {carry_in, operand, carry_in, operand} >> modifier;
wire[WIDTH+WIDTH+1:0] temp_rotate_left_c = {carry_in, operand, carry_in, operand} << modifier;
`endif

always @(*) begin

    case (opcode)
        
        `SRL_SHIFT: begin
`ifndef USE_MODIFIER
            // shift right logical by 1
            result <= {1'b0, operand[WIDTH-1:1]};
`else
            // shift right by modifier
            result <= operand >> modifier[MOD_WIDTH-2:0];
`endif
            carry_out <= carry_in;
        end

        `SLL_SHIFT: begin
`ifndef USE_MODIFIER
            // shift left logical by 1
            result <= {operand[WIDTH-2:0], 1'b0};
`else
            // shift left by modifier
            result <= operand << modifier[MOD_WIDTH-2:0];
`endif
            carry_out <= carry_in;
        end

        `SRA_SHIFT: begin
`ifndef USE_MODIFIER
            // shift right arithmetic by 1
            result <= {operand[WIDTH-1], operand[WIDTH-1:1]};
`else
            // shift right arithmetic by modifier
            result <= operand >>> modifier[MOD_WIDTH-2:0];
`endif
            carry_out <= carry_in;
        end

        `RTR_SHIFT: begin
`ifndef USE_MODIFIER
            // rotate right by 1
            result <= {operand[0], operand[WIDTH-1:1]};
`else
            // rotate right by modifier
            result <= temp_rotate_right[WIDTH-1:0];
`endif      
            carry_out <= carry_in;
        end

        `RTL_SHIFT: begin
`ifndef USE_MODIFIER
            // rotate left
            result <= {operand[WIDTH-2:0], operand[WIDTH-1]};
`else
            // rotate left by modifier
            result <= temp_rotate_left[WIDTH+WIDTH-1:WIDTH];
`endif
            carry_out <= carry_in;
        end

        `RRC_SHIFT: begin
`ifndef USE_MODIFIER
            // rotate right through carry
            result <= {carry_in, operand[WIDTH-1:1]};
            carry_out <= operand[0];
`else

            // rotate right through carry by modifier
            result <= temp_rotate_right_c[WIDTH-1:0];
            carry_out <= temp_rotate_right_c[WIDTH];
`endif
        end

        `RLC_SHIFT: begin
`ifndef USE_MODIFIER            
            // rotate left through carry
            result <= {operand[WIDTH-2:0], carry_in};
            carry_out <= operand[WIDTH-1];
`else
            // rotate left through carry by modifier
            result <= temp_rotate_left_c[WIDTH+WIDTH:WIDTH+1];
            carry_out <= temp_rotate_left_c[WIDTH];
`endif
        end

        default: begin
            result <= operand;
            carry_out <= carry_in;
        end // default

    endcase // opcode

end // always @(*)
    
endmodule // cjg_alu
