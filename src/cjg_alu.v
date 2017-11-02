// Dynamic width combinational logic ALU

`include "src/cjg_opcodes.vh"

module cjg_alu #(parameter WIDTH = 32) (
    // sys ports
    input reset,
    input clk,

    input [WIDTH-1:0] a,
    input [WIDTH-1:0] b,
    input [3:0] opcode,

    output [WIDTH-1:0] result,
    output c, n, v, z,

    // dft
    input scan_in0,
    input scan_en,
    input test_mode,
    output scan_out0
);

reg[WIDTH:0] internal_result;
wire overflow, underflow;

assign result = internal_result[WIDTH-1:0];
assign c = internal_result[WIDTH];
assign n = internal_result[WIDTH-1];
assign z = (internal_result == 0 ? 1'b1 : 1'b0);

assign overflow = (internal_result[WIDTH:WIDTH-1] == 2'b01 ? 1'b1 : 1'b0);
assign underflow = (internal_result[WIDTH:WIDTH-1] == 2'b10 ? 1'b1 : 1'b0);

assign v = overflow | underflow;

always @(*) begin
    internal_result = 0;

    case (opcode)
        
        `ADD_ALU: begin
            // signed addition
            internal_result = {a[WIDTH-1], a} + {b[WIDTH-1], b};
        end

        `SUB_ALU: begin
            // signed subtraction
            internal_result = ({a[WIDTH-1], a} + ~{b[WIDTH-1], b}) + 1'b1;
        end

        `AND_ALU: begin
            // logical AND
            internal_result = a & b;
        end

        `BIC_ALU: begin
            // logical bit clear
            internal_result = a & (~b);
        end

        `OR_ALU : begin
            // logical OR
            internal_result = a | b;
        end

        `NOT_ALU: begin
            // logical invert
            internal_result = ~a;
        end

        `XOR_ALU: begin
            // logical XOR
            internal_result = a ^ b;
        end

        `NOP_ALU: begin
            // no operation
            // sign extend a to prevent wrongful overflow flag by accident
            internal_result = {a[WIDTH-1], a};
        end

        `MUL_ALU: begin
            // signed multiplication
            internal_result = a * b;
        end

        `DIV_ALU: begin
            // unsigned division
            internal_result = a / b;
        end

        default: begin
            internal_result = internal_result;
        end // default

    endcase // opcode

end // always @(*)
    
endmodule // cjg_alu
