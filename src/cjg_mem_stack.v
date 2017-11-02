module cjg_mem_stack #(parameter WIDTH = 32, DEPTH = 32, ADDRW = 5) (
    
    input clk,
    input reset,
    input [WIDTH-1:0] d,
    input [ADDRW-1:0] addr,
    input push,
    input pop,
    
    output reg [WIDTH-1:0] q,

    // dft
    input scan_in0,
    input scan_en,
    input test_mode,
    output scan_out0
);

reg [WIDTH-1:0] stack [DEPTH-1:0];
integer i;


always @(posedge clk or negedge reset) begin
    if (~reset) begin
        q <= {WIDTH{1'b0}};
        for (i=0; i < DEPTH; i=i+1) begin
            stack[i] <= {WIDTH{1'b0}};
        end
    end
    else begin
        if (push) begin
            stack[addr] <= d;
        end
        else begin
            stack[addr] <= stack[addr];
        end

        q <= stack[addr];
    end
end

endmodule // cjg_mem_stack
