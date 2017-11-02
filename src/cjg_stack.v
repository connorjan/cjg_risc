module cjg_stack #(parameter WIDTH = 32, DEPTH = 16) (
    
    input clk,
    input reset,
    input [WIDTH-1:0] d,
    input push,
    input pop,
    
    output [WIDTH-1:0] q,

    // dft
    input scan_in0,
    input scan_en,
    input test_mode,
    output scan_out0
);

reg [WIDTH-1:0] stack [DEPTH-1:0];
integer i;
assign q = stack[0];

always @(posedge clk or negedge reset) begin
    if (~reset) begin
        for (i=0; i < DEPTH; i=i+1) begin
            stack[i] <= {WIDTH{1'b0}};
        end
    end
    else begin
        if (push) begin
            stack[0] <= d;
            for (i=1; i < DEPTH; i=i+1) begin
                stack[i] <= stack[i-1];
            end
        end
        else if (pop) begin
            for (i=0; i < DEPTH-1; i=i+1) begin
                stack[i] <= stack[i+1];
            end
            stack[DEPTH-1] <= 0;
        end
        else begin
            for (i=0; i < DEPTH; i=i+1) begin
                stack[i] <= stack[i];
            end
        end
    end
end

endmodule // cjg_stack
