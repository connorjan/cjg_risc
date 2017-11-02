module cjg_clkgen (
    // system inputs
    input reset,                    // system reset
    input clk,                      // system clock

    // system outputs
    output clk_p1,                  // phase 0
    output clk_p2,                  // phase 1

    // dft
    input scan_in0,
    input scan_en,
    input test_mode,
    output scan_out0
);

// Clock counter
reg[1:0] clk_cnt;

// Signals for generating the clocks
wire pre_p1 = (~clk_cnt[1] & ~clk_cnt[0]);
wire pre_p2 = (clk_cnt[1] & ~clk_cnt[0]);

// Buffer output of phase 0 clock
CLKBUFX4 clk_p1_buf (
    .A(pre_p1),
    .Y(clk_p1)
);

// Buffer output of phase 1 clock
CLKBUFX4 clk_p2_buf (
    .A(pre_p2),
    .Y(clk_p2)
);

// Clock counter
always @ (posedge clk, negedge reset) begin
    if(~reset) begin
        clk_cnt <= 2'h0;
    end 
    else begin
        clk_cnt <= clk_cnt + 1'b1;
    end
end

endmodule // cjg_clkgen
