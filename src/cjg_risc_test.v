`include "src/cjg_opcodes.vh"

// must be in mif directory
`define MIF "myDouble"

//`define TEST_ALU

module test;

// tb stuff
integer i;

// system ports
reg  clk, reset;
wire clk_p1, clk_p2;

// dft ports
wire  scan_out0;
reg  scan_in0, scan_en, test_mode;

always begin
    #0.5 clk = ~clk; // 1000 MHz clk
end

// program memory
reg [7:0] pm [0:65535];     // program memory
reg [31:0] pm_out;         // program memory output data
wire [15:0] pm_address;     // program memory address

// data memory
reg [7:0] dm [0:65535];     // data memory
reg [31:0] dm_out;         // data memory output 
wire [31:0] dm_data;        // data memory input data
wire dm_wren;               // data memory write enable
wire [15:0] dm_address;     // data memory address

always @(posedge clk_p2) begin
    if (dm_wren == 1'b1) begin
        dm[dm_address+3] = dm_data[31:24];
        dm[dm_address+2] = dm_data[23:16];
        dm[dm_address+1] = dm_data[15:8];
        dm[dm_address]   = dm_data[7:0];
    end
    pm_out = {pm[pm_address+3], pm[pm_address+2], pm[pm_address+1], pm[pm_address]};
    dm_out = {dm[dm_address+3], dm[dm_address+2], dm[dm_address+1], dm[dm_address]};
end

// inputs
reg [31:0] gpio_in;         // button inputs
reg [3:0] ext_interrupt_bus;  //external interrupts

// outputs
wire [31:0] gpio_out;

`ifdef TEST_ALU

reg [31:0] alu_a, alu_b;
reg [3:0] alu_opcode;
wire [31:0] alu_result;
wire alu_c, alu_n, alu_v, alu_z;

reg [31:0] tb_alu_result;

cjg_alu alu(
    .a(alu_a),
    .b(alu_b),
    .opcode(alu_opcode),

    .result(alu_result),
    .c(alu_c),
    .n(alu_n),
    .v(alu_v),
    .z(alu_z)
);
`endif

cjg_risc top(
    // system inputs
    .reset(reset),
    .clk(clk),
    .gpio_in(gpio_in),
    .ext_interrupt_bus(ext_interrupt_bus),

    // generated clock phases
    .clk_p1(clk_p1),
    .clk_p2(clk_p2),

    // system outputs
    .gpio_out(gpio_out),

    // program memory
    .pm_out(pm_out),
    .pm_address(pm_address),

    // data memory
    .dm_data(dm_data),
    .dm_out(dm_out),
    .dm_wren(dm_wren),
    .dm_address(dm_address),

    // dft
    .scan_in0(scan_in0),
    .scan_en(scan_en),
    .test_mode(test_mode),
    .scan_out0(scan_out0)
);

initial begin
    $timeformat(-9,2,"ns", 16);
`ifdef SDFSCAN
    $sdf_annotate("sdf/cjg_risc_tsmc065_scan.sdf", test.top);
`endif

`ifdef TEST_ALU
    // ALU TEST
    alu_a = 32'hfffffffc;
    alu_b = 32'hfffffffe;
    alu_opcode = `ADD_ALU;

    #10 tb_alu_result = alu_result;
    
    $display("alu_result = %x", tb_alu_result);
    $display("internal_result = %x", alu.internal_result);
    $display("alu_c = %x", alu_c);
    $display("alu_n = %x", alu_n);
    $display("alu_v = %x", alu_v);
    $display("alu_z = %x", alu_z);

    $finish;
`endif

// RISC TEST

    // init memories
    $readmemh({"mif/", `MIF, ".mif"}, pm);
    $readmemh({"mif/", `MIF, "_dm", ".mif"}, dm);
    $display("Loaded %s", {"mif/", `MIF, ".mif"});
    // reset for some cycles
    assert_reset;
    repeat (3) begin
        @(posedge clk);
    end

    // come out of reset a little before the edge
    #0.25 deassert_reset;
    @(posedge clk_p1);

    gpio_in = 12;

    // run until program reaches end of memory
    while (!(^pm_out === 1'bX) && (pm_out != 32'hFFFFFFFF) && (gpio_out != 32'hDEADBEEF)) begin
       @(posedge clk_p1);
    end

    $display("Trying to read from unknown program memory");

    // run for a few more clock cycles to empty the pipeline
    repeat (6) begin
        @(posedge clk);
    end

    $display("gpio_out = %x", gpio_out);
`ifndef SDFSCAN
    print_reg_file;
`endif
    //print_stack;
    $display("DONE");
    $stop;
end // initial

`ifndef SDFSCAN
task print_reg_file; begin
    $display("Register Contents:");
    for (i=0; i<32; i=i+1) begin
        $display("R%0d = 0x%X", i, top.reg_file[i]);
    end
    $display({30{"-"}});
end
endtask // print_reg_file

task print_stack; begin
    $display("Stack Contents:");
    for (i=0; i<32; i=i+1) begin
        $display("S%0d = 0x%X", i, top.data_stack.stack[i]);
    end
    $display({30{"-"}});
end
endtask // print_stack
`endif

task assert_reset; begin
    // reset dft ports
    scan_in0 = 1'b0;
    scan_en = 1'b0;
    test_mode = 1'b0;

    // reset system inputs
    clk = 1'b0;
    reset = 1'b0;
    gpio_in = 32'b0;
    ext_interrupt_bus = 4'b0;
end
endtask // assert_reset

task deassert_reset; begin
    reset = 1'b1;
end
endtask // deassert_reset

endmodule // test
