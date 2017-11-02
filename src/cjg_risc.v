/*
 * Title: cjg_risc
 * Author: Connor Goldberg
 * 
 */

`include "src/cjg_definitions.vh"
`include "src/cjg_opcodes.vh"

// Any instruction with a writeback operation
`define WB_INSTRUCTION(mc) (opcode[mc] == `LD_IC || opcode[mc] == `CPY_IC || opcode[mc] == `POP_IC || opcode[mc] == `ADD_IC || opcode[mc] == `SUB_IC || opcode[mc] == `CMP_IC || opcode[mc] == `NOT_IC || opcode[mc] == `AND_IC || opcode[mc] == `BIC_IC || opcode[mc] == `OR_IC || opcode[mc] == `XOR_IC || opcode[mc] == `RS_IC || opcode[mc] == `MUL_IC || opcode[mc] == `DIV_IC)

// ALU instructions
`define ALU_INSTRUCTION(mc) (opcode[mc] == `CPY_IC || opcode[mc] == `ADD_IC || opcode[mc] == `SUB_IC || opcode[mc] == `CMP_IC || opcode[mc] == `NOT_IC || opcode[mc] == `AND_IC || opcode[mc] == `BIC_IC || opcode[mc] == `OR_IC || opcode[mc] == `XOR_IC || opcode[mc] == `MUL_IC || opcode[mc] == `DIV_IC)

// Stack instructions
`define STACK_INSTRUCTION(mc) (opcode[mc] == `PUSH_IC) || (opcode[mc] == `POP_IC)

`define LOAD_MMIO(dest,bits,expr) \
if (dm_address < `MMIO_START_ADDR) begin \
  dest <= dm_out[bits] expr; \
end \
else begin \
  case (dm_address) \
    `MMIO_GPIO_IN: begin \
      dest <= gpio_in[bits] expr; \
    end \
    default: begin \
      dest <= temp_wb[bits] expr; \
    end \
  endcase \
end

module cjg_risc (
  // system inputs
  input reset,                    // system reset
  input clk,                      // system clock
  input [31:0] gpio_in,           // gpio inputs
  input [3:0] ext_interrupt_bus,  //external interrupts

  // system outputs
  output reg [31:0] gpio_out,     // gpio outputs

  // program memory
  input [31:0] pm_out,            // program memory output data
  output [15:0] pm_address,       // program memory address

  // data memory
  input [31:0] dm_out,            // data memory output 
  output reg [31:0] dm_data,      // data memory input data
  output reg dm_wren,             // data memory write enable
  output reg [15:0] dm_address,   // data memory address

  // generated clock phases
  output clk_p1,                   // clock phase 0
  output clk_p2,                   // clock phase 1

  // dft
  input scan_in0,
  input scan_en,
  input test_mode,
  output scan_out0
);

// integer for resetting arrays
integer i;

// register file
reg[31:0] reg_file[31:0];

// program counter regsiter (program memory address)
assign pm_address = reg_file[`REG_PC][15:0];

// temp address for jumps/calls
reg[15:0] temp_address;

// pipelined instruction registers
reg[31:0] instruction_word[3:1];

// address storage for each instruction
reg[13:0] instruction_addr[3:1];

// opcode slices
reg[4:0] opcode[3:0];

// TODO: is this even ok? 2d wires dont seem to work in simvision
always @(instruction_word[3] or instruction_word[2] or instruction_word[1] or pm_out) begin
  opcode[0] = pm_out[`OPCODE];
  opcode[1] = instruction_word[1][`OPCODE];
  opcode[2] = instruction_word[2][`OPCODE];
  opcode[3] = instruction_word[3][`OPCODE];
end

// stall signals
reg[3:0] stall_cycles;
reg stall[3:0];

// temp writeback register
reg[31:0] temp_wb; // general purpose
reg[31:0] temp_sp; // stack pointer

// data stack stuff
reg[31:0] data_stack_data;
reg[5:0] data_stack_addr;
reg data_stack_push;
reg data_stack_pop;
wire[31:0] data_stack_out;

// call stack stuff
reg[31:0] call_stack_data;
reg call_stack_push;
reg call_stack_pop;
wire[31:0] call_stack_out;

// ALU stuff
reg[31:0] alu_a, alu_b, temp_sr;
reg[3:0] alu_opcode;
wire[31:0] alu_result;
wire alu_c, alu_n, alu_v, alu_z; 

// Shifter stuff
reg[31:0] shifter_operand;
reg[5:0] shifter_modifier;
reg shifter_carry_in;
reg[2:0] shifter_opcode;
wire[31:0] shifter_result;
wire shifter_carry_out;

// Clock phase generator
cjg_clkgen clkgen(
  .reset(reset),
  .clk(clk),
  .clk_p1(clk_p1),
  .clk_p2(clk_p2),

  // dft
  .scan_in0(scan_in0),
  .scan_en(scan_en),
  .test_mode(test_mode),
  .scan_out0(scan_out0)
);

// Data Stack
cjg_mem_stack #(.DEPTH(64), .ADDRW(6)) data_stack (
  // inputs
  .clk(clk_p2),
  .reset(reset),
  .d(data_stack_data),
  .addr(data_stack_addr),
  .push(data_stack_push),
  .pop(data_stack_pop),

  // output
  .q(data_stack_out),

  // dft
  .scan_in0(scan_in0),
  .scan_en(scan_en),
  .test_mode(test_mode),
  .scan_out0(scan_out0)
);

// Call Stack
cjg_stack #(.DEPTH(64)) call_stack (
  // inputs
  .clk(clk_p2),
  .reset(reset),
  .d(call_stack_data),
  .push(call_stack_push),
  .pop(call_stack_pop),

  // output
  .q(call_stack_out),

  // dft
  .scan_in0(scan_in0),
  .scan_en(scan_en),
  .test_mode(test_mode),
  .scan_out0(scan_out0)
);

// ALU
cjg_alu alu (
  // dft
  .reset(reset),
  .clk(clk),
  .scan_in0(scan_in0),
  .scan_en(scan_en),
  .test_mode(test_mode),
  .scan_out0(scan_out0),

  // inputs
  .a(alu_a),
  .b(alu_b),
  .opcode(alu_opcode),

  // outputs
  .result(alu_result),
  .c(alu_c),
  .n(alu_n),
  .v(alu_v),
  .z(alu_z)
);

// Shifter and rotater
cjg_shifter shifter (
  // dft
  .reset(reset),
  .clk(clk),
  .scan_in0(scan_in0),
  .scan_en(scan_en),
  .test_mode(test_mode),
  .scan_out0(scan_out0),

  // inputs
  .operand(shifter_operand),
  .carry_in(shifter_carry_in),
  .modifier(shifter_modifier),
  .opcode(shifter_opcode),

  // outputs
  .result(shifter_result),
  .carry_out(shifter_carry_out)
);


// Here we go

always @(posedge clk_p1 or negedge reset) begin
  if (~reset) begin
    // reset
    reset_all;
  end // if (~reset)
  else begin
    // Main code

    // process stall signals
    stall[3] <= stall[2];
    stall[2] <= stall[1];
    stall[1] <= stall[0];

    if (stall_cycles != 0) begin
      stall[0] <= 1'b1;
      stall_cycles <= stall_cycles - 1'b1;
    end
    else begin
      stall[0] <= 1'b0;
    end

    // Machine cycle 3
    // writeback
    if (stall[3] == 1'b0) begin
      
      case (opcode[3])
        `ADD_IC, `SUB_IC, `NOT_IC, `AND_IC, `BIC_IC, `OR_IC, `XOR_IC, `CPY_IC, `LD_IC, `RS_IC, `MUL_IC, `DIV_IC: begin
          if (instruction_word[3][`REG_I] == `REG_PC) begin
            // Do not allow writing to the program counter
            reg_file[`REG_PC] <= reg_file[`REG_PC];
          end
          else begin
            reg_file[instruction_word[3][`REG_I]] <= temp_wb;
          end
        end

        `PUSH_IC: begin
          reg_file[`REG_SP] <= temp_sp; // incremented stack pointer
        end

        `POP_IC: begin
          reg_file[`REG_SP] <= temp_sp; // decremented stack pointer
          reg_file[instruction_word[3][`REG_I]] <= temp_wb;
          data_stack_pop <= 1'b0;
        end

        `ST_IC: begin
          dm_wren <= 1'b0;
        end

        `JMP_IC: begin
          // check the status register
          case (instruction_word[3][`JMP_CODE])
            
            `JU: begin
              reg_file[`REG_PC] <= {16'h0, temp_address};
            end

            `JC: begin
              if (reg_file[`REG_SR][`SR_C] == 1'b1) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JN: begin
              if (reg_file[`REG_SR][`SR_N] == 1'b1) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JV: begin
              if (reg_file[`REG_SR][`SR_V] == 1'b1) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JZ: begin
              if (reg_file[`REG_SR][`SR_Z] == 1'b1) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JNC: begin
              if (reg_file[`REG_SR][`SR_C] == 1'b0) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JNN: begin
              if (reg_file[`REG_SR][`SR_N] == 1'b0) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JNV: begin
              if (reg_file[`REG_SR][`SR_V] == 1'b0) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JNZ: begin
              if (reg_file[`REG_SR][`SR_Z] == 1'b0) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JGE: begin
              if (reg_file[`REG_SR][`SR_GE] == 1'b1) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            `JL: begin
              if (reg_file[`REG_SR][`SR_L] == 1'b1) begin
                reg_file[`REG_PC] <= {16'h0, temp_address};
              end
              else begin
                reg_file[`REG_PC] <= reg_file[`REG_PC];
              end
            end

            default: begin
              reg_file[`REG_PC] <= reg_file[`REG_PC];
            end
          endcase // instruction_word[3][`JMP_CODE]

        end // JMP_IC

        `CALL_IC: begin
          // jump to the routine address
          call_stack_push <= 1'b0;
          reg_file[`REG_PC] <= {16'h0, temp_address};
        end

        `RET_IC: begin
          // pop the program counter
          call_stack_pop <= 1'b0;
          reg_file[`REG_PC] <= {16'h0, temp_address};
        end

        default: begin
        end
      endcase // opcode[3]

      case (opcode[3])
        `ADD_IC, `SUB_IC, `CMP_IC, `NOT_IC, `AND_IC, `BIC_IC, `OR_IC, `XOR_IC, `RS_IC, `MUL_IC, `DIV_IC: begin
          // set the status register from the alu output
          reg_file[`REG_SR] <= temp_sr;
        end

        default: begin
          reg_file[`REG_SR] <= reg_file[`REG_SR];
        end
      endcase // opcode[3]

    end // if (stall[3] == 1'b0)

    // Machine cycle 2
    // execution
    if (stall[2] == 1'b0) begin

      case (opcode[2])
        `ADD_IC, `SUB_IC, `CMP_IC, `NOT_IC, `AND_IC, `BIC_IC, `OR_IC, `XOR_IC, `CPY_IC, `MUL_IC, `DIV_IC: begin
          // set temp ALU out            
          temp_wb <= alu_result;

          // Set status register
          if (instruction_word[3][`REG_I] == `REG_SR && `WB_INSTRUCTION(3)) begin
            // data forward from the status register
            temp_sr <= {temp_wb[31:6], alu_n, ~alu_n, alu_z, alu_v, alu_n, alu_c};
          end
          else begin
            // take the current status register
            temp_sr <= {reg_file[`REG_SR][31:6], alu_n, ~alu_n, alu_z, alu_v, alu_n, alu_c};
          end
          // TODO: data forward from other sources in mc3
        end

        `RS_IC: begin
          // grab the output from the shifter
          temp_wb <= shifter_result;

          // if rotating through carry, set the new carry value
          if ((instruction_word[2][`RS_OPCODE] == `RRC_SHIFT) || (instruction_word[2][`RS_OPCODE] == `RLC_SHIFT)) begin
            // Set status register
            if (instruction_word[3][`REG_I] == `REG_SR && `WB_INSTRUCTION(3)) begin
              // data forward from the status register
              temp_sr <= {temp_wb[31:1], shifter_carry_out};
            end
            else begin
              // take the current status register
              temp_sr <= {reg_file[`REG_SR][31:1], shifter_carry_out};
            end
          end
          else begin
            // dont change the status register
            temp_sr <= reg_file[`REG_SR];
          end
        end

        `PUSH_IC: begin
          temp_sp <= alu_result; // incremented Stack Pointer
          data_stack_push <= 1'b0;
        end

        `POP_IC: begin
          // data_stack_pop <= 1'b1;
          // data_stack_pop <= 1'b0;
          temp_sp <= alu_result; // decremented Stack Pointer
          temp_wb <= data_stack_out;
        end

        `LD_IC: begin
          `LOAD_MMIO(temp_wb,31:0,)
        end

        `ST_IC: begin
          if (dm_address < `MMIO_START_ADDR) begin
            // enable write if not mmio
            dm_wren <= 1'b1;
          end
          else begin
            // write to mmio
            dm_wren <= 1'b0;

            case (dm_address)

              `MMIO_GPIO_OUT: begin
                gpio_out <= dm_data;
              end

              default: begin
              end

            endcase // dm_address
          end
        end

        `JMP_IC: begin
          // Do nothing?
        end

        `CALL_IC: begin
          // push the status register onto the stack
          call_stack_push <= 1'b1;
          call_stack_data <= reg_file[`REG_SR];
        end

        `RET_IC: begin
          // pop the program counter
          call_stack_pop <= 1'b1;
          temp_address <= call_stack_out[15:0];
        end

        default: begin
        end
      endcase // opcode[2]

      instruction_word[3] <= instruction_word[2];
      instruction_addr[3] <= instruction_addr[2];
    end // if (stall[2] == 1'b0)

    // Machine cycle 1
    // operand fetch
    if (stall[1] == 1'b0) begin
    
      case (opcode[1])
        `ADD_IC, `SUB_IC, `CMP_IC, `NOT_IC, `AND_IC, `BIC_IC, `OR_IC, `XOR_IC, `MUL_IC, `DIV_IC: begin
          
          // set alu_a
          if ((instruction_word[1][`REG_J] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              // data forward from alu output
              alu_a <= alu_result;
            end
            else if (opcode[2] == `POP_IC) begin
              alu_a <= data_stack_out;
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(alu_a,31:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              alu_a <= shifter_result;
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              alu_a <= reg_file[instruction_word[1][`REG_J]];
            end
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
            // data forward from the increment/decrement of the stack pointer
            alu_a <= alu_result;
          end
          else if ((instruction_word[1][`REG_J] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            alu_a <= temp_wb;
            // TODO: data forward from other wb sources in mc3
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
            // data forward from the increment/decrement of the stack pointer
            alu_a <= temp_sp;
          end
          else begin
            // no data forwarding
            alu_a <= reg_file[instruction_word[1][`REG_J]];
          end

          // set alu_b
          if (instruction_word[1][`ALU_CONTROL] == 1'b1) begin
            // constant operand
            alu_b <= {{16{instruction_word[1][`ALU_CONSTANT_MSB]}}, instruction_word[1][`ALU_CONSTANT]}; // sign extend constant
          end
          else if ((instruction_word[1][`REG_K] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            //data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              alu_b <= alu_result;
            end
            else if (opcode[2] == `POP_IC) begin
              alu_b <= data_stack_out;
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(alu_b,31:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              alu_b <= shifter_result;
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              alu_b <= reg_file[instruction_word[1][`REG_K]];
            end
          end
          else if (instruction_word[1][`REG_K] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
            // data forward from the increment/decrement of the stack pointer
            alu_b <= alu_result;
          end
          else if ((instruction_word[1][`REG_K] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            alu_b <= temp_wb;
            // TODO: data forward from other wb sources in mc3
          end
          else if (instruction_word[1][`REG_K] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
            // data forward from the increment/decrement of the stack pointer
            alu_b <= temp_sp;
          end
          else begin
            // no data forwarding
            alu_b <= reg_file[instruction_word[1][`REG_K]];
          end
        end // `ADD_IC, `SUB_IC, `CMP_IC, `NOT_IC, `AND_IC, `BIC_IC, `OR_IC, `XOR_IC

        `CPY_IC: begin
          // set source alu_a
          if (instruction_word[1][`DT_CONTROL] == 1'b1) begin
            // copy from constant
            alu_a <= {{16{instruction_word[1][`DT_CONSTANT_MSB]}}, instruction_word[1][`DT_CONSTANT]}; // sign extend constant
          end
          else if ((instruction_word[1][`REG_J] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              alu_a <= alu_result;
            end
            else if (opcode[2] == `POP_IC) begin
              alu_a <= data_stack_out;
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(alu_a,31:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              alu_a <= shifter_result;
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              alu_a <= reg_file[instruction_word[1][`REG_J]];
            end
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
            // data forward from the increment/decrement of the stack pointer
            alu_a <= alu_result;
          end
          else if ((instruction_word[1][`REG_J] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            alu_a <= temp_wb;
            // TODO: data forward from other wb sources in mc3
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
            // data forward from the increment/decrement of the stack pointer
            alu_a <= temp_sp;
          end
          else begin
            // no data forwarding
            alu_a <= reg_file[instruction_word[1][`REG_J]];
          end

          // alu_b unused for cpy so just keep it the same
          alu_b <= alu_b;
        end // `CPY_IC

        `RS_IC: begin
          // set the opcode
          shifter_opcode <= instruction_word[1][`RS_OPCODE];

          // set the operand
          if ((instruction_word[1][`REG_J] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              shifter_operand <= alu_result;
            end
            else if (opcode[2] == `POP_IC) begin
              shifter_operand <= data_stack_out;
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(shifter_operand,31:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              shifter_operand <= shifter_result;
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
               shifter_operand <= reg_file[instruction_word[1][`REG_J]];
            end
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
            // data forward from the increment/decrement of the stack pointer
            shifter_operand <= alu_result;
          end
          else if ((instruction_word[1][`REG_J] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            shifter_operand <= temp_wb;
            // TODO: data forward from other wb sources in mc3
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
            // data forward from the increment/decrement of the stack pointer
            shifter_operand <= temp_sp;
          end
          else begin
            // no data forwarding
            shifter_operand <= reg_file[instruction_word[1][`REG_J]];
          end

          // set the modifier
          if (instruction_word[1][`RS_CONTROL] == 1'b1) begin
            // copy from constant
            shifter_modifier <= instruction_word[1][`RS_CONSTANT];
          end
          else if ((instruction_word[1][`REG_K] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2])  begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              shifter_modifier <= alu_result[5:0];
            end
            else if (opcode[2] == `POP_IC) begin
              shifter_modifier <= data_stack_out[5:0];
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(shifter_modifier,5:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              shifter_modifier <= shifter_result[5:0];
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              shifter_modifier <= reg_file[instruction_word[1][`REG_K]][5:0];
            end
          end
          else if (instruction_word[1][`REG_K] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
            // data forward from the increment/decrement of the stack pointer
            shifter_modifier <= alu_result[5:0];
          end
          else if ((instruction_word[1][`REG_K] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            shifter_modifier <= temp_wb[5:0];
            // TODO: data forward from other wb sources in mc3
          end
          else if (instruction_word[1][`REG_K] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
            // data forward from the increment/decrement of the stack pointer
            shifter_modifier <= temp_sp[5:0];
          end
          else begin
            // no data forwarding
            shifter_modifier <= reg_file[instruction_word[1][`REG_K]][5:0];
          end

          // set the carry in if rotating through carry
          if ((instruction_word[1][`RS_OPCODE] == `RRC_SHIFT) || (instruction_word[1][`RS_OPCODE] == `RLC_SHIFT)) begin
            if ((instruction_word[2][`REG_I] == `REG_SR) && `WB_INSTRUCTION(2) && !stall[2]) begin // if mc2 is writing to the REG_SR
              // data forward from mc2
              if (`ALU_INSTRUCTION(2)) begin
                shifter_carry_in <= alu_result[`SR_C];
              end
              else if (opcode[2] == `POP_IC) begin
                shifter_carry_in <= data_stack_out[`SR_C];
              end
              else if (opcode[2] == `LD_IC) begin
                `LOAD_MMIO(shifter_carry_in,`SR_C,)
              end
              else if (opcode[2] == `RS_IC) begin
                shifter_carry_in <= shifter_result[`SR_C];
              end
              // TODO: data forward from other wb sources in mc2
              else begin
                // no data forwarding
                shifter_carry_in <= reg_file[`REG_SR][`SR_C];
              end
            end
            else if ((instruction_word[3][`REG_I] == `REG_SR) && `WB_INSTRUCTION(3) && !stall[3]) begin // if mc3 is writing to the REG_SR
              // data forward from mc3
              shifter_carry_in <= temp_wb[`SR_C];
              // TODO: data forward from other wb sources in mc3
            end
            else if (`ALU_INSTRUCTION(2) && !stall[2]) begin // if the mc2 ALU instruction will change the REG_SR
              // data forward from the alu output
              shifter_carry_in <= alu_c;
            end
            else if (opcode[2] == `RS_IC && !stall[2]) begin // if the mc2 shift instruction will change the REG_SR
              shifter_carry_in <= shifter_carry_out;
            end
            else if (`ALU_INSTRUCTION(3) || opcode[3] == `RS_IC && !stall[3]) begin // if the mc3 instruction will change the REG_SR
              // data forward from the temp status register
              shifter_carry_in <= temp_sr[`SR_C];
            end
            else begin
              // no data forwarding
              shifter_carry_in <= reg_file[`REG_SR][`SR_C];
            end
          end
          else begin
            shifter_carry_in <= reg_file[`REG_SR][`SR_C];
          end

        end // `RS_IC

        `PUSH_IC: begin
          // data forwarding for the data input
          if (instruction_word[1][`DT_CONTROL] == 1'b1) begin
            // push from constant
            data_stack_data <= {{16{instruction_word[1][`DT_CONSTANT_MSB]}}, instruction_word[1][`DT_CONSTANT]};
          end
          else if ((instruction_word[1][`REG_J] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              data_stack_data <= alu_result;
            end
            else if (opcode[2] == `POP_IC) begin
              data_stack_data <= data_stack_out;
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(data_stack_data,31:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              data_stack_data <= shifter_result;
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              data_stack_data <= reg_file[instruction_word[1][`REG_J]];
            end
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
            // data forward from the increment/decrement of the stack pointer
            data_stack_data <= alu_result;
          end
          else if ((instruction_word[1][`REG_J] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            data_stack_data <= temp_wb;
            // TODO: data forward from other wb sources in mc3
          end
          else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
            // data forward from the increment/decrement of the stack pointer
            data_stack_data <= temp_sp;
          end
          else begin
            // no data forwarding
            data_stack_data <= reg_file[instruction_word[1][`REG_J]];
          end

          // data foward stack pointer
          // set alu_a to increment stack pointer
          if ((`REG_SP == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              // data forward from alu output
              alu_a <= alu_result;
              data_stack_addr <= alu_result[5:0];
            end
            else if (opcode[2] == `POP_IC) begin
              alu_a <= data_stack_out;
              data_stack_addr <= data_stack_out[5:0];
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(alu_a,31:0,)
              `LOAD_MMIO(data_stack_addr,5:0,)
            end
            else if (opcode[2] == `RS_IC) begin
              alu_a <= shifter_result;
              data_stack_addr <= shifter_result[5:0];
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              alu_a <= reg_file[`REG_SP];
              data_stack_addr <= reg_file[`REG_SP][5:0];
            end
          end
          else if ((opcode[2] == `PUSH_IC) || (opcode[2] == `POP_IC) && !stall[2]) begin
            // data forward from the output of the increment
            alu_a <= alu_result;
            data_stack_addr <= alu_result[5:0];
          end
          else if ((`REG_SP == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            alu_a <= temp_wb;
            data_stack_addr <= temp_wb[5:0];
            // TODO: data forward from other wb sources in mc3
          end
          else if ((opcode[3] == `PUSH_IC) || (opcode[3] == `POP_IC) && !stall[3]) begin
            // data forward from the output of the increment
            alu_a <= temp_sp;
            data_stack_addr <= temp_wb[5:0];
          end
          else begin
            // no data forwarding
            alu_a <= reg_file[`REG_SP];
            data_stack_addr <= reg_file[`REG_SP][5:0];
          end

          alu_b <= 32'h00000001;

          data_stack_push <= 1'b1;
        end

        `POP_IC: begin
          // data foward stack pointer
          // set alu_a to decrement stack pointer
          if ((`REG_SP == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
            // data forward from mc2
            if (`ALU_INSTRUCTION(2)) begin
              // data forward from alu output
              alu_a <= alu_result;
              data_stack_addr <= alu_result[5:0] - 1'b1;
            end
            else if (opcode[2] == `POP_IC) begin
              alu_a <= data_stack_out;
              data_stack_addr <= data_stack_out[5:0] - 1'b1;
            end
            else if (opcode[2] == `LD_IC) begin
              `LOAD_MMIO(alu_a,31:0,)
              // data_stack_addr <= dm_out[5:0] - 1'b1;
              `LOAD_MMIO(/*dest=*/data_stack_addr,/*bits=*/5:0,/*expr=*/-1'b1)
            end
            else if (opcode[2] == `RS_IC) begin
              alu_a <= shifter_result;
              data_stack_addr <= shifter_result[5:0] - 1'b1;
            end
            // TODO: data forward from other wb sources in mc2
            else begin
              // no data forwarding
              alu_a <= reg_file[`REG_SP];
              data_stack_addr <= reg_file[`REG_SP][5:0] - 1'b1;
            end
          end
          else if ((opcode[2] == `PUSH_IC) || (opcode[2] == `POP_IC) && !stall[2]) begin
            // data forward from the output of the increment
            alu_a <= alu_result;
            data_stack_addr <= alu_result[5:0] - 1'b1;
          end
          else if ((`REG_SP == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
            // data forward from mc3
            alu_a <= temp_wb;
            data_stack_addr <= temp_wb[5:0] - 1'b1;
            // TODO: data forward from other wb sources in mc3
          end
          else if ((opcode[3] == `PUSH_IC) || (opcode[3] == `POP_IC) && !stall[3]) begin
            // data forward from the output of the decrement
            alu_a <= temp_sp;
            data_stack_addr <= temp_sp[5:0] - 1'b1;
          end
          else begin
            // no data forwarding
            alu_a <= reg_file[`REG_SP];
            data_stack_addr <= reg_file[`REG_SP][5:0] - 1'b1;
          end

          alu_b <= 32'h00000001;
        end

        `LD_IC, `ST_IC: begin
          // Set the data memory address
          if (instruction_word[1][`REG_J] != 5'b0 && instruction_word[1][`DT_CONTROL] == 1'b0) begin
            // Indexed
            if ((instruction_word[1][`REG_J] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
              // data forward from mc2
              if (`ALU_INSTRUCTION(2)) begin
                dm_address <= alu_result + instruction_word[1][`DT_CONSTANT];
              end
              else if (opcode[2] == `POP_IC) begin
                dm_address <= data_stack_out + instruction_word[1][`DT_CONSTANT];
              end
              else if (opcode[2] == `LD_IC) begin
                `LOAD_MMIO(/*dest=*/dm_address,/*bits=*/31:0,/*expr=*/+instruction_word[1][`DT_CONSTANT])
              end
              else if (opcode[2] == `RS_IC) begin
                dm_address <= shifter_result + instruction_word[1][`DT_CONSTANT];
              end
              // TODO: data forward from other wb sources in mc2
              else begin
                 // No data forwarding
                dm_address <= reg_file[instruction_word[1][`REG_J]] + instruction_word[1][`DT_CONSTANT]; 
              end
            end
            else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
              // data forward from tne increment/decrement of the stack pointer
              dm_address <= alu_result + instruction_word[1][`DT_CONSTANT];
            end
            else if ((instruction_word[1][`REG_J] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
              // data forward from mc3
              dm_address <= temp_wb + instruction_word[1][`DT_CONSTANT];
              // TODO: data forward from other wb sources in mc3
            end
            else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
              // data forward from the increment/decrement of the stack pointer
              dm_address <= temp_sp + instruction_word[1][`DT_CONSTANT];
            end
            else begin
              // No data forwarding
              dm_address <= reg_file[instruction_word[1][`REG_J]] + instruction_word[1][`DT_CONSTANT];
            end
          end
          else if (instruction_word[1][`REG_J] != 5'b0 && instruction_word[1][`DT_CONTROL] == 1'b1) begin
            // Register Direct
            if ((instruction_word[1][`REG_J] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
              // data forward from mc2
              if (`ALU_INSTRUCTION(2)) begin
                dm_address <= alu_result;
              end
              else if (opcode[2] == `POP_IC) begin
                dm_address <= data_stack_out;
              end
              else if (opcode[2] == `LD_IC) begin
                `LOAD_MMIO(dm_address,31:0,)
              end
              else if (opcode[2] == `RS_IC) begin
                dm_address <= shifter_result;
              end
              // TODO: data forward from other wb sources in mc2
              else begin
                // No data forwarding
                dm_address <= reg_file[instruction_word[1][`REG_J]];
              end
            end
            else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
              // data forward from tne increment/decrement of the stack pointer
              dm_address <= alu_result;
            end
            else if ((instruction_word[1][`REG_J] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
              // data forward from mc3
              dm_address <= temp_wb;
              // TODO: data forward from other wb sources in mc3
            end
            else if (instruction_word[1][`REG_J] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
              // data forward from the increment/decrement of the stack pointer
              dm_address <= temp_sp;
            end
            else begin
              // No data forwarding
              dm_address <= reg_file[instruction_word[1][`REG_J]];
            end
          end
          else if (instruction_word[1][`REG_J] == 5'b0 && instruction_word[1][`DT_CONTROL] == 1'b0) begin
            // PC Relative
            dm_address <= instruction_addr[1] + instruction_word[1][`DT_CONSTANT];
          end
          else begin
            // Absolute
            dm_address <= instruction_word[1][`DT_CONSTANT];
          end


          // Set the data input
          if (opcode[1] == `ST_IC) begin

            // set the data value
            if ((instruction_word[1][`REG_I] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
              // data forward from mc2
              if (`ALU_INSTRUCTION(2)) begin
                dm_data <= alu_result;
              end
              else if (opcode[2] == `POP_IC) begin
                dm_data <= data_stack_out;
              end
              else if (opcode[2] == `LD_IC) begin
                `LOAD_MMIO(dm_data,31:0,)
              end
              else if (opcode[2] == `RS_IC) begin
                dm_data <= shifter_result;
              end
              // TODO: data forward from other wb sources in mc2
              else begin
                // No data forwarding
                dm_data <= reg_file[instruction_word[1][`REG_I]];
              end
            end
            else if (instruction_word[1][`REG_I] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
              // data forward from tne increment/decrement of the stack pointer
              dm_data <= alu_result;
            end
            else if ((instruction_word[1][`REG_I] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
              // data forward from mc3    
              dm_data <= temp_wb;
              // TODO: data forward from other wb sources in mc3
            end
            else if (instruction_word[1][`REG_I] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
              // data forward from the increment/decrement of the stack pointer
              dm_data <= temp_sp;
            end
            else begin
              // No data forwarding
              dm_data <= reg_file[instruction_word[1][`REG_I]];
            end
          end

        end

        `JMP_IC: begin
          // Set the temp program counter
          if (instruction_word[1][`REG_I] != 5'b0 && instruction_word[1][`JMP_CONTROL] == 1'b0) begin
            // Indexed
            if ((instruction_word[1][`REG_I] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
              // data forward from mc2
              if (`ALU_INSTRUCTION(2)) begin
                temp_address <= alu_result + instruction_word[1][`JMP_ADDR];
              end
              else if (opcode[2] == `POP_IC) begin
                temp_address <= data_stack_out + instruction_word[1][`JMP_ADDR];
              end
              else if (opcode[2] == `LD_IC) begin
                `LOAD_MMIO(/*dest=*/temp_address,/*bits=*/31:0,/*expr=*/+instruction_word[1][`JMP_ADDR])
              end
              else if (opcode[2] == `RS_IC) begin
                temp_address <= shifter_result + instruction_word[1][`JMP_ADDR];
              end
              // TODO: data forward from other wb sources in mc2
              else begin
                // No data forwarding
                temp_address <= reg_file[instruction_word[1][`REG_I]] + instruction_word[1][`JMP_ADDR];
              end
            end 
            else if (instruction_word[1][`REG_I] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
              // data forward from tne increment/decrement of the stack pointer
              temp_address <= alu_result + instruction_word[1][`JMP_ADDR];
            end
            else if ((instruction_word[1][`REG_I] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
              // data forward from mc3
              temp_address <= temp_wb + instruction_word[1][`JMP_ADDR];
              // TODO: data forward from other wb sources in mc3
            end
            else if (instruction_word[1][`REG_I] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
              // data forward from the increment/decrement of the stack pointer
              temp_address <= temp_sp + instruction_word[1][`JMP_ADDR];
            end
            else begin
              // No data forwarding
              temp_address <= reg_file[instruction_word[1][`REG_I]] + instruction_word[1][`JMP_ADDR];
            end
          end
          else if (instruction_word[1][`REG_I] != 5'b0 && instruction_word[1][`JMP_CONTROL] == 1'b1) begin
            // Register Direct
            if ((instruction_word[1][`REG_I] == instruction_word[2][`REG_I]) && `WB_INSTRUCTION(2) && !stall[2]) begin
              // data forward from mc2
              if (`ALU_INSTRUCTION(2)) begin
                temp_address <= alu_result;
              end
              else if (opcode[2] == `POP_IC) begin
                temp_address <= data_stack_out;
              end
              else if (opcode[2] == `LD_IC) begin
                `LOAD_MMIO(temp_address,31:0,)
              end
              else if (opcode[2] == `RS_IC) begin
                temp_address <= shifter_result;
              end
              // TODO: data forward from other wb sources in mc2
              else begin
                // No data forwarding
                temp_address <= reg_file[instruction_word[1][`REG_I]];
              end
            end
            else if (instruction_word[1][`REG_I] == `REG_SP && `STACK_INSTRUCTION(2) && !stall[2]) begin
              // data forward from tne increment/decrement of the stack pointer
              temp_address <= alu_result;
            end
            else if ((instruction_word[1][`REG_I] == instruction_word[3][`REG_I]) && `WB_INSTRUCTION(3) && !stall[3]) begin
              // data forward from mc3
              temp_address <= temp_wb;
              // TODO: data forward from other wb sources in mc3
            end
            else if (instruction_word[1][`REG_I] == `REG_SP && `STACK_INSTRUCTION(3) && !stall[3]) begin
              // data forward from the increment/decrement of the stack pointer
              temp_address <= temp_sp;
            end
            else begin
              // No data forwarding
              temp_address <= reg_file[instruction_word[1][`REG_I]];
            end
          end
          else if (instruction_word[1][`REG_I] == 5'b0 && instruction_word[1][`JMP_CONTROL] == 1'b0) begin
            // PC Relative
            temp_address <= instruction_addr[1] + instruction_word[1][`JMP_ADDR];
          end
          else begin
            // Absolute
            temp_address <= instruction_word[1][`JMP_ADDR];
          end
        end // JMP_IC

        `CALL_IC: begin
          // Set address
          // Always absolute mode for call (for now)
          temp_address <= instruction_word[1][`JMP_ADDR];

          // push the program counter onto the stack for when we return
          call_stack_push <= 1'b1;
          call_stack_data <= reg_file[`REG_PC];
        end

        `RET_IC: begin
          // pop the status register
          call_stack_pop <= 1'b1;
          reg_file[`REG_SR] <= call_stack_out;
        end

        default: begin
        end
      endcase // opcode[1]
      
      // set the alu opcode
      case (opcode[1])
        `ADD_IC, `PUSH_IC: begin
          alu_opcode <= `ADD_ALU;
        end

        `SUB_IC, `CMP_IC, `POP_IC: begin
          alu_opcode <= `SUB_ALU;
        end

        `NOT_IC: begin
          alu_opcode <= `NOT_ALU;
        end

        `AND_IC: begin
          alu_opcode <= `AND_ALU;
        end

        `BIC_IC: begin
          alu_opcode <= `BIC_ALU;
        end

        `OR_IC: begin
          alu_opcode <= `OR_ALU;
        end

        `XOR_IC: begin
          alu_opcode <= `XOR_ALU;
        end

        `CPY_IC: begin
          alu_opcode <= `NOP_ALU;
        end

        `MUL_IC: begin
          alu_opcode <= `MUL_ALU;
        end

        `DIV_IC: begin
          alu_opcode <= `DIV_ALU;
        end

        default: begin
          alu_opcode <= alu_opcode;
        end

      endcase // opcode[1]
      
      instruction_word[2] <= instruction_word[1];
      instruction_addr[2] <= instruction_addr[1];
    end // if (stall[1] == 1'b0)

    // Machine cycle 0
    // instruction fetch
    if (stall[0] == 1'b0) begin
      reg_file[`REG_PC] <= reg_file[`REG_PC] + 3'h4;
      instruction_addr[1] <= reg_file[`REG_PC][13:0];
      instruction_word[1] <= pm_out;

      // set stall cycles
      if ((opcode[0] == `JMP_IC) || (opcode[0] == `CALL_IC) || (opcode[0] == `RET_IC)) begin
        stall_cycles <= 3'h3;
        stall[0] <= 1'b1;
      end
    end // if (stall[0] == 1'b0)

  end // else begin
end // always @(posedge clk)

task reset_all; begin
  gpio_out <= 32'b0;
  dm_data <= 32'b0;
  dm_wren <= 1'b0;
  dm_address <= 14'b0;

  temp_address <= 16'b0;

  instruction_word[3] <= 32'b0;
  instruction_word[2] <= 32'b0;
  instruction_word[1] <= 32'b0;

  instruction_addr[3] <= 14'b0;
  instruction_addr[2] <= 14'b0;
  instruction_addr[1] <= 14'b0;

  stall_cycles <= 4'b0;
  stall[3] <= 1'b1;
  stall[2] <= 1'b1;
  stall[1] <= 1'b1;
  stall[0] <= 1'b1;

  data_stack_data <= 32'b0;
  data_stack_addr <= 6'b0;
  data_stack_push <= 1'b0;
  data_stack_pop <= 1'b0;

  call_stack_data <= 32'b0;
  call_stack_push <= 1'b0;
  call_stack_pop <= 1'b0;

  alu_a <= 32'b0;
  alu_b <= 32'b0;
  temp_sr <= 32'b0;
  temp_sp <= 32'b0;
  alu_opcode <= 4'b0;

  shifter_operand <= 32'b0;
  shifter_carry_in <= 1'b0;
  shifter_modifier <= 6'b0;
  shifter_opcode <= 3'b0;

  temp_wb <= 32'b0;

  for (i=0; i<32; i=i+1) begin
    reg_file[i] <= 32'h0;
  end

end
endtask // reset_all

endmodule // cjg_risc
