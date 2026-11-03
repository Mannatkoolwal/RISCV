// ===== fetch =====
// Instruction fetch: PC register, PC+4 adder, next-PC mux (H&P Fig. 4.x)
module fetch (
    input         clk,
    input         reset,
    input         pc_src,          // 1 = take branch (Branch & Zero)
    input  [63:0] branch_target,   // PC + imm, computed in the datapath
    output reg [63:0] pc,
    output [63:0] pc_plus4
);
    wire [63:0] next_pc;

    assign pc_plus4 = pc + 64'd4;
    assign next_pc  = pc_src ? branch_target : pc_plus4;

    always @(posedge clk or posedge reset) begin
        if (reset) pc <= 64'd0;
        else       pc <= next_pc;
    end
endmodule

// ===== instr_mem =====
// Instruction memory: read-only, combinational read, 32-bit instructions
// (instructions stay 32 bits wide in RV64; only the address is 64 bits)
module instr_mem #(
    parameter DEPTH = 1024                 // number of 32-bit words
)(
    input  [63:0] addr,                    // byte address (the PC)
    output [31:0] instr
);
    localparam AW = $clog2(DEPTH);
    reg [31:0] mem [0:DEPTH-1];

    initial $readmemh("program.hex", mem); // one 32-bit hex word per line

    assign instr = mem[addr[AW+1:2]];      // drop the 2 byte-offset bits
endmodule

// ===== imm_gen =====
// Immediate generator: sign-extends to 64 bits.
// NOTE: B- and J-type outputs already include the implicit 0 in bit 0,
// so do NOT add a separate "shift left 1" in the datapath.
module imm_gen (
    input  [31:0] instr,
    output reg [63:0] imm
);
    always @(*) begin
        case (instr[6:0])
            // I-type: loads, ALU-immediate, jalr
            7'b0000011, 7'b0010011, 7'b1100111:
                imm = {{52{instr[31]}}, instr[31:20]};
            // S-type: stores
            7'b0100011:
                imm = {{52{instr[31]}}, instr[31:25], instr[11:7]};
            // B-type: branches
            7'b1100011:
                imm = {{51{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            // U-type: lui, auipc
            7'b0110111, 7'b0010111:
                imm = {{32{instr[31]}}, instr[31:12], 12'b0};
            // J-type: jal
            7'b1101111:
                imm = {{43{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            default:
                imm = 64'd0;
        endcase
    end
endmodule

// ===== main_control =====
// Main control: opcode -> control signals incl. 2-bit ALUOp (H&P)
// ALUOp: 00 = add (ld/sd)   01 = subtract (branch)
//        10 = R-type (decode funct)   11 = I-type ALU (decode funct3)
module main_control (
    input  [6:0] opcode,
    output reg       branch,
    output reg       mem_read,
    output reg       mem_to_reg,
    output reg       mem_write,
    output reg       alu_src,      // 1 = immediate, 0 = register
    output reg       reg_write,
    output reg [1:0] alu_op
);
    always @(*) begin
        // defaults (prevents latches)
        branch = 0; mem_read = 0; mem_to_reg = 0; mem_write = 0;
        alu_src = 0; reg_write = 0; alu_op = 2'b00;

        case (opcode)
            7'b0110011: begin // R-type
                reg_write = 1; alu_op = 2'b10;
            end
            7'b0010011: begin // I-type ALU (addi, andi, ...)
                reg_write = 1; alu_src = 1; alu_op = 2'b11;
            end
            7'b0000011: begin // ld
                reg_write = 1; alu_src = 1; mem_read = 1; mem_to_reg = 1; alu_op = 2'b00;
            end
            7'b0100011: begin // sd
                alu_src = 1; mem_write = 1; alu_op = 2'b00;
            end
            7'b1100011: begin // beq
                branch = 1; alu_op = 2'b01;
            end
            default: ;
        endcase
    end
endmodule

// ===== alu_control =====
// ALU control: ALUOp + funct fields -> 4-bit ALU select
module alu_control (
    input  [1:0] alu_op,
    input  [2:0] funct3,
    input        funct7_5,        // instr[30]
    output reg [3:0] alu_ctrl
);
    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = 4'b0010;            // ADD (ld/sd address)
            2'b01: alu_ctrl = 4'b0110;            // SUB (beq compare)
            2'b10: begin                          // R-type
                case (funct3)
                    3'b000: alu_ctrl = funct7_5 ? 4'b0110 : 4'b0010; // SUB : ADD
                    3'b001: alu_ctrl = 4'b0100;                      // SLL
                    3'b010: alu_ctrl = 4'b1000;                      // SLT
                    3'b011: alu_ctrl = 4'b1001;                      // SLTU
                    3'b100: alu_ctrl = 4'b0011;                      // XOR
                    3'b101: alu_ctrl = funct7_5 ? 4'b0111 : 4'b0101; // SRA : SRL
                    3'b110: alu_ctrl = 4'b0001;                      // OR
                    3'b111: alu_ctrl = 4'b0000;                      // AND
                endcase
            end
            2'b11: begin                          // I-type ALU (no SUB form)
                case (funct3)
                    3'b000: alu_ctrl = 4'b0010;                      // ADDI
                    3'b001: alu_ctrl = 4'b0100;                      // SLLI
                    3'b010: alu_ctrl = 4'b1000;                      // SLTI
                    3'b011: alu_ctrl = 4'b1001;                      // SLTIU
                    3'b100: alu_ctrl = 4'b0011;                      // XORI
                    3'b101: alu_ctrl = funct7_5 ? 4'b0111 : 4'b0101; // SRAI : SRLI
                    3'b110: alu_ctrl = 4'b0001;                      // ORI
                    3'b111: alu_ctrl = 4'b0000;                      // ANDI
                endcase
            end
        endcase
    end
endmodule

// ===== alu =====
// 64-bit combinational ALU (RV64I). Shift amount is 6 bits in RV64.
module alu (
    input  [63:0] a, b,
    input  [3:0]  alu_ctrl,
    output reg [63:0] result,
    output zero
);
    always @(*) begin
        case (alu_ctrl)
            4'b0000: result = a & b;                                   // AND
            4'b0001: result = a | b;                                   // OR
            4'b0010: result = a + b;                                   // ADD
            4'b0011: result = a ^ b;                                   // XOR
            4'b0100: result = a << b[5:0];                             // SLL
            4'b0101: result = a >> b[5:0];                             // SRL
            4'b0110: result = a - b;                                   // SUB
            4'b0111: result = $signed(a) >>> b[5:0];                   // SRA
            4'b1000: result = ($signed(a) < $signed(b)) ? 64'd1 : 64'd0; // SLT
            4'b1001: result = (a < b) ? 64'd1 : 64'd0;                 // SLTU
            default: result = 64'd0;
        endcase
    end

    assign zero = (result == 64'd0);
endmodule

