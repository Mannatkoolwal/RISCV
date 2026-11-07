// Single-cycle RV64I top level: connects all the modules (H&P datapath)
module riscv_single (
    input clk,
    input reset
);
    // ---------- wires ----------
    wire [63:0] pc, pc_plus4, branch_target;
    wire [31:0] instr;
    wire [63:0] imm, rd1, rd2, alu_b, alu_result, mem_rdata, wb_data;
    wire        branch, mem_read, mem_to_reg, mem_write, alu_src, reg_write;
    wire [1:0]  alu_op;
    wire [3:0]  alu_ctrl;
    wire        zero, pc_src;

    // ---------- instruction fields ----------
    wire [6:0] opcode   = instr[6:0];
    wire [4:0] rd       = instr[11:7];
    wire [2:0] funct3   = instr[14:12];
    wire [4:0] rs1      = instr[19:15];
    wire [4:0] rs2      = instr[24:20];
    wire       funct7_5 = instr[30];

    // ---------- branch logic ----------
    assign branch_target = pc + imm;      // imm already includes the x2 for B-type
    assign pc_src        = branch & zero; // beq: taken when rs1 - rs2 == 0

    // ---------- modules ----------
    fetch u_fetch (
        .clk(clk), .reset(reset),
        .pc_src(pc_src), .branch_target(branch_target),
        .pc(pc), .pc_plus4(pc_plus4)
    );

    instr_mem u_imem (.addr(pc), .instr(instr));

    main_control u_ctrl (
        .opcode(opcode),
        .branch(branch), .mem_read(mem_read), .mem_to_reg(mem_to_reg),
        .mem_write(mem_write), .alu_src(alu_src), .reg_write(reg_write),
        .alu_op(alu_op)
    );

    imm_gen u_imm (.instr(instr), .imm(imm));

    regfile u_rf (
        .clk(clk), .reset(reset),
        .readreg1(rs1), .readreg2(rs2),
        .writereg(rd), .writeen(reg_write), .writedata(wb_data),
        .readdata1(rd1), .readdata2(rd2)
    );

    assign alu_b = alu_src ? imm : rd2;   // register or immediate

    alu_control u_aluctrl (
        .alu_op(alu_op), .funct3(funct3), .funct7_5(funct7_5),
        .alu_ctrl(alu_ctrl)
    );

    alu u_alu (.a(rd1), .b(alu_b), .alu_ctrl(alu_ctrl), .result(alu_result), .zero(zero));

    data_mem u_dmem (
        .clk(clk), .mem_read(mem_read), .mem_write(mem_write),
        .addr(alu_result), .writedata(rd2), .readdata(mem_rdata)
    );

    assign wb_data = mem_to_reg ? mem_rdata : alu_result;  // write-back mux
endmodule

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

module regfile (
    input  logic        clk,
    input  logic        reset,
    input  logic [4:0]  readreg1,
    input  logic [4:0]  readreg2,
    input  logic [4:0]  writereg,
    input  logic        writeen,
    input  logic [63:0] writedata,
    output logic [63:0] readdata1,
    output logic [63:0] readdata2
);

    logic [63:0] rf [0:31];

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 32; i++) rf[i] <= 64'd0;
        end else if (writeen && writereg != 5'd0) begin
            rf[writereg] <= writedata;
        end
    end

    always_comb begin
        readdata1 = (readreg1 == 5'd0) ? 64'd0 : rf[readreg1];
        readdata2 = (readreg2 == 5'd0) ? 64'd0 : rf[readreg2];
    end
endmodule

module data_mem #(
    parameter DEPTH = 1024                 // number of 64-bit words
)(
    input         clk,
    input         mem_read,
    input         mem_write,
    input  [63:0] addr,                    // byte address
    input  [63:0] writedata,
    output [63:0] readdata
);
    localparam AW = $clog2(DEPTH);
    reg [63:0] mem [0:DEPTH-1];
    integer i; 
    initial for (i = 0; i < DEPTH; i = i + 1) mem[i] = 64'd0;
    

    assign readdata = mem[addr[AW+2:3]];   // drop the 3 byte-offset bits

    always @(posedge clk) begin
        if (mem_write) begin
            mem[addr[AW+2:3]] <= writedata;
        end
    end
endmodule

