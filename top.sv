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
