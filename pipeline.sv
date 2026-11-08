// 5-stage pipelined RV64I (IF, ID, EX, MEM, WB)
// Reuses: instr_mem, main_control, imm_gen, regfile, alu_control, alu, data_mem
// Hazards handled:
//   - EX/MEM and MEM/WB forwarding into EX
//   - register-file write-through (WB -> ID in the same cycle)
//   - load-use stall (1 bubble)
//   - taken beq resolved in EX: flush IF/ID and ID/EX (2-cycle penalty)
module riscv_pipe (
    input clk,
    input reset
);
    localparam [31:0] NOP = 32'h00000013;   // addi x0, x0, 0

    // ================= pipeline registers (declared first) =================
    // IF/ID
    reg [63:0] ifid_pc;
    reg [31:0] ifid_instr;
    // ID/EX
    reg [63:0] idex_pc, idex_rd1, idex_rd2, idex_imm;
    reg [4:0]  idex_rs1, idex_rs2, idex_rd;
    reg [2:0]  idex_funct3;
    reg        idex_funct7_5;
    reg        idex_branch, idex_mem_read, idex_mem_to_reg, idex_mem_write, idex_alu_src, idex_reg_write;
    reg [1:0]  idex_alu_op;
    // EX/MEM
    reg [63:0] exmem_alu_result, exmem_store_data;
    reg [4:0]  exmem_rd;
    reg        exmem_mem_read, exmem_mem_write, exmem_mem_to_reg, exmem_reg_write;
    // MEM/WB
    reg [63:0] memwb_alu_result, memwb_mem_data;
    reg [4:0]  memwb_rd;
    reg        memwb_mem_to_reg, memwb_reg_write;

    // =========================== IF ===========================
    reg  [63:0] pc;
    wire [63:0] pc_plus4 = pc + 64'd4;
    wire [31:0] if_instr;
    instr_mem u_imem (.addr(pc), .instr(if_instr));


    // =========================== ID ===========================
    wire [6:0] id_opcode   = ifid_instr[6:0];
    wire [4:0] id_rd       = ifid_instr[11:7];
    wire [2:0] id_funct3   = ifid_instr[14:12];
    wire [4:0] id_rs1      = ifid_instr[19:15];
    wire [4:0] id_rs2      = ifid_instr[24:20];
    wire       id_funct7_5 = ifid_instr[30];

    wire       id_branch, id_mem_read, id_mem_to_reg, id_mem_write, id_alu_src, id_reg_write;
    wire [1:0] id_alu_op;
    main_control u_ctrl (
        .opcode(id_opcode),
        .branch(id_branch), .mem_read(id_mem_read), .mem_to_reg(id_mem_to_reg),
        .mem_write(id_mem_write), .alu_src(id_alu_src), .reg_write(id_reg_write),
        .alu_op(id_alu_op)
    );

    wire [63:0] id_imm;
    imm_gen u_imm (.instr(ifid_instr), .imm(id_imm));

    wire [63:0] rf_rd1, rf_rd2, wb_data;
    regfile u_rf (
        .clk(clk), .reset(reset),
        .readreg1(id_rs1), .readreg2(id_rs2),
        .writereg(memwb_rd), .writeen(memwb_reg_write), .writedata(wb_data),
        .readdata1(rf_rd1), .readdata2(rf_rd2)
    );

    // write-through: if WB writes the register ID is reading this cycle, use the new value
    wire [63:0] id_rd1 = (memwb_reg_write && memwb_rd != 5'd0 && memwb_rd == id_rs1) ? wb_data : rf_rd1;
    wire [63:0] id_rd2 = (memwb_reg_write && memwb_rd != 5'd0 && memwb_rd == id_rs2) ? wb_data : rf_rd2;

    // load-use hazard: instruction in EX is a load whose rd is needed by the instruction in ID
    wire stall = idex_mem_read && (idex_rd != 5'd0) &&
                 (idex_rd == id_rs1 || idex_rd == id_rs2);


    // =========================== EX ===========================
    reg [1:0] fwd_a, fwd_b;      // 00 = register file, 10 = from EX/MEM, 01 = from MEM/WB
    always @(*) begin
        fwd_a = 2'b00;
        if      (exmem_reg_write && exmem_rd != 5'd0 && exmem_rd == idex_rs1) fwd_a = 2'b10;
        else if (memwb_reg_write && memwb_rd != 5'd0 && memwb_rd == idex_rs1) fwd_a = 2'b01;

        fwd_b = 2'b00;
        if      (exmem_reg_write && exmem_rd != 5'd0 && exmem_rd == idex_rs2) fwd_b = 2'b10;
        else if (memwb_reg_write && memwb_rd != 5'd0 && memwb_rd == idex_rs2) fwd_b = 2'b01;
    end

    wire [63:0] ex_a     = (fwd_a == 2'b10) ? exmem_alu_result :
                           (fwd_a == 2'b01) ? wb_data : idex_rd1;
    wire [63:0] ex_b_reg = (fwd_b == 2'b10) ? exmem_alu_result :
                           (fwd_b == 2'b01) ? wb_data : idex_rd2;   // also the store data
    wire [63:0] ex_b     = idex_alu_src ? idex_imm : ex_b_reg;

    wire [3:0]  ex_alu_ctrl;
    wire [63:0] ex_alu_result;
    wire        ex_zero;
    alu_control u_aluctrl (
        .alu_op(idex_alu_op), .funct3(idex_funct3), .funct7_5(idex_funct7_5),
        .alu_ctrl(ex_alu_ctrl)
    );
    alu u_alu (.a(ex_a), .b(ex_b), .alu_ctrl(ex_alu_ctrl), .result(ex_alu_result), .zero(ex_zero));

    wire [63:0] ex_branch_target = idex_pc + idex_imm;
    // beq (funct3=000): taken when result == 0;  bne (funct3=001): taken when result != 0
    wire        ex_take          = (idex_funct3 == 3'b000 &&  ex_zero) ||
                                   (idex_funct3 == 3'b001 && !ex_zero);
    wire        ex_pc_src        = idex_branch & ex_take;   // taken branch -> flush


    // =========================== MEM ==========================
    wire [63:0] mem_rdata;
    data_mem u_dmem (
        .clk(clk), .mem_read(exmem_mem_read), .mem_write(exmem_mem_write),
        .addr(exmem_alu_result), .writedata(exmem_store_data), .readdata(mem_rdata)
    );


    // =========================== WB ===========================
    assign wb_data = memwb_mem_to_reg ? memwb_mem_data : memwb_alu_result;

    // ===================== sequential updates =================
    // PC
    always @(posedge clk or posedge reset) begin
        if (reset)            pc <= 64'd0;
        else if (ex_pc_src)   pc <= ex_branch_target;   // taken branch
        else if (!stall)      pc <= pc_plus4;           // (stall: hold PC)
    end

    // IF/ID
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ifid_pc <= 64'd0; ifid_instr <= NOP;
        end else if (ex_pc_src) begin                   // flush wrong-path instruction
            ifid_pc <= 64'd0; ifid_instr <= NOP;
        end else if (!stall) begin
            ifid_pc <= pc;    ifid_instr <= if_instr;
        end                                             // stall: hold
    end

    // ID/EX
    always @(posedge clk or posedge reset) begin
        if (reset || ex_pc_src || stall) begin          // bubble: all control signals = 0
            idex_pc <= 64'd0; idex_rd1 <= 64'd0; idex_rd2 <= 64'd0; idex_imm <= 64'd0;
            idex_rs1 <= 5'd0; idex_rs2 <= 5'd0; idex_rd <= 5'd0;
            idex_funct3 <= 3'd0; idex_funct7_5 <= 1'b0;
            idex_branch <= 0; idex_mem_read <= 0; idex_mem_to_reg <= 0;
            idex_mem_write <= 0; idex_alu_src <= 0; idex_reg_write <= 0;
            idex_alu_op <= 2'b00;
        end else begin
            idex_pc <= ifid_pc; idex_rd1 <= id_rd1; idex_rd2 <= id_rd2; idex_imm <= id_imm;
            idex_rs1 <= id_rs1; idex_rs2 <= id_rs2; idex_rd <= id_rd;
            idex_funct3 <= id_funct3; idex_funct7_5 <= id_funct7_5;
            idex_branch <= id_branch; idex_mem_read <= id_mem_read;
            idex_mem_to_reg <= id_mem_to_reg; idex_mem_write <= id_mem_write;
            idex_alu_src <= id_alu_src; idex_reg_write <= id_reg_write;
            idex_alu_op <= id_alu_op;
        end
    end

    // EX/MEM
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            exmem_alu_result <= 64'd0; exmem_store_data <= 64'd0; exmem_rd <= 5'd0;
            exmem_mem_read <= 0; exmem_mem_write <= 0; exmem_mem_to_reg <= 0; exmem_reg_write <= 0;
        end else begin
            exmem_alu_result <= ex_alu_result; exmem_store_data <= ex_b_reg; exmem_rd <= idex_rd;
            exmem_mem_read <= idex_mem_read; exmem_mem_write <= idex_mem_write;
            exmem_mem_to_reg <= idex_mem_to_reg; exmem_reg_write <= idex_reg_write;
        end
    end

    // MEM/WB
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            memwb_alu_result <= 64'd0; memwb_mem_data <= 64'd0; memwb_rd <= 5'd0;
            memwb_mem_to_reg <= 0; memwb_reg_write <= 0;
        end else begin
            memwb_alu_result <= exmem_alu_result; memwb_mem_data <= mem_rdata; memwb_rd <= exmem_rd;
            memwb_mem_to_reg <= exmem_mem_to_reg; memwb_reg_write <= exmem_reg_write;
        end
    end
endmodule
