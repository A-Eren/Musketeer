// rtl/Decoder.sv
module Decoder (
  input  logic [31:0] instr,

  // extracted fields
  output logic [6:0]  opcode,
  output logic [4:0]  rd,
  output logic [2:0]  funct3,
  output logic [4:0]  rs1,
  output logic [4:0]  rs2,
  output logic [6:0]  funct7,

  // immediate selection
  output riscv_pkg::imm_sel_t  imm_sel,

  // ALU control
  output riscv_pkg::alu_op_t   alu_op,
  output riscv_pkg::src_a_sel_t src_a_sel,
  output riscv_pkg::src_b_sel_t src_b_sel,

  // LSU control
  output logic        is_load,
  output logic        is_store,

  // branch/jump control
  output logic        is_branch,
  output logic        is_jal,
  output logic        is_jalr,

  // writeback + regfile
  output logic        reg_write,
  output riscv_pkg::wb_sel_t wb_sel,

  // optional helpers (nice for later hazards)
  output logic        uses_rs1,
  output logic        uses_rs2,

  // illegal/unsupported instruction flag (you can trap later)
  output logic        illegal
);

  import riscv_pkg::*;

  always_comb begin
    // -------------------------
    // Field extraction
    // -------------------------
    opcode = instr[6:0];
    rd     = instr[11:7];
    funct3 = instr[14:12];
    rs1    = instr[19:15];
    rs2    = instr[24:20];
    funct7 = instr[31:25];

    // -------------------------
    // Safe defaults
    // -------------------------
    imm_sel   = IMM_I;

    alu_op    = ALU_ADD;
    src_a_sel = SRC_A_RS1;
    src_b_sel = SRC_B_RS2;

    is_load   = 1'b0;
    is_store  = 1'b0;

    is_branch = 1'b0;
    is_jal    = 1'b0;
    is_jalr   = 1'b0;

    reg_write = 1'b0;
    wb_sel    = WB_ALU;

    uses_rs1  = 1'b0;
    uses_rs2  = 1'b0;

    illegal   = 1'b0;

    // -------------------------
    // Main opcode decode (RV32I)
    // -------------------------
    unique case (opcode)

      // R-type ALU ops
      OPCODE_OP: begin
        reg_write = 1'b1;
        wb_sel    = WB_ALU;

        src_a_sel = SRC_A_RS1;
        src_b_sel = SRC_B_RS2;

        uses_rs1  = 1'b1;
        uses_rs2  = 1'b1;

        // ALU op decode
        unique case (funct3)
          3'b000: alu_op = (funct7[FUNCT7_ALT_BIT]) ? ALU_SUB : ALU_ADD;
          3'b001: alu_op = ALU_SLL;
          3'b010: alu_op = ALU_SLT;
          3'b011: alu_op = ALU_SLTU;
          3'b100: alu_op = ALU_XOR;
          3'b101: alu_op = (funct7[FUNCT7_ALT_BIT]) ? ALU_SRA : ALU_SRL;
          3'b110: alu_op = ALU_OR;
          3'b111: alu_op = ALU_AND;
          default: begin alu_op = ALU_ADD; illegal = 1'b1; end
        endcase
      end

      // I-type ALU ops
      OPCODE_OP_IMM: begin
        reg_write = 1'b1;
        wb_sel    = WB_ALU;

        imm_sel   = IMM_I;
        src_a_sel = SRC_A_RS1;
        src_b_sel = SRC_B_IMM;

        uses_rs1  = 1'b1;
        uses_rs2  = 1'b0;

        unique case (funct3)
          3'b000: alu_op = ALU_ADD;   // ADDI
          3'b010: alu_op = ALU_SLT;   // SLTI
          3'b011: alu_op = ALU_SLTU;  // SLTIU
          3'b100: alu_op = ALU_XOR;   // XORI
          3'b110: alu_op = ALU_OR;    // ORI
          3'b111: alu_op = ALU_AND;   // ANDI
          3'b001: alu_op = ALU_SLL;   // SLLI
          3'b101: alu_op = (funct7[FUNCT7_ALT_BIT]) ? ALU_SRA : ALU_SRL; // SRAI/SRLI
          default: begin alu_op = ALU_ADD; illegal = 1'b1; end
        endcase
      end

      // Loads
      OPCODE_LOAD: begin
        reg_write = 1'b1;
        wb_sel    = WB_MEM;

        is_load   = 1'b1;

        imm_sel   = IMM_I;
        src_a_sel = SRC_A_RS1;
        src_b_sel = SRC_B_IMM;
        alu_op    = ALU_ADD; // address = rs1 + imm

        uses_rs1  = 1'b1;
        uses_rs2  = 1'b0;

        // (optional) you can mark illegal on unknown funct3
        if (!(funct3 inside {F3_LB, F3_LH, F3_LW, F3_LBU, F3_LHU})) illegal = 1'b1;
      end

      // Stores
      OPCODE_STORE: begin
        reg_write = 1'b0;

        is_store  = 1'b1;

        imm_sel   = IMM_S;
        src_a_sel = SRC_A_RS1;
        src_b_sel = SRC_B_IMM;
        alu_op    = ALU_ADD; // address = rs1 + imm

        uses_rs1  = 1'b1;
        uses_rs2  = 1'b1; // store data comes from rs2

        if (!(funct3 inside {F3_SB, F3_SH, F3_SW})) illegal = 1'b1;
      end

      // Branches
      OPCODE_BRANCH: begin
        reg_write = 1'b0;

        is_branch = 1'b1;

        imm_sel   = IMM_B;
        src_a_sel = SRC_A_RS1;
        src_b_sel = SRC_B_RS2;
        alu_op    = ALU_SUB; // common: use zero flag for BEQ/BNE

        uses_rs1  = 1'b1;
        uses_rs2  = 1'b1;

        // RV32I branch funct3 are 000/001/100/101/110/111
        if (!(funct3 inside {3'b000,3'b001,3'b100,3'b101,3'b110,3'b111})) illegal = 1'b1;
      end

      // JAL
      OPCODE_JAL: begin
        reg_write = 1'b1;
        wb_sel    = WB_PC4;

        is_jal    = 1'b1;

        imm_sel   = IMM_J;

        // target calc often uses PC + imm
        src_a_sel = SRC_A_PC;
        src_b_sel = SRC_B_IMM;
        alu_op    = ALU_ADD;

        uses_rs1  = 1'b0;
        uses_rs2  = 1'b0;
      end

      // JALR
      OPCODE_JALR: begin
        reg_write = 1'b1;
        wb_sel    = WB_PC4;

        is_jalr   = 1'b1;

        imm_sel   = IMM_I;

        // target = rs1 + imm, then clear bit0 outside (per spec)
        src_a_sel = SRC_A_RS1;
        src_b_sel = SRC_B_IMM;
        alu_op    = ALU_ADD;

        uses_rs1  = 1'b1;
        uses_rs2  = 1'b0;

        // funct3 must be 000 for JALR in RV32I
        if (funct3 != 3'b000) illegal = 1'b1;
      end

      // AUIPC: rd = PC + (imm_u)
      OPCODE_AUIPC: begin
        reg_write = 1'b1;
        wb_sel    = WB_ALU;

        imm_sel   = IMM_U;

        src_a_sel = SRC_A_PC;
        src_b_sel = SRC_B_IMM;
        alu_op    = ALU_ADD;

        uses_rs1  = 1'b0;
        uses_rs2  = 1'b0;
      end

      // LUI: rd = imm_u
      OPCODE_LUI: begin
        reg_write = 1'b1;
        wb_sel    = WB_ALU;

        imm_sel   = IMM_U;

        // easiest: ALU copies B (imm)
        src_a_sel = SRC_A_RS1; // don't care
        src_b_sel = SRC_B_IMM;
        alu_op    = ALU_COPY_B;

        uses_rs1  = 1'b0;
        uses_rs2  = 1'b0;
      end

      default: begin
        illegal = 1'b1; // unsupported opcode
      end

    endcase
  end

endmodule
