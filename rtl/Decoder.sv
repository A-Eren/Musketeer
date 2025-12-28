module Decoder (
  input  riscv_pkg::dec_in_t  dec_i,
  output riscv_pkg::dec_out_t dec_o
);

  import riscv_pkg::*;

  logic [31:0] instr;

  logic [6:0] opcode_bits;
  opcode_t    opcode;

  logic [4:0] rd;
  logic [2:0] funct3;
  logic [4:0] rs1;
  logic [4:0] rs2;
  logic [6:0] funct7;

  logic shamt_legal;
  logic sr_legal;

  always_comb begin
    instr       = dec_i.instr;

    opcode_bits = instr[6:0];
    opcode      = opcode_t'(opcode_bits);

    rd          = instr[11:7];
    funct3      = instr[14:12];
    rs1         = instr[19:15];
    rs2         = instr[24:20];
    funct7      = instr[31:25];

    // outputs: fields
    dec_o.opcode = opcode;
    dec_o.rd     = rd;
    dec_o.funct3 = funct3;
    dec_o.rs1    = rs1;
    dec_o.rs2    = rs2;
    dec_o.funct7 = funct7;

    // defaults
    dec_o.imm_sel   = IMM_I;

    dec_o.alu_op    = ALU_ADD;
    dec_o.src_a_sel = SRC_A_RS1;
    dec_o.src_b_sel = SRC_B_RS2;

    dec_o.wb_sel    = WB_ALU;
    dec_o.reg_write = 1'b0;

    dec_o.br_op     = BR_NONE;
    dec_o.lsu_op    = LSU_NONE;

    dec_o.is_jal    = 1'b0;
    dec_o.is_jalr   = 1'b0;

    dec_o.uses_rs1  = 1'b0;
    dec_o.uses_rs2  = 1'b0;

    dec_o.illegal   = 1'b0;

    // shift legality (RV32I)
    shamt_legal = (funct7 == 7'b0000000);      // SLLI
    sr_legal = (funct7[6] == 1'b0) && (funct7[4:0] == 5'b00000); // allows 0000000 and 0100000


    unique case (opcode)

      // -------------------------
      // R-type ALU
      // -------------------------
      OP_OP: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_ALU;

        dec_o.src_a_sel = SRC_A_RS1;
        dec_o.src_b_sel = SRC_B_RS2;
        dec_o.uses_rs1  = 1'b1;
        dec_o.uses_rs2  = 1'b1;

        unique case (funct3)
          3'b000: dec_o.alu_op = (instr[30]) ? ALU_SUB : ALU_ADD; // funct7[5] == instr[30]
          3'b001: dec_o.alu_op = ALU_SLL;
          3'b010: dec_o.alu_op = ALU_SLT;
          3'b011: dec_o.alu_op = ALU_SLTU;
          3'b100: dec_o.alu_op = ALU_XOR;
          3'b101: dec_o.alu_op = (instr[30]) ? ALU_SRA : ALU_SRL;
          3'b110: dec_o.alu_op = ALU_OR;
          3'b111: dec_o.alu_op = ALU_AND;
          default: dec_o.illegal = 1'b1;
        endcase
      end

      // -------------------------
      // I-type ALU immediates
      // -------------------------
      OP_OP_IMM: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_ALU;

        dec_o.imm_sel   = IMM_I;
        dec_o.src_a_sel = SRC_A_RS1;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.uses_rs1  = 1'b1;

        unique case (funct3)
          3'b000: dec_o.alu_op = ALU_ADD;   // ADDI
          3'b010: dec_o.alu_op = ALU_SLT;   // SLTI
          3'b011: dec_o.alu_op = ALU_SLTU;  // SLTIU
          3'b100: dec_o.alu_op = ALU_XOR;   // XORI
          3'b110: dec_o.alu_op = ALU_OR;    // ORI
          3'b111: dec_o.alu_op = ALU_AND;   // ANDI

          3'b001: begin                    // SLLI
            dec_o.alu_op = ALU_SLL;
            if (!shamt_legal) dec_o.illegal = 1'b1;
          end

          3'b101: begin                    // SRLI/SRAI
            dec_o.alu_op = (instr[30]) ? ALU_SRA : ALU_SRL;
            if (!sr_legal) dec_o.illegal = 1'b1;
          end

          default: dec_o.illegal = 1'b1;
        endcase
      end

      // -------------------------
      // Loads -> lsu_op
      // -------------------------
      OP_LOAD: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_MEM;

        dec_o.imm_sel   = IMM_I;
        dec_o.src_a_sel = SRC_A_RS1;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.alu_op    = ALU_ADD;

        dec_o.uses_rs1  = 1'b1;

        unique case (funct3)
          3'b000: dec_o.lsu_op = LSU_LB;
          3'b001: dec_o.lsu_op = LSU_LH;
          3'b010: dec_o.lsu_op = LSU_LW;
          3'b100: dec_o.lsu_op = LSU_LBU;
          3'b101: dec_o.lsu_op = LSU_LHU;
          default: begin
            dec_o.lsu_op  = LSU_NONE;
            dec_o.illegal = 1'b1;
          end
        endcase
      end

      // -------------------------
      // Stores -> lsu_op
      // -------------------------
      OP_STORE: begin
        dec_o.reg_write = 1'b0;

        dec_o.imm_sel   = IMM_S;
        dec_o.src_a_sel = SRC_A_RS1;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.alu_op    = ALU_ADD;

        dec_o.uses_rs1  = 1'b1;
        dec_o.uses_rs2  = 1'b1;

        unique case (funct3)
          3'b000: dec_o.lsu_op = LSU_SB;
          3'b001: dec_o.lsu_op = LSU_SH;
          3'b010: dec_o.lsu_op = LSU_SW;
          default: begin
            dec_o.lsu_op  = LSU_NONE;
            dec_o.illegal = 1'b1;
          end
        endcase
      end

      // -------------------------
      // Branch -> br_op
      // -------------------------
      OP_BRANCH: begin
        dec_o.reg_write = 1'b0;

        dec_o.src_a_sel = SRC_A_RS1;
        dec_o.src_b_sel = SRC_B_RS2;
        dec_o.alu_op    = ALU_SUB;

        dec_o.imm_sel   = IMM_B;

        dec_o.uses_rs1  = 1'b1;
        dec_o.uses_rs2  = 1'b1;

        unique case (funct3)
          3'b000: dec_o.br_op = BR_BEQ;
          3'b001: dec_o.br_op = BR_BNE;
          3'b100: dec_o.br_op = BR_BLT;
          3'b101: dec_o.br_op = BR_BGE;
          3'b110: dec_o.br_op = BR_BLTU;
          3'b111: dec_o.br_op = BR_BGEU;
          default: begin
            dec_o.br_op   = BR_NONE;
            dec_o.illegal = 1'b1;
          end
        endcase
      end

      // -------------------------
      // JAL
      // -------------------------
      OP_JAL: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_PC4;

        dec_o.is_jal    = 1'b1;

        dec_o.imm_sel   = IMM_J;
        dec_o.src_a_sel = SRC_A_PC;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.alu_op    = ALU_ADD;
      end

      // -------------------------
      // JALR
      // -------------------------
      OP_JALR: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_PC4;

        dec_o.is_jalr   = 1'b1;

        dec_o.imm_sel   = IMM_I;
        dec_o.src_a_sel = SRC_A_RS1;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.alu_op    = ALU_ADD;

        dec_o.uses_rs1  = 1'b1;

        if (funct3 != 3'b000) dec_o.illegal = 1'b1;
      end

      // -------------------------
      // AUIPC
      // -------------------------
      OP_AUIPC: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_ALU;

        dec_o.imm_sel   = IMM_U;
        dec_o.src_a_sel = SRC_A_PC;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.alu_op    = ALU_ADD;
      end

      // -------------------------
      // LUI
      // -------------------------
      OP_LUI: begin
        dec_o.reg_write = 1'b1;
        dec_o.wb_sel    = WB_ALU;

        dec_o.imm_sel   = IMM_U;
        dec_o.src_b_sel = SRC_B_IMM;
        dec_o.alu_op    = ALU_COPY_B;
      end

      default: begin
        dec_o.illegal = 1'b1;
      end

    endcase
  end

endmodule
