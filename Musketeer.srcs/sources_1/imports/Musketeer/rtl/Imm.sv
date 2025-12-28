// rtl/ImmGen.sv
module ImmGen (
  input  logic [31:0] instr,

  output riscv_pkg::imm_sel_t imm_sel, // selected immediate type
  output logic [31:0]         imm,     // selected immediate (based on imm_sel)

  output logic [31:0] imm_i,
  output logic [31:0] imm_s,
  output logic [31:0] imm_b,
  output logic [31:0] imm_u,
  output logic [31:0] imm_j
);

  import riscv_pkg::*;

  logic [6:0] opcode;

  always_comb begin
    opcode = instr[6:0];

    // I-type: instr[31:20]
    imm_i = {{20{instr[31]}}, instr[31:20]};

    // S-type: instr[31:25] | instr[11:7]
    imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};

    // B-type: instr[31|7|30:25|11:8|0]
    imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};

    // U-type: instr[31:12] << 12
    imm_u = {instr[31:12], 12'b0};

    // J-type: instr[31|19:12|20|30:21|0]
    imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    // imm_sel by opcode (decoder can also replicate this later if desired)
    unique case (opcode)
      OPCODE_LOAD,
      OPCODE_OP_IMM,
      OPCODE_JALR:   imm_sel = IMM_I;

      OPCODE_STORE:  imm_sel = IMM_S;

      OPCODE_BRANCH: imm_sel = IMM_B;

      OPCODE_LUI,
      OPCODE_AUIPC:  imm_sel = IMM_U;

      OPCODE_JAL:    imm_sel = IMM_J;

      default:       imm_sel = IMM_I;
    endcase

    // Selected imm output
    unique case (imm_sel)
      IMM_I: imm = imm_i;
      IMM_S: imm = imm_s;
      IMM_B: imm = imm_b;
      IMM_U: imm = imm_u;
      IMM_J: imm = imm_j;
      default: imm = imm_i;
    endcase
  end

endmodule
