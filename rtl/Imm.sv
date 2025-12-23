// rtl/ImmGen.sv
module ImmGen (
  input  logic [31:0] instr,

  output logic [31:0] imm,    // auto-selected by opcode (convenience)

  output logic [31:0] imm_i,  // I-type
  output logic [31:0] imm_s,  // S-type
  output logic [31:0] imm_b,  // B-type
  output logic [31:0] imm_u,  // U-type
  output logic [31:0] imm_j   // J-type
);

  import riscv_pkg::*;

  logic [6:0] opcode;

  always_comb begin
    opcode = instr[6:0];

    // I-type: imm[11:0] = instr[31:20]
    imm_i = {{20{instr[31]}}, instr[31:20]};

    // S-type: imm[11:0] = instr[31:25] | instr[11:7]
    imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};

    // B-type: imm[12|10:5|4:1|11|0] = instr[31|30:25|11:8|7|0]
    // LSB is always 0 (branch targets are 2-byte aligned)
    imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};

    // U-type: imm[31:12] = instr[31:12], low 12 bits are 0
    imm_u = {instr[31:12], 12'b0};

    // J-type: imm[20|10:1|11|19:12|0] = instr[31|30:21|20|19:12|0]
    // LSB is always 0
    imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    // Convenient "selected" immediate by opcode
    unique case (opcode)
      OPCODE_LOAD,
      OPCODE_OP_IMM,
      OPCODE_JALR:   imm = imm_i;

      OPCODE_STORE:  imm = imm_s;

      OPCODE_BRANCH: imm = imm_b;

      OPCODE_LUI,
      OPCODE_AUIPC:  imm = imm_u;

      OPCODE_JAL:    imm = imm_j;

      default:       imm = imm_i; // safe default
    endcase
  end

endmodule
