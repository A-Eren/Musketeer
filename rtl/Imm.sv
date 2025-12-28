// rtl/Imm.sv  (or rtl/ImmGen.sv)
module ImmGen (
  input  riscv_pkg::imm_in_t  imm_i,
  output riscv_pkg::imm_out_t imm_o
);

  import riscv_pkg::*;

  // Sign-extend a 32-bit value to XLEN
  function automatic logic [XLEN-1:0] sext32(input logic [31:0] x);
    sext32 = {{(XLEN-32){x[31]}}, x};
  endfunction

  logic [31:0] imm_i32, imm_s32, imm_b32, imm_u32, imm_j32;

  always_comb begin
    // raw 32-bit immediates (RV32I formats)
    imm_i32 = {{20{imm_i.instr[31]}}, imm_i.instr[31:20]};
    imm_s32 = {{20{imm_i.instr[31]}}, imm_i.instr[31:25], imm_i.instr[11:7]};
    imm_b32 = {{19{imm_i.instr[31]}}, imm_i.instr[31], imm_i.instr[7],
               imm_i.instr[30:25], imm_i.instr[11:8], 1'b0};
    imm_u32 = {imm_i.instr[31:12], 12'b0};
    imm_j32 = {{11{imm_i.instr[31]}}, imm_i.instr[31], imm_i.instr[19:12],
               imm_i.instr[20], imm_i.instr[30:21], 1'b0};

    // export sign-extended versions
    imm_o.imm_i = sext32(imm_i32);
    imm_o.imm_s = sext32(imm_s32);
    imm_o.imm_b = sext32(imm_b32);

    // U-type: for RV64 this is typically sign-extended from bit31 after placement
    imm_o.imm_u = {{(XLEN-32){imm_u32[31]}}, imm_u32};

    imm_o.imm_j = sext32(imm_j32);

    // selected immediate (selection inside ImmGen)
    unique case (imm_i.imm_sel)
      IMM_I: imm_o.imm = imm_o.imm_i;
      IMM_S: imm_o.imm = imm_o.imm_s;
      IMM_B: imm_o.imm = imm_o.imm_b;
      IMM_U: imm_o.imm = imm_o.imm_u;
      IMM_J: imm_o.imm = imm_o.imm_j;
      default: imm_o.imm = imm_o.imm_i;
    endcase
  end

endmodule
