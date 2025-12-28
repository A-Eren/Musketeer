// rtl/BranchUnit.sv
module BranchUnit (
  input  logic        is_branch,
  input  logic [2:0]  funct3,

  input  logic        zero,
  input  logic        lt_signed,
  input  logic        lt_unsigned,

  output logic        take_branch
);

  import riscv_pkg::*;

  always_comb begin
    take_branch = 1'b0;

    if (is_branch) begin
      unique case (funct3)
        F3_BEQ:  take_branch =  zero;
        F3_BNE:  take_branch = ~zero;
        F3_BLT:  take_branch =  lt_signed;
        F3_BGE:  take_branch = ~lt_signed;   // >= signed
        F3_BLTU: take_branch =  lt_unsigned;
        F3_BGEU: take_branch = ~lt_unsigned; // >= unsigned
        default: take_branch = 1'b0;
      endcase
    end
  end

endmodule
