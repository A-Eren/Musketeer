// rtl/Branch_Unit.sv
module BranchUnit (
  input  riscv_pkg::branch_in_t  br_i,
  output riscv_pkg::branch_out_t br_o
);

  import riscv_pkg::*;

  always_comb begin
    br_o.take = 1'b0;

    unique case (br_i.op)
      BR_BEQ:  br_o.take =  br_i.alu.zero;
      BR_BNE:  br_o.take = ~br_i.alu.zero;

      BR_BLT:  br_o.take =  br_i.alu.lt_signed;
      BR_BGE:  br_o.take = ~br_i.alu.lt_signed;   // >= signed

      BR_BLTU: br_o.take =  br_i.alu.lt_unsigned;
      BR_BGEU: br_o.take = ~br_i.alu.lt_unsigned; // >= unsigned

      default: br_o.take = 1'b0;
    endcase
  end

endmodule
