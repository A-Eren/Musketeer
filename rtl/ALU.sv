// rtl/ALU.sv
module ALU (
  input  riscv_pkg::alu_in_t  alu_i,
  output riscv_pkg::alu_out_t alu_o
);

  import riscv_pkg::*;

  logic signed [XLEN-1:0] a_s, b_s;
  logic [4:0]             shamt;

  always_comb begin
    a_s   = $signed(alu_i.a);
    b_s   = $signed(alu_i.b);
    shamt = alu_i.b[4:0];

    alu_o.y = '0;

    unique case (alu_i.op)
      ALU_ADD:    alu_o.y = alu_i.a + alu_i.b;
      ALU_SUB:    alu_o.y = alu_i.a - alu_i.b;

      ALU_AND:    alu_o.y = alu_i.a & alu_i.b;
      ALU_OR:     alu_o.y = alu_i.a | alu_i.b;
      ALU_XOR:    alu_o.y = alu_i.a ^ alu_i.b;

      ALU_SLL:    alu_o.y = alu_i.a << shamt;
      ALU_SRL:    alu_o.y = alu_i.a >> shamt;
      ALU_SRA:    alu_o.y = $signed(a_s >>> shamt);

      ALU_SLT:    alu_o.y = (a_s < b_s) ? {{(XLEN-1){1'b0}}, 1'b1} : '0;
      ALU_SLTU:   alu_o.y = (alu_i.a < alu_i.b) ? {{(XLEN-1){1'b0}}, 1'b1} : '0;

      ALU_COPY_A: alu_o.y = alu_i.a;
      ALU_COPY_B: alu_o.y = alu_i.b;

      default:    alu_o.y = '0;
    endcase

    alu_o.zero        = (alu_o.y == '0);
    alu_o.lt_signed   = (a_s < b_s);
    alu_o.lt_unsigned = (alu_i.a < alu_i.b);
  end

endmodule
