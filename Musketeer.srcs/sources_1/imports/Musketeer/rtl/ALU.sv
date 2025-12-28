// rtl/ALU.sv
module ALU #(
  parameter int XLEN = 32
) (
  input  logic [XLEN-1:0] a,
  input  logic [XLEN-1:0] b,
  input  riscv_pkg::alu_op_t op,

  output logic [XLEN-1:0] y,

  // handy flags (branches, etc.)
  output logic            zero,
  output logic            lt_signed,
  output logic            lt_unsigned
);

  import riscv_pkg::*;

  logic signed [XLEN-1:0] a_s, b_s;
  logic [4:0] shamt;

  always_comb begin
    a_s  = $signed(a);
    b_s  = $signed(b);
    shamt = b[4:0];

    y = '0;

    unique case (op)
      ALU_ADD:    y = a + b;
      ALU_SUB:    y = a - b;

      ALU_AND:    y = a & b;
      ALU_OR:     y = a | b;
      ALU_XOR:    y = a ^ b;

      ALU_SLL:    y = a << shamt;
      ALU_SRL:    y = a >> shamt;
      ALU_SRA:    y = $signed(a_s >>> shamt);

      ALU_SLT:    y = (a_s < b_s) ? {{(XLEN-1){1'b0}},1'b1} : '0;
      ALU_SLTU:   y = (a   < b  ) ? {{(XLEN-1){1'b0}},1'b1} : '0;

      ALU_COPY_A: y = a;
      ALU_COPY_B: y = b;

      default:    y = '0;
    endcase

    zero        = (y == '0);
    lt_signed   = (a_s < b_s);
    lt_unsigned = (a < b);
  end

endmodule
