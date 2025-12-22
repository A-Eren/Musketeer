module ALU_tb;

  import riscv_pkg::*;

  localparam int W = 32;

  logic [W-1:0] a, b;
  logic [3:0]   op;
  logic [W-1:0] y;
  logic         zero;

  ALU #(.W(W)) dut (.a(a), .b(b), .op(op), .y(y), .zero(zero));

  task automatic expect(input logic [W-1:0] exp, input string msg);
    #1;
    if (y !== exp) begin
      ("FAIL: %s | got=%0d (0x%08h) exp=%0d (0x%08h)", msg, y, y, exp, exp);
      (1);
    end
  endtask

  initial begin
    a=10; b=3;

    op=ALU_OP_ADD; expect(13, "ADD 10+3");
    op=ALU_OP_SUB; expect(7,  "SUB 10-3");
    op=ALU_OP_AND; expect(10 & 3, "AND");
    op=ALU_OP_OR ; expect(10 | 3, "OR");
    op=ALU_OP_XOR; expect(10 ^ 3, "XOR");

    a = -5; b = 2; op=ALU_OP_SLT; expect(1, "SLT -5 < 2");

    ("PASS");
    ;
  end

endmodule
