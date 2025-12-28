// rtl/RegisterFile.sv
module RegisterFile (
  input  logic            clk,
  input  logic            rst_n,

  input  riscv_pkg::rf_in_t  rf_i,
  output riscv_pkg::rf_out_t rf_o
);

  import riscv_pkg::*;

  logic [XLEN-1:0] regs [32];

  // Synchronous write, x0 hardwired to zero
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      integer i;
      for (i = 0; i < 32; i++) begin
        regs[i] <= '0;
      end
    end else begin
      if (rf_i.we && (rf_i.waddr != 5'd0)) begin
        regs[rf_i.waddr] <= rf_i.wdata;
      end
      regs[5'd0] <= '0;
    end
  end

  // Asynchronous reads (NON-forwarding: no bypass / no write-first)
  always_comb begin
    rf_o.rdata1 = (rf_i.raddr1 == 5'd0) ? '0 : regs[rf_i.raddr1];
    rf_o.rdata2 = (rf_i.raddr2 == 5'd0) ? '0 : regs[rf_i.raddr2];
  end

endmodule
