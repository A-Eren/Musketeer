// rtl/RegisterFile.sv
module RegisterFile #(
  parameter int XLEN = 32
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic  [4:0]   raddr1,
  input  logic  [4:0]   raddr2,
  output logic [XLEN-1:0] rdata1,
  output logic [XLEN-1:0] rdata2,

  input  logic         we,
  input  logic  [4:0]  waddr,
  input  logic [XLEN-1:0] wdata
);

  logic [XLEN-1:0] regs [31:0];

  // sync write, x0 is hardwired to 0 by ignoring writes to address 0
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if( !rst_ni ) begin
      integer i;
      for (i = 0; i < 32; i=i+1) begin
        regs[i] <= '0;
      end
    end
    else
    if (we && (waddr != 5'd0)) begin
      regs[waddr] <= wdata;
    end
    regs[5'd0] <= '0; // keep x0 clean in simulation/synthesis
  end

  // async reads + simple forwarding
  always_comb begin
    logic [XLEN-1:0] raw1, raw2;

    raw1 = (raddr1 == 5'd0) ? '0 : regs[raddr1];
    raw2 = (raddr2 == 5'd0) ? '0 : regs[raddr2];

    // write-first behavior (forwarding)
    //if (we && (waddr != 5'd0) && (waddr == raddr1)) rdata1 = wdata;
    //else
                                                 rdata1 = raw1;

    //if (we && (waddr != 5'd0) && (waddr == raddr2)) rdata2 = wdata;
    //else
                                                 rdata2 = raw2;
  end

endmodule
