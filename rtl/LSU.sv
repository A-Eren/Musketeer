// rtl/LSU.sv
module LSU (
  input  riscv_pkg::lsu_in_t  lsu_i,
  output riscv_pkg::lsu_out_t lsu_o
);

  import riscv_pkg::*;

  logic [1:0]        byte_off;
  logic [XLEN-1:0]   rshifted;
  logic [7:0]        rbyte;
  logic [15:0]       rhalf;

  always_comb begin
    byte_off = lsu_i.addr[1:0];

    // defaults
    lsu_o.mem_re     = 1'b0;
    lsu_o.mem_we     = 1'b0;
    lsu_o.mem_addr   = lsu_i.addr;
    lsu_o.mem_wdata  = '0;
    lsu_o.mem_wstrb  = '0;

    lsu_o.load_data  = '0;
    lsu_o.misaligned = 1'b0;
    lsu_o.illegal    = 1'b0;

    // read helpers
    rshifted = (lsu_i.mem_rdata >> (8*byte_off));
    rbyte    = rshifted[7:0];
    rhalf    = rshifted[15:0];

    unique case (lsu_i.op)

      // -------------------------
      // Loads
      // -------------------------
      LSU_LB: begin
        lsu_o.mem_re    = 1'b1;
        lsu_o.load_data = {{(XLEN-8){rbyte[7]}}, rbyte};
      end

      LSU_LBU: begin
        lsu_o.mem_re    = 1'b1;
        lsu_o.load_data = {{(XLEN-8){1'b0}}, rbyte};
      end

      LSU_LH: begin
        lsu_o.misaligned = (byte_off[0] != 1'b0);
        lsu_o.mem_re     = ~lsu_o.misaligned;
        lsu_o.load_data  = lsu_o.misaligned ? '0 : {{(XLEN-16){rhalf[15]}}, rhalf};
      end

      LSU_LHU: begin
        lsu_o.misaligned = (byte_off[0] != 1'b0);
        lsu_o.mem_re     = ~lsu_o.misaligned;
        lsu_o.load_data  = lsu_o.misaligned ? '0 : {{(XLEN-16){1'b0}}, rhalf};
      end

      LSU_LW: begin
        lsu_o.misaligned = (byte_off != 2'b00);
        lsu_o.mem_re     = ~lsu_o.misaligned;
        lsu_o.load_data  = lsu_o.misaligned ? '0 : lsu_i.mem_rdata;
      end

      // -------------------------
      // Stores
      // -------------------------
LSU_SB: begin
  lsu_o.mem_wstrb = logic [XLEN/8-1:0]'(4'b0001 << byte_off);
  lsu_o.mem_wdata = (lsu_i.store_data & 32'h000000FF) << (8*byte_off);
  lsu_o.mem_we    = (lsu_o.mem_wstrb != '0);
end

LSU_SH: begin
  lsu_o.misaligned = (byte_off[0] != 1'b0);
  lsu_o.mem_wstrb  = lsu_o.misaligned ? '0
                                      : logic [XLEN/8-1:0]'(4'b0011 << byte_off);
  lsu_o.mem_wdata  = lsu_o.misaligned ? '0
                                      : ((lsu_i.store_data & 32'h0000FFFF) << (8*byte_off));
  lsu_o.mem_we     = (!lsu_o.misaligned) && (lsu_o.mem_wstrb != '0);
end

LSU_SW: begin
  lsu_o.misaligned = (byte_off != 2'b00);
  lsu_o.mem_wstrb  = lsu_o.misaligned ? '0 : {(XLEN/8){1'b1}}; // 4'b1111 when XLEN=32
  lsu_o.mem_wdata  = lsu_o.misaligned ? '0 : lsu_i.store_data;
  lsu_o.mem_we     = (!lsu_o.misaligned) && (lsu_o.mem_wstrb != '0);
end

      // -------------------------
      // No memory op
      // -------------------------
      LSU_NONE: begin
        // nothing
      end

      default: begin
        lsu_o.illegal = 1'b1;
      end

    endcase
  end

endmodule
