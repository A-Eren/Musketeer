// rtl/LSU.sv
module LSU #(
  parameter int XLEN = 32
) (
  // from core
  input  logic         is_load,
  input  logic         is_store,
  input  logic [2:0]   funct3,
  input  logic [XLEN-1:0] addr,
  input  logic [XLEN-1:0] store_data,

  // memory interface (simple)
  output logic [XLEN-1:0] mem_addr,   // aligned address
  output logic [31:0]     mem_wdata,
  output logic [3:0]      mem_wstrb,  // byte enables
  output logic            mem_we,
  output logic            mem_re,
  input  logic [31:0]     mem_rdata,

  // back to core
  output logic [XLEN-1:0] load_data,
  output logic            misaligned
);

  import riscv_pkg::*;

  logic [1:0]  byte_off;
  logic [31:0] rword;

  logic [7:0]  rbyte;
  logic [15:0] rhalf;

  always_comb begin
    byte_off  = addr[1:0];
    mem_addr  = {addr[XLEN-1:2], 2'b00}; // align down
    rword     = mem_rdata;

    // defaults
    mem_wdata   = 32'h0;
    mem_wstrb   = 4'b0000;
    mem_we      = 1'b0;
    mem_re      = 1'b0;
    load_data   = '0;
    misaligned  = 1'b0;

    // ---------------------------
    // misalignment checks
    // ---------------------------
    if (is_load) begin
      unique case (funct3)
        LSU_LH, LSU_LHU: misaligned = byte_off[0];        // halfword needs addr[0]=0
        LSU_LW:         misaligned = (byte_off != 2'b00); // word needs addr[1:0]=00
        default:       misaligned = 1'b0;                // LB/LBU are always aligned
      endcase
    end
    else if (is_store) begin
      unique case (funct3)
        LSU_SH: misaligned = byte_off[0];
        LSU_SW: misaligned = (byte_off != 2'b00);
        default: misaligned = 1'b0; // SB always aligned
      endcase
    end

    // ---------------------------
    // LOAD path
    // ---------------------------
    if (is_load && !misaligned) begin
      mem_re = 1'b1;

      // select byte/half based on little-endian offset
      rbyte = (rword >> (8 * byte_off)) & 8'hFF;
      rhalf = (rword >> (8 * byte_off)) & 16'hFFFF;

      unique case (funct3)
        LSU_LB:  load_data = {{(XLEN-8){rbyte[7]}}, rbyte};
        LSU_LBU: load_data = {{(XLEN-8){1'b0}},     rbyte};
        LSU_LH:  load_data = {{(XLEN-16){rhalf[15]}}, rhalf};
        LSU_LHU: load_data = {{(XLEN-16){1'b0}},      rhalf};
        LSU_LW:  load_data = {{(XLEN-32){1'b0}},      rword}; // XLEN=32 => pass through
        default: load_data = '0;
      endcase
    end

    // ---------------------------
    // STORE path
    // ---------------------------
    if (is_store && !misaligned) begin
      mem_we = 1'b1;

      unique case (funct3)
        LSU_SB: begin
          mem_wstrb = (4'b0001 << byte_off);
          mem_wdata = ( {24'h0, store_data[7:0]} << (8 * byte_off) );
        end

        LSU_SH: begin
          // byte_off is 00 or 10 when aligned for halfword
          mem_wstrb = (4'b0011 << byte_off);
          mem_wdata = ( {16'h0, store_data[15:0]} << (8 * byte_off) );
        end

        LSU_SW: begin
          mem_wstrb = 4'b1111;
          mem_wdata = store_data[31:0];
        end

        default: begin
          mem_wstrb = 4'b0000;
          mem_wdata = 32'h0;
        end
      endcase
    end
  end

endmodule
