// rtl/Riscv_Core.sv
// Single-cycle RV32I core using packetized submodules (Decoder/ImmGen/ALU/BranchUnit/LSU)
// and scalar NextPC + scalar RegisterFile (non-forwarding).
//
// NOTE: XLEN must match riscv_pkg::XLEN (currently 32).

module Riscv_Core #(
  parameter int XLEN = 32
) (
  input  logic             clk,
  input  logic             rst_n,

  // -------------------------
  // Instruction memory
  // -------------------------
  output logic [XLEN-1:0]   imem_addr,
  input  logic [31:0]       imem_rdata,

  // -------------------------
  // Data memory
  // -------------------------
  output logic [XLEN-1:0]   dmem_addr,
  output logic [31:0]       dmem_wdata,
  output logic [3:0]        dmem_wstrb,
  output logic              dmem_we,
  output logic              dmem_re,
  input  logic [31:0]       dmem_rdata,

  // -------------------------
  // Debug
  // -------------------------
  output logic [XLEN-1:0]   dbg_pc,
  output logic              dbg_illegal,
  output logic              dbg_misaligned
);

  import riscv_pkg::*;

  // -------------------------
  // PC + instruction fetch
  // -------------------------
  logic [XLEN-1:0] pc, pc_next, pc_plus4;
  logic [31:0]     instr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pc <= '0;
    else        pc <= pc_next;
  end

  assign imem_addr = pc;
  assign instr     = imem_rdata;

  assign dbg_pc = pc;

  // -------------------------
  // Decoder
  // -------------------------
  dec_in_t  dec_in;
  dec_out_t dec_out;

  assign dec_in.instr = instr;

  Decoder u_dec (
    .dec_i(dec_in),
    .dec_o(dec_out)
  );

  assign dbg_illegal = dec_out.illegal;

  // -------------------------
  // Immediate generator (selection inside ImmGen)
  // -------------------------
  imm_in_t  imm_in;
  imm_out_t imm_out;

  assign imm_in.instr   = instr;
  assign imm_in.imm_sel = dec_out.imm_sel;

  ImmGen u_imm (
    .imm_i(imm_in),
    .imm_o(imm_out)
  );

  // -------------------------
  // Register file (non-forwarding)
  // -------------------------
  logic [XLEN-1:0] rs1_val, rs2_val;
  logic [XLEN-1:0] wb_data;
  logic            reg_write_eff;

  RegisterFile #(.XLEN(XLEN)) u_rf (
    .clk   (clk),
    .rst_n (rst_n),

    .raddr1(dec_out.rs1),
    .raddr2(dec_out.rs2),
    .rdata1(rs1_val),
    .rdata2(rs2_val),

    .we    (reg_write_eff),
    .waddr (dec_out.rd),
    .wdata (wb_data)
  );

  // -------------------------
  // ALU operand muxing + ALU
  // -------------------------
  logic [XLEN-1:0] op_a, op_b;

  always_comb begin
    op_a = (dec_out.src_a_sel == SRC_A_PC)   ? pc          : rs1_val;
    op_b = (dec_out.src_b_sel == SRC_B_IMM)  ? imm_out.imm : rs2_val;
  end

  alu_in_t  alu_in;
  alu_out_t alu_out;

  assign alu_in.a  = op_a;
  assign alu_in.b  = op_b;
  assign alu_in.op = dec_out.alu_op;

  ALU u_alu (
    .alu_i(alu_in),
    .alu_o(alu_out)
  );

  // -------------------------
  // Branch decision
  // -------------------------
  branch_in_t  br_in;
  branch_out_t br_out;

  assign br_in.op  = dec_out.illegal ? BR_NONE : dec_out.br_op;
  assign br_in.alu = alu_out;

  BranchUnit u_bu (
    .br_i(br_in),
    .br_o(br_out)
  );

  // -------------------------
  // LSU (data memory)
  // Address comes from ALU result.
  // -------------------------
  lsu_in_t  lsu_in;
  lsu_out_t lsu_out;

  assign lsu_in.op         = dec_out.illegal ? LSU_NONE : dec_out.lsu_op;
  assign lsu_in.addr       = alu_out.y;
  assign lsu_in.store_data = rs2_val;
  assign lsu_in.mem_rdata  = dmem_rdata;

  LSU u_lsu (
    .lsu_i(lsu_in),
    .lsu_o(lsu_out)
  );

  // expose LSU -> memory
  assign dmem_addr  = lsu_out.mem_addr;
  assign dmem_wdata = lsu_out.mem_wdata[31:0];
  assign dmem_wstrb = lsu_out.mem_wstrb;
  assign dmem_we    = lsu_out.mem_we;
  assign dmem_re    = lsu_out.mem_re;

  assign dbg_misaligned = lsu_out.misaligned;

  // Gate writes on illegal or misaligned/illegal LSU response (loads)
  // (Stores already have reg_write=0 from decoder)
  assign reg_write_eff = dec_out.reg_write
                       & ~dec_out.illegal
                       & ~lsu_out.misaligned
                       & ~lsu_out.illegal;

  // -------------------------
  // Next PC (scalar NextPC module in your repo)
  // -------------------------
  NextPC u_npc (
    .pc          (pc),
    .imm_b       (imm_out.imm_b),
    .imm_j       (imm_out.imm_j),
    .imm_i       (imm_out.imm_i),
    .rs1_val     (rs1_val),

    .is_jal      (dec_out.is_jal  & ~dec_out.illegal),
    .is_jalr     (dec_out.is_jalr & ~dec_out.illegal),
    .take_branch (br_out.take     & ~dec_out.illegal),

    .pc_plus4    (pc_plus4),
    .pc_next     (pc_next)
  );

  // -------------------------
  // Writeback mux
  // -------------------------
  always_comb begin
    unique case (dec_out.wb_sel)
      WB_ALU: wb_data = alu_out.y;
      WB_MEM: wb_data = lsu_out.load_data;
      WB_PC4: wb_data = pc_plus4;
      default: wb_data = alu_out.y;
    endcase
  end

endmodule
