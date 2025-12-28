// rtl/RiscV_Core.sv
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

  // optional debug
  output logic [XLEN-1:0]   dbg_pc,
  output logic              dbg_illegal,
  output logic              dbg_misaligned
);

  import riscv_pkg::*;

  // -------------------------
  // PC + instruction
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
  // Decode outputs
  // -------------------------
  logic [6:0]   opcode, funct7;
  logic [2:0]   funct3;
  logic [4:0]   rd, rs1, rs2;

  imm_sel_t     imm_sel;
  alu_op_t      alu_op;
  src_a_sel_t   src_a_sel;
  src_b_sel_t   src_b_sel;
  wb_sel_t      wb_sel;

  logic         is_load, is_store;
  logic         is_branch, is_jal, is_jalr;
  logic         reg_write;
  logic         uses_rs1, uses_rs2;
  logic         illegal;

  Decoder u_dec (
    .instr(instr),

    .opcode(opcode),
    .rd(rd),
    .funct3(funct3),
    .rs1(rs1),
    .rs2(rs2),
    .funct7(funct7),

    .imm_sel(imm_sel),

    .alu_op(alu_op),
    .src_a_sel(src_a_sel),
    .src_b_sel(src_b_sel),

    .is_load(is_load),
    .is_store(is_store),

    .is_branch(is_branch),
    .is_jal(is_jal),
    .is_jalr(is_jalr),

    .reg_write(reg_write),
    .wb_sel(wb_sel),

    .uses_rs1(uses_rs1),
    .uses_rs2(uses_rs2),

    .illegal(illegal)
  );

  assign dbg_illegal = illegal;

  // -------------------------
  // Immediates
  // -------------------------
  logic [31:0] imm, imm_i, imm_s, imm_b, imm_u, imm_j;
  imm_sel_t    imm_sel_unused; // ImmGen also outputs imm_sel; we ignore it here.

  ImmGen u_imm (
    .instr(instr),
    .imm_sel(imm_sel_unused),
    .imm(imm),
    .imm_i(imm_i),
    .imm_s(imm_s),
    .imm_b(imm_b),
    .imm_u(imm_u),
    .imm_j(imm_j)
  );

  logic [31:0] imm_selected;
  always_comb begin
    unique case (imm_sel)
      IMM_I: imm_selected = imm_i;
      IMM_S: imm_selected = imm_s;
      IMM_B: imm_selected = imm_b;
      IMM_U: imm_selected = imm_u;
      IMM_J: imm_selected = imm_j;
      default: imm_selected = imm_i;
    endcase
  end

  // -------------------------
  // Register file
  // -------------------------
  logic [XLEN-1:0] rs1_val, rs2_val;
  logic [XLEN-1:0] wb_data;
  logic            reg_write_eff;

  // Gate reg write if illegal; also if misaligned load (optional but sensible)
  // (Stores already have reg_write=0)
  // misaligned is produced later by LSU, so declare now:
  logic lsu_misaligned;

  assign reg_write_eff = reg_write & ~illegal & ~(is_load & lsu_misaligned);

  RegisterFile #(.XLEN(XLEN)) u_rf (
    .clk_i(clk),
    .rst_ni(rst_n),
    .raddr1(rs1),
    .raddr2(rs2),
    .rdata1(rs1_val),
    .rdata2(rs2_val),

    .we(reg_write_eff),
    .waddr(rd),
    .wdata(wb_data)
  );

  // -------------------------
  // ALU operand muxing
  // -------------------------
  logic [XLEN-1:0] alu_a, alu_b, alu_y;
  logic            alu_zero, alu_lt_s, alu_lt_u;

  always_comb begin
    alu_a = (src_a_sel == SRC_A_PC)  ? pc : rs1_val;
    alu_b = (src_b_sel == SRC_B_IMM) ? imm_selected : rs2_val;
  end

  ALU #(.XLEN(XLEN)) u_alu (
    .a(alu_a),
    .b(alu_b),
    .op(alu_op),
    .y(alu_y),
    .zero(alu_zero),
    .lt_signed(alu_lt_s),
    .lt_unsigned(alu_lt_u)
  );

  // -------------------------
  // LSU (data memory)
  // address comes from ALU result
  // -------------------------
  logic [XLEN-1:0] load_data;

  LSU #(.XLEN(XLEN)) u_lsu (
    .is_load(is_load),
    .is_store(is_store),
    .funct3(funct3),
    .addr(alu_y),
    .store_data(rs2_val),

    .mem_addr(dmem_addr),
    .mem_wdata(dmem_wdata),
    .mem_wstrb(dmem_wstrb),
    .mem_we(dmem_we),
    .mem_re(dmem_re),
    .mem_rdata(dmem_rdata),

    .load_data(load_data),
    .misaligned(lsu_misaligned)
  );

  assign dbg_misaligned = lsu_misaligned;

  // -------------------------
  // Branch decision
  // -------------------------
  logic take_branch;

  BranchUnit u_bu (
    .is_branch(is_branch),
    .funct3(funct3),
    .zero(alu_zero),
    .lt_signed(alu_lt_s),
    .lt_unsigned(alu_lt_u),
    .take_branch(take_branch)
  );

  // -------------------------
  // Next PC
  // -------------------------
  NextPC u_npc (
    .pc(pc),
    .imm_b(imm_b),
    .imm_j(imm_j),
    .imm_i(imm_i),
    .rs1_val(rs1_val),
    .is_jal(is_jal),
    .is_jalr(is_jalr),
    .take_branch(take_branch),
    .pc_plus4(pc_plus4),
    .pc_next(pc_next)
  );

  // -------------------------
  // Writeback mux
  // -------------------------
  always_comb begin
    unique case (wb_sel)
      WB_ALU: wb_data = alu_y;
      WB_MEM: wb_data = load_data;
      WB_PC4: wb_data = pc_plus4;
      default: wb_data = alu_y;
    endcase
  end

endmodule
