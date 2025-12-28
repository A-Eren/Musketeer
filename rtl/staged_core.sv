// rtl/staged_core.sv
module staged_core #(
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

  localparam logic [31:0] INSTR_NOP = 32'h0000_0013; // ADDI x0, x0, 0

  localparam if_id_t  IF_ID_RESET = '{pc: '0, instr: INSTR_NOP};
  localparam if_id_t  IF_ID_FLUSH = '{pc: '0, instr: INSTR_NOP};
  localparam id_ex_t  ID_EX_BUBBLE = '0;
  localparam ex_mem_t EX_MEM_RESET = '0;
  localparam mem_wb_t MEM_WB_RESET = '0;

  // -------------------------
  // IF stage
  // -------------------------
  logic [XLEN-1:0] pc_q, pc_next, pc_plus4_if;
  logic            stall_fetch;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pc_q <= '0;
    else if (!stall_fetch) pc_q <= pc_next;
  end

  assign pc_plus4_if = pc_q + {{(XLEN-3){1'b0}}, 3'd4};
  assign imem_addr   = pc_q;
  assign dbg_pc      = pc_q;

  if_id_t if_id_d, if_id_q;

  assign if_id_d.pc    = pc_q;
  assign if_id_d.instr = imem_rdata;

  // -------------------------
  // ID stage
  // -------------------------
  logic [6:0] opcode_id, funct7_id;
  logic [2:0] funct3_id;
  logic [4:0] rd_id, rs1_id, rs2_id;

  imm_sel_t    imm_sel_dec_unused;
  imm_sel_t    imm_sel_immgen_unused;
  alu_op_t     alu_op_id;
  src_a_sel_t  src_a_sel_id;
  src_b_sel_t  src_b_sel_id;
  wb_sel_t     wb_sel_id;

  logic        is_load_id, is_store_id;
  logic        is_branch_id, is_jal_id, is_jalr_id;
  logic        reg_write_id;
  logic        uses_rs1_id, uses_rs2_id;
  logic        uses_rs1_eff, uses_rs2_eff;
  logic        illegal_id;

  Decoder u_dec (
    .instr(if_id_q.instr),
    .opcode(opcode_id),
    .rd(rd_id),
    .funct3(funct3_id),
    .rs1(rs1_id),
    .rs2(rs2_id),
    .funct7(funct7_id),
    .imm_sel(imm_sel_dec_unused),
    .alu_op(alu_op_id),
    .src_a_sel(src_a_sel_id),
    .src_b_sel(src_b_sel_id),
    .is_load(is_load_id),
    .is_store(is_store_id),
    .is_branch(is_branch_id),
    .is_jal(is_jal_id),
    .is_jalr(is_jalr_id),
    .reg_write(reg_write_id),
    .wb_sel(wb_sel_id),
    .uses_rs1(uses_rs1_id),
    .uses_rs2(uses_rs2_id),
    .illegal(illegal_id)
  );

  assign dbg_illegal = illegal_id;

  logic [31:0] imm_id, imm_i_id, imm_s_id, imm_b_id, imm_u_id, imm_j_id;

  ImmGen u_imm (
    .instr(if_id_q.instr),
    .imm_sel(imm_sel_immgen_unused),
    .imm(imm_id),
    .imm_i(imm_i_id),
    .imm_s(imm_s_id),
    .imm_b(imm_b_id),
    .imm_u(imm_u_id),
    .imm_j(imm_j_id)
  );

  logic [XLEN-1:0] rs1_val_id, rs2_val_id;
  logic [XLEN-1:0] wb_data;
  logic            reg_write_wb;

  RegisterFile #(.XLEN(XLEN)) u_rf (
    .clk_i(clk),
    .rst_ni(rst_n),
    .raddr1(rs1_id),
    .raddr2(rs2_id),
    .rdata1(rs1_val_id),
    .rdata2(rs2_val_id),
    .we(reg_write_wb),
    .waddr(mem_wb_q.rd),
    .wdata(wb_data)
  );

  // -------------------------
  // ID/EX pipeline
  // -------------------------
  id_ex_t id_ex_d, id_ex_q;

  logic kill_id;
  assign kill_id = illegal_id;
  assign uses_rs1_eff = uses_rs1_id & ~kill_id;
  assign uses_rs2_eff = uses_rs2_id & ~kill_id;

  assign id_ex_d.pc        = if_id_q.pc;
  assign id_ex_d.rs1_val   = rs1_val_id;
  assign id_ex_d.rs2_val   = rs2_val_id;
  assign id_ex_d.rs1       = rs1_id;
  assign id_ex_d.rs2       = rs2_id;
  assign id_ex_d.rd        = rd_id;
  assign id_ex_d.funct3    = funct3_id;
  assign id_ex_d.imm       = imm_id;
  assign id_ex_d.imm_i     = imm_i_id;
  assign id_ex_d.imm_b     = imm_b_id;
  assign id_ex_d.imm_j     = imm_j_id;
  assign id_ex_d.alu_op    = alu_op_id;
  assign id_ex_d.src_a_sel = src_a_sel_id;
  assign id_ex_d.src_b_sel = src_b_sel_id;
  assign id_ex_d.wb_sel    = wb_sel_id;
  assign id_ex_d.is_load   = is_load_id  & ~kill_id;
  assign id_ex_d.is_store  = is_store_id & ~kill_id;
  assign id_ex_d.is_branch = is_branch_id & ~kill_id;
  assign id_ex_d.is_jal    = is_jal_id & ~kill_id;
  assign id_ex_d.is_jalr   = is_jalr_id & ~kill_id;
  assign id_ex_d.reg_write = reg_write_id & ~kill_id;

  // -------------------------
  // Load-use hazard detection
  // -------------------------
  logic load_use_hazard;

  assign load_use_hazard =
    id_ex_q.is_load &&
    (id_ex_q.rd != 5'd0) &&
    ((uses_rs1_eff && (rs1_id == id_ex_q.rd)) ||
     (uses_rs2_eff && (rs2_id == id_ex_q.rd)));

  assign stall_fetch = load_use_hazard;

  // -------------------------
  // EX stage
  // -------------------------
  logic [XLEN-1:0] ex_rs1_val, ex_rs2_val;
  logic [XLEN-1:0] alu_a, alu_b, alu_y;
  logic            alu_zero, alu_lt_s, alu_lt_u;
  alu_pkg_t        alu_pkt;

  ex_mem_t ex_mem_d, ex_mem_q;
  mem_wb_t mem_wb_q;

  logic [XLEN-1:0] exmem_wb_value;
  logic [XLEN-1:0] memwb_wb_value;

  always_comb begin
    unique case (ex_mem_q.wb_sel)
      WB_ALU: exmem_wb_value = ex_mem_q.alu_y;
      WB_PC4: exmem_wb_value = ex_mem_q.pc_plus4;
      default: exmem_wb_value = ex_mem_q.alu_y;
    endcase
  end

  // Forwarding for EX operands
  always_comb begin
    ex_rs1_val = id_ex_q.rs1_val;
    if (ex_mem_q.reg_write && (ex_mem_q.rd != 5'd0) &&
        (ex_mem_q.rd == id_ex_q.rs1) && (ex_mem_q.wb_sel != WB_MEM)) begin
      ex_rs1_val = exmem_wb_value;
    end
    else if (mem_wb_q.reg_write && (mem_wb_q.rd != 5'd0) &&
             (mem_wb_q.rd == id_ex_q.rs1)) begin
      ex_rs1_val = memwb_wb_value;
    end

    ex_rs2_val = id_ex_q.rs2_val;
    if (ex_mem_q.reg_write && (ex_mem_q.rd != 5'd0) &&
        (ex_mem_q.rd == id_ex_q.rs2) && (ex_mem_q.wb_sel != WB_MEM)) begin
      ex_rs2_val = exmem_wb_value;
    end
    else if (mem_wb_q.reg_write && (mem_wb_q.rd != 5'd0) &&
             (mem_wb_q.rd == id_ex_q.rs2)) begin
      ex_rs2_val = memwb_wb_value;
    end
  end

  always_comb begin
    alu_a = (id_ex_q.src_a_sel == SRC_A_PC)  ? id_ex_q.pc : ex_rs1_val;
    alu_b = (id_ex_q.src_b_sel == SRC_B_IMM) ? id_ex_q.imm : ex_rs2_val;
  end

  assign alu_pkt.a  = alu_a;
  assign alu_pkt.b  = alu_b;
  assign alu_pkt.op = id_ex_q.alu_op;

  ALU #(.XLEN(XLEN)) u_alu (
    .alu_i(alu_pkt),
    .y(alu_y),
    .zero(alu_zero),
    .lt_signed(alu_lt_s),
    .lt_unsigned(alu_lt_u)
  );

  logic take_branch_ex;

  BranchUnit u_bu (
    .is_branch(id_ex_q.is_branch),
    .funct3(id_ex_q.funct3),
    .zero(alu_zero),
    .lt_signed(alu_lt_s),
    .lt_unsigned(alu_lt_u),
    .take_branch(take_branch_ex)
  );

  logic [XLEN-1:0] pc_plus4_ex;
  logic [XLEN-1:0] pc_next_ex;

  NextPC u_npc (
    .pc(id_ex_q.pc),
    .imm_b(id_ex_q.imm_b),
    .imm_j(id_ex_q.imm_j),
    .imm_i(id_ex_q.imm_i),
    .rs1_val(ex_rs1_val),
    .is_jal(id_ex_q.is_jal),
    .is_jalr(id_ex_q.is_jalr),
    .take_branch(take_branch_ex),
    .pc_plus4(pc_plus4_ex),
    .pc_next(pc_next_ex)
  );

  logic pc_redirect_ex;
  assign pc_redirect_ex = id_ex_q.is_jal || id_ex_q.is_jalr || take_branch_ex;

  assign pc_next = pc_redirect_ex ? pc_next_ex : pc_plus4_if;

  // -------------------------
  // EX/MEM pipeline
  // -------------------------
  assign ex_mem_d.alu_y      = alu_y;
  assign ex_mem_d.store_data = ex_rs2_val;
  assign ex_mem_d.rd         = id_ex_q.rd;
  assign ex_mem_d.funct3     = id_ex_q.funct3;
  assign ex_mem_d.wb_sel     = id_ex_q.wb_sel;
  assign ex_mem_d.reg_write  = id_ex_q.reg_write;
  assign ex_mem_d.is_load    = id_ex_q.is_load;
  assign ex_mem_d.is_store   = id_ex_q.is_store;
  assign ex_mem_d.pc_plus4   = pc_plus4_ex;

  // -------------------------
  // MEM stage
  // -------------------------
  logic [XLEN-1:0] load_data_mem;
  logic            misaligned_mem;

  LSU #(.XLEN(XLEN)) u_lsu (
    .is_load(ex_mem_q.is_load),
    .is_store(ex_mem_q.is_store),
    .funct3(ex_mem_q.funct3),
    .addr(ex_mem_q.alu_y),
    .store_data(ex_mem_q.store_data),
    .mem_addr(dmem_addr),
    .mem_wdata(dmem_wdata),
    .mem_wstrb(dmem_wstrb),
    .mem_we(dmem_we),
    .mem_re(dmem_re),
    .mem_rdata(dmem_rdata),
    .load_data(load_data_mem),
    .misaligned(misaligned_mem)
  );

  assign dbg_misaligned = misaligned_mem;

  // -------------------------
  // MEM/WB pipeline
  // -------------------------
  mem_wb_t mem_wb_d;

  assign mem_wb_d.alu_y      = ex_mem_q.alu_y;
  assign mem_wb_d.load_data  = load_data_mem;
  assign mem_wb_d.rd         = ex_mem_q.rd;
  assign mem_wb_d.wb_sel     = ex_mem_q.wb_sel;
  assign mem_wb_d.reg_write  = ex_mem_q.reg_write;
  assign mem_wb_d.pc_plus4   = ex_mem_q.pc_plus4;
  assign mem_wb_d.is_load    = ex_mem_q.is_load;
  assign mem_wb_d.misaligned = misaligned_mem;

  // -------------------------
  // WB stage
  // -------------------------
  always_comb begin
    unique case (mem_wb_q.wb_sel)
      WB_ALU: wb_data = mem_wb_q.alu_y;
      WB_MEM: wb_data = mem_wb_q.load_data;
      WB_PC4: wb_data = mem_wb_q.pc_plus4;
      default: wb_data = mem_wb_q.alu_y;
    endcase
  end

  assign memwb_wb_value = wb_data;

  assign reg_write_wb =
    mem_wb_q.reg_write & ~(mem_wb_q.is_load & mem_wb_q.misaligned);

  // -------------------------
  // Pipeline registers
  // -------------------------
  Pipe_Registers #(
    .T(if_id_t),
    .RESET_VAL(IF_ID_RESET),
    .FLUSH_VAL(IF_ID_FLUSH)
  ) u_if_id (
    .clk(clk),
    .rst_n(rst_n),
    .stall(stall_fetch),
    .flush(pc_redirect_ex),
    .d(if_id_d),
    .q(if_id_q)
  );

  Pipe_Registers #(
    .T(id_ex_t),
    .RESET_VAL(ID_EX_BUBBLE),
    .FLUSH_VAL(ID_EX_BUBBLE)
  ) u_id_ex (
    .clk(clk),
    .rst_n(rst_n),
    .stall(1'b0),
    .flush(pc_redirect_ex | load_use_hazard),
    .d(id_ex_d),
    .q(id_ex_q)
  );

  Pipe_Registers #(
    .T(ex_mem_t),
    .RESET_VAL(EX_MEM_RESET),
    .FLUSH_VAL(EX_MEM_RESET)
  ) u_ex_mem (
    .clk(clk),
    .rst_n(rst_n),
    .stall(1'b0),
    .flush(1'b0),
    .d(ex_mem_d),
    .q(ex_mem_q)
  );

  Pipe_Registers #(
    .T(mem_wb_t),
    .RESET_VAL(MEM_WB_RESET),
    .FLUSH_VAL(MEM_WB_RESET)
  ) u_mem_wb (
    .clk(clk),
    .rst_n(rst_n),
    .stall(1'b0),
    .flush(1'b0),
    .d(mem_wb_d),
    .q(mem_wb_q)
  );

endmodule
