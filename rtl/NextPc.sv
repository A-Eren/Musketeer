// rtl/NextPc.sv
module NextPC (
  input  riscv_pkg::npc_in_t  npc_i,
  output riscv_pkg::npc_out_t npc_o
);

  import riscv_pkg::*;

  logic [XLEN-1:0] pc4;
  logic [XLEN-1:0] br_target;
  logic [XLEN-1:0] jal_target;
  logic [XLEN-1:0] jalr_sum;
  logic [XLEN-1:0] jalr_target;

  always_comb begin
    pc4        = npc_i.pc + XLEN'(32'd4);
    br_target  = npc_i.pc + npc_i.imm_b;
    jal_target = npc_i.pc + npc_i.imm_j;

    jalr_sum    = npc_i.rs1_val + npc_i.imm_i;
    jalr_target = {jalr_sum[XLEN-1:1], 1'b0}; // JALR clears bit0

    npc_o.pc_plus4 = pc4;

    // Priority: JALR > JAL > BRANCH > PC+4
    npc_o.sel     = NPC_PC4;
    npc_o.pc_next = pc4;

    if (npc_i.br_taken) begin
      npc_o.sel     = NPC_BRANCH;
      npc_o.pc_next = br_target;
    end

    if (npc_i.is_jal) begin
      npc_o.sel     = NPC_JAL;
      npc_o.pc_next = jal_target;
    end

    if (npc_i.is_jalr) begin
      npc_o.sel     = NPC_JALR;
      npc_o.pc_next = jalr_target;
    end
  end

endmodule
