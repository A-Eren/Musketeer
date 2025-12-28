// rtl/NextPC.sv
module NextPC (
  input  logic [31:0] pc,

  // immediates (from ImmGen)
  input  logic [31:0] imm_b,
  input  logic [31:0] imm_j,
  input  logic [31:0] imm_i,

  // rs1 value for JALR base
  input  logic [31:0] rs1_val,

  // control
  input  logic        is_jal,
  input  logic        is_jalr,
  input  logic        take_branch,

  // outputs
  output logic [31:0] pc_plus4,
  output logic [31:0] pc_next
);

  logic [31:0] branch_target, jal_target, jalr_target;

  always_comb begin
    pc_plus4      = pc + 32'd4;
    branch_target = pc + imm_b;
    jal_target    = pc + imm_j;

    // JALR target = (rs1 + imm_i) & ~1
    jalr_target   = (rs1_val + imm_i) & 32'hFFFF_FFFE;

    // priority select
    if (is_jal)         pc_next = jal_target;
    else if (is_jalr)   pc_next = jalr_target;
    else if (take_branch) pc_next = branch_target;
    else                pc_next = pc_plus4;
  end

endmodule
