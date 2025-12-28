package riscv_pkg;

  localparam int XLEN = 32;
  // -------------------------
  // NextPC selection enum + packets
  // -------------------------
  typedef enum logic [1:0] {
    NPC_PC4    = 2'd0,
    NPC_BRANCH = 2'd1,
    NPC_JAL    = 2'd2,
    NPC_JALR   = 2'd3
  } npc_sel_t;

  typedef struct packed {
    logic [XLEN-1:0] pc;

    // immediates (from ImmGen outputs)
    logic [XLEN-1:0] imm_i;
    logic [XLEN-1:0] imm_b;
    logic [XLEN-1:0] imm_j;

    // rs1 base for JALR
    logic [XLEN-1:0] rs1_val;

    // control (from decoder/branch unit)
    logic            is_jal;
    logic            is_jalr;
    logic            br_taken;  // from BranchUnit
  } npc_in_t;

  typedef struct packed {
    logic [XLEN-1:0] pc_plus4;
    logic [XLEN-1:0] pc_next;
    npc_sel_t        sel;       // debug/visibility
  } npc_out_t;

  // -------------------------
  // Register file packets
  // -------------------------
  typedef struct packed {
    logic [4:0]      raddr1;
    logic [4:0]      raddr2;

    logic            we;
    logic [4:0]      waddr;
    logic [XLEN-1:0] wdata;
  } rf_in_t;

  typedef struct packed {
    logic [XLEN-1:0] rdata1;
    logic [XLEN-1:0] rdata2;
  } rf_out_t;
  // -------------------------
  // Strongly-typed opcode enum (no localparam OPCODE_*)
  // -------------------------
  typedef enum logic [6:0] {
    OP_LUI      = 7'h37,
    OP_AUIPC    = 7'h17,
    OP_JAL      = 7'h6F,
    OP_JALR     = 7'h67,
    OP_BRANCH   = 7'h63,
    OP_LOAD     = 7'h03,
    OP_STORE    = 7'h23,
    OP_OP_IMM   = 7'h13,
    OP_OP       = 7'h33
  } opcode_t;

  // -------------------------
  // Immediate selection
  // -------------------------
  typedef enum logic [2:0] {
    IMM_I = 3'd0,
    IMM_S = 3'd1,
    IMM_B = 3'd2,
    IMM_U = 3'd3,
    IMM_J = 3'd4
  } imm_sel_t;

  // -------------------------
  // ALU op
  // -------------------------
  typedef enum logic [4:0] {
    ALU_ADD      = 5'd0,
    ALU_SUB      = 5'd1,
    ALU_AND      = 5'd2,
    ALU_OR       = 5'd3,
    ALU_XOR      = 5'd4,
    ALU_SLL      = 5'd5,
    ALU_SRL      = 5'd6,
    ALU_SRA      = 5'd7,
    ALU_SLT      = 5'd8,
    ALU_SLTU     = 5'd9,
    ALU_COPY_A   = 5'd10,
    ALU_COPY_B   = 5'd11
  } alu_op_t;

  typedef enum logic [0:0] { SRC_A_RS1 = 1'b0, SRC_A_PC  = 1'b1 } src_a_sel_t;
  typedef enum logic [0:0] { SRC_B_RS2 = 1'b0, SRC_B_IMM = 1'b1 } src_b_sel_t;

  typedef enum logic [1:0] { WB_ALU = 2'd0, WB_MEM = 2'd1, WB_PC4 = 2'd2 } wb_sel_t;

  // -------------------------
  // ALU packet types
  // -------------------------
  typedef struct packed {
    logic [XLEN-1:0] a;
    logic [XLEN-1:0] b;
    alu_op_t         op;
  } alu_in_t;

  typedef struct packed {
    logic [XLEN-1:0] y;
    logic            zero;
    logic            lt_signed;
    logic            lt_unsigned;
  } alu_out_t;

  // -------------------------
  // Branch op enum (canonicalized)
  // -------------------------
  typedef enum logic [2:0] {
    BR_NONE = 3'd0,
    BR_BEQ  = 3'd1,
    BR_BNE  = 3'd2,
    BR_BLT  = 3'd3,
    BR_BGE  = 3'd4,
    BR_BLTU = 3'd5,
    BR_BGEU = 3'd6
  } branch_op_t;

  function automatic logic br_is_branch(branch_op_t op);
    return (op != BR_NONE);
  endfunction

  // -------------------------
  // Branch packets
  // -------------------------
  typedef struct packed {
    branch_op_t op;
    alu_out_t   alu;
  } branch_in_t;

  typedef struct packed {
    logic take;
  } branch_out_t;

  // -------------------------
  // ImmGen packets
  // -------------------------
  typedef struct packed {
    logic [31:0] instr;
    imm_sel_t    imm_sel;
  } imm_in_t;

  typedef struct packed {
    logic [XLEN-1:0] imm;   // selected

    logic [XLEN-1:0] imm_i;
    logic [XLEN-1:0] imm_s;
    logic [XLEN-1:0] imm_b;
    logic [XLEN-1:0] imm_u;
    logic [XLEN-1:0] imm_j;
  } imm_out_t;

  // -------------------------
  // LSU op enum (canonicalized)
  // -------------------------
  typedef enum logic [3:0] {
    LSU_NONE = 4'd0,

    LSU_LB   = 4'd1,
    LSU_LH   = 4'd2,
    LSU_LW   = 4'd3,
    LSU_LBU  = 4'd4,
    LSU_LHU  = 4'd5,

    LSU_SB   = 4'd8,
    LSU_SH   = 4'd9,
    LSU_SW   = 4'd10
  } lsu_op_t;

  function automatic logic lsu_is_load(lsu_op_t op);
    return (op inside {LSU_LB, LSU_LH, LSU_LW, LSU_LBU, LSU_LHU});
  endfunction

  function automatic logic lsu_is_store(lsu_op_t op);
    return (op inside {LSU_SB, LSU_SH, LSU_SW});
  endfunction

  // -------------------------
  // LSU packets
  // -------------------------
  typedef struct packed {
    lsu_op_t           op;
    logic [XLEN-1:0]   addr;
    logic [XLEN-1:0]   store_data;
    logic [XLEN-1:0]   mem_rdata;
  } lsu_in_t;

  typedef struct packed {
    logic              mem_re;
    logic              mem_we;
    logic [XLEN-1:0]   mem_addr;
    logic [XLEN-1:0]   mem_wdata;
    logic [XLEN/8-1:0] mem_wstrb;

    logic [XLEN-1:0]   load_data;
    logic              misaligned;
    logic              illegal;
  } lsu_out_t;

  // -------------------------
  // Decoder packets
  // -------------------------
  typedef struct packed {
    logic [31:0] instr;
  } dec_in_t;

  typedef struct packed {
    opcode_t       opcode;
    logic [4:0]    rd;
    logic [2:0]    funct3;
    logic [4:0]    rs1;
    logic [4:0]    rs2;
    logic [6:0]    funct7;

    imm_sel_t      imm_sel;

    alu_op_t       alu_op;
    src_a_sel_t    src_a_sel;
    src_b_sel_t    src_b_sel;

    wb_sel_t       wb_sel;
    logic          reg_write;

    branch_op_t    br_op;
    lsu_op_t       lsu_op;

    logic          is_jal;
    logic          is_jalr;

    logic          uses_rs1;
    logic          uses_rs2;

    logic          illegal;
  } dec_out_t;

endpackage
