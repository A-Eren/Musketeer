// packages/riscv_pkg.sv
package riscv_pkg;

  // --------------------------------------------
  // RV32I Opcodes
  // --------------------------------------------
  localparam logic [6:0] OPCODE_OP       = 7'b0110011; // R-type
  localparam logic [6:0] OPCODE_OP_IMM   = 7'b0010011; // I-type ALU
  localparam logic [6:0] OPCODE_LOAD     = 7'b0000011;
  localparam logic [6:0] OPCODE_STORE    = 7'b0100011;
  localparam logic [6:0] OPCODE_BRANCH   = 7'b1100011;
  localparam logic [6:0] OPCODE_JALR     = 7'b1100111;
  localparam logic [6:0] OPCODE_JAL      = 7'b1101111;
  localparam logic [6:0] OPCODE_LUI      = 7'b0110111;
  localparam logic [6:0] OPCODE_AUIPC    = 7'b0010111;

  localparam int FUNCT7_ALT_BIT = 5; // funct7[5] == instr[30]

  // --------------------------------------------
  // ALU Operations
  // --------------------------------------------
  typedef enum logic [4:0] {
    ALU_ADD      = 5'd0,
    ALU_SUB      = 5'd1,
    ALU_AND      = 5'd2,
    ALU_OR       = 5'd3,
    ALU_XOR      = 5'd4,
    ALU_SLL      = 5'd5,
    ALU_SRL      = 5'd6,
    ALU_SRA      = 5'd7,
    ALU_SLT      = 5'd8,   // signed
    ALU_SLTU     = 5'd9,   // unsigned
    ALU_COPY_A   = 5'd10,
    ALU_COPY_B   = 5'd11
  } alu_op_t;

  // --------------------------------------------
  // LOAD funct3
  // --------------------------------------------
  localparam logic [2:0] LSU_LB  = 3'b000;
  localparam logic [2:0] LSU_LH  = 3'b001;
  localparam logic [2:0] LSU_LW  = 3'b010;
  localparam logic [2:0] LSU_LBU = 3'b100;
  localparam logic [2:0] LSU_LHU = 3'b101;  
  // STORE funct3
  localparam logic [2:0] LSU_SB  = 3'b000;
  localparam logic [2:0] LSU_SH  = 3'b001;
  localparam logic [2:0] LSU_SW  = 3'b010;
  // --------------------------------------------
  // Datapath control selects
  // --------------------------------------------
  typedef enum logic [0:0] {
    SRC_A_RS1 = 1'b0,
    SRC_A_PC  = 1'b1
  } src_a_sel_t;

  typedef enum logic [0:0] {
    SRC_B_RS2 = 1'b0,
    SRC_B_IMM = 1'b1
  } src_b_sel_t;

  typedef enum logic [1:0] {
    WB_ALU = 2'd0,
    WB_MEM = 2'd1,
    WB_PC4 = 2'd2
  } wb_sel_t;
  // --------------------------------------------
  // BRANCH funct3
  // --------------------------------------------
  localparam logic [2:0] F3_BEQ  = 3'b000;
  localparam logic [2:0] F3_BNE  = 3'b001;
  localparam logic [2:0] F3_BLT  = 3'b100;
  localparam logic [2:0] F3_BGE  = 3'b101;
  localparam logic [2:0] F3_BLTU = 3'b110;
  localparam logic [2:0] F3_BGEU = 3'b111;

endpackage
