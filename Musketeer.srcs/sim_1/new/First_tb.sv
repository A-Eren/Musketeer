`timescale 1ns/1ps

module tb_core_ext;
  import riscv_pkg::*;

  // clock/reset
  logic clk;
  logic rst_n;

  // IMEM
  logic [31:0] imem_rdata;
  logic [31:0] imem_addr;

  // DMEM
  logic [31:0] dmem_addr;
  logic [31:0] dmem_wdata;
  logic [3:0]  dmem_wstrb;
  logic        dmem_we;
  logic        dmem_re;
  logic [31:0] dmem_rdata;

  // debug
  logic [31:0] dbg_pc;
  logic        dbg_illegal;
  logic        dbg_misaligned;

  // -------------------------
  // DUT
  // -------------------------
  Riscv_Core dut (
    .clk(clk),
    .rst_n(rst_n),

    .imem_addr(imem_addr),
    .imem_rdata(imem_rdata),

    .dmem_addr(dmem_addr),
    .dmem_wdata(dmem_wdata),
    .dmem_wstrb(dmem_wstrb),
    .dmem_we(dmem_we),
    .dmem_re(dmem_re),
    .dmem_rdata(dmem_rdata),

    .dbg_pc(dbg_pc),
    .dbg_illegal(dbg_illegal),
    .dbg_misaligned(dbg_misaligned)
  );

  // -------------------------
  // Clock
  // -------------------------
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // -------------------------
  // Simple IMEM (word addressed)
  // -------------------------
  logic [31:0] imem [0:255];
  assign imem_rdata = imem[imem_addr[31:2]];

  // -------------------------
  // Simple DMEM (byte-addressable)
  // -------------------------
  logic [7:0] dmem [0:4095];

  function automatic logic [31:0] dmem_read_word(input logic [31:0] addr);
    logic [31:0] a;
    begin
      a = {addr[31:2], 2'b00}; // align
      dmem_read_word = {
        dmem[a + 32'd3],
        dmem[a + 32'd2],
        dmem[a + 32'd1],
        dmem[a + 32'd0]
      };
    end
  endfunction

  function automatic logic [7:0] dmem_read_byte(input logic [31:0] addr);
    begin
      dmem_read_byte = dmem[addr];
    end
  endfunction

  function automatic logic [15:0] dmem_read_half(input logic [31:0] addr);
    begin
      dmem_read_half = {dmem[addr + 32'd1], dmem[addr + 32'd0]};
    end
  endfunction

  // combinational read (aligned word)
  always_comb begin
    dmem_rdata = dmem_read_word(dmem_addr);
  end

  // write on posedge with byte strobes (little-endian)
  always_ff @(posedge clk) begin
    if (dmem_we) begin
      if (dmem_addr + 32'd3 < 32'd4096) begin
        if (dmem_wstrb[0]) dmem[dmem_addr + 32'd0] <= dmem_wdata[7:0];
        if (dmem_wstrb[1]) dmem[dmem_addr + 32'd1] <= dmem_wdata[15:8];
        if (dmem_wstrb[2]) dmem[dmem_addr + 32'd2] <= dmem_wdata[23:16];
        if (dmem_wstrb[3]) dmem[dmem_addr + 32'd3] <= dmem_wdata[31:24];
      end
    end
  end

  // -------------------------
  // RV32I instruction encoders
  // -------------------------
  function automatic logic [31:0] enc_r(
    input logic [6:0] funct7,
    input logic [4:0] rs2,
    input logic [4:0] rs1,
    input logic [2:0] funct3,
    input logic [4:0] rd,
    input logic [6:0] opcode
  );
    enc_r = {funct7, rs2, rs1, funct3, rd, opcode};
  endfunction

  function automatic logic [31:0] enc_i(
    input logic [11:0] imm12,
    input logic [4:0]  rs1,
    input logic [2:0]  funct3,
    input logic [4:0]  rd,
    input logic [6:0]  opcode
  );
    enc_i = {{20{imm12[11]}}, imm12, rs1, funct3, rd, opcode};
  endfunction

  function automatic logic [31:0] enc_s(
    input logic [11:0] imm12,
    input logic [4:0]  rs2,
    input logic [4:0]  rs1,
    input logic [2:0]  funct3,
    input logic [6:0]  opcode
  );
    enc_s = {imm12[11:5], rs2, rs1, funct3, imm12[4:0], opcode};
  endfunction

  function automatic logic [31:0] enc_b(
    input logic [12:0] imm13, // bit0 must be 0 (byte offset), we assume caller does that
    input logic [4:0]  rs2,
    input logic [4:0]  rs1,
    input logic [2:0]  funct3,
    input logic [6:0]  opcode
  );
    enc_b = {imm13[12], imm13[10:5], rs2, rs1, funct3, imm13[4:1], imm13[11], opcode};
  endfunction

  function automatic logic [31:0] enc_u(
    input logic [19:0] imm20,
    input logic [4:0]  rd,
    input logic [6:0]  opcode
  );
    enc_u = {imm20, rd, opcode};
  endfunction

  function automatic logic [31:0] enc_j(
    input logic [20:0] imm21, // bit0 must be 0
    input logic [4:0]  rd,
    input logic [6:0]  opcode
  );
    enc_j = {imm21[20], imm21[10:1], imm21[11], imm21[19:12], rd, opcode};
  endfunction

  // handy NOP
  function automatic logic [31:0] enc_nop();
    enc_nop = enc_i(12'd0, 5'd0, 3'b000, 5'd0, OPCODE_OP_IMM); // addi x0,x0,0
  endfunction

  // -------------------------
  // Program
  // -------------------------
  integer i;

  initial begin
    // init memories
    for (i = 0; i < 256; i++)  imem[i] = enc_nop();
    for (i = 0; i < 4096; i++) dmem[i] = 8'h00;

    // Register naming reminder:
    // x2  = 0x80 data byte
    // x3  = addr 1
    // x4  = LB result (sign-extended)
    // x5  = LBU result (zero-extended)
    // x7  = addr 2
    // x8  = 0x8001 value (built via shifts)
    // x9  = LH result (sign-extended)
    // x11 = LHU result (zero-extended)
    // x6  = link register from JALR (return address)
    // x10 = subroutine target address (64)
    // x15 = subroutine computed result (=15)

    // ---- main @ PC=0 ----
    // x2 = 0x80
    imem[0]  = enc_i(12'd128, 5'd0, 3'b000, 5'd2,  OPCODE_OP_IMM); // addi x2,x0,128
    // x3 = 1
    imem[1]  = enc_i(12'd1,   5'd0, 3'b000, 5'd3,  OPCODE_OP_IMM); // addi x3,x0,1
    // sb x2,0(x3)  -> mem[1] = 0x80
    imem[2]  = enc_s(12'd0,   5'd2, 5'd3, LSU_SB,    OPCODE_STORE); // sb x2,0(x3)
    // lb x4,0(x3)  -> 0xFFFF_FF80
    imem[3]  = enc_i(12'd0,   5'd3, LSU_LB, 5'd4,    OPCODE_LOAD);  // lb x4,0(x3)
    // lbu x5,0(x3) -> 0x0000_0080
    imem[4]  = enc_i(12'd0,   5'd3, LSU_LBU,5'd5,    OPCODE_LOAD);  // lbu x5,0(x3)

    // x7 = 2
    imem[5]  = enc_i(12'd2,   5'd0, 3'b000, 5'd7,    OPCODE_OP_IMM); // addi x7,x0,2
    // x8 = 1
    imem[6]  = enc_i(12'd1,   5'd0, 3'b000, 5'd8,    OPCODE_OP_IMM); // addi x8,x0,1
    // x8 = x8 << 15  (SLLI)
    imem[7]  = enc_i({7'b0000000, 5'd15}, 5'd8, 3'b001, 5'd8, OPCODE_OP_IMM); // slli x8,x8,15
    // x8 = x8 + 1 => 0x8001
    imem[8]  = enc_i(12'd1,   5'd8, 3'b000, 5'd8,    OPCODE_OP_IMM); // addi x8,x8,1
    // sh x8,0(x7) -> mem[2..3] = 0x8001 (little-endian: [2]=0x01, [3]=0x80)
    imem[9]  = enc_s(12'd0,   5'd8, 5'd7, LSU_SH,     OPCODE_STORE);  // sh x8,0(x7)
    // lh x9,0(x7)  -> 0xFFFF_8001
    imem[10] = enc_i(12'd0,   5'd7, LSU_LH, 5'd9,     OPCODE_LOAD);   // lh x9,0(x7)
    // lhu x11,0(x7) -> 0x0000_8001
    imem[11] = enc_i(12'd0,   5'd7, LSU_LHU,5'd11,    OPCODE_LOAD);   // lhu x11,0(x7)

    // JALR subroutine call:
    // x10 = 64 (byte address)
    imem[12] = enc_i(12'd64,  5'd0, 3'b000, 5'd10,    OPCODE_OP_IMM); // addi x10,x0,64
    // jalr x6,0(x10)  -> x6 = PC+4 (= 12*4+4 = 52), pc = 64
    imem[13] = enc_i(12'd0,   5'd10,3'b000, 5'd6,     OPCODE_JALR);   // jalr x6,0(x10)

    // after return, we execute here:
    imem[14] = enc_i(12'd1,   5'd0, 3'b000, 5'd12,    OPCODE_OP_IMM); // addi x12,x0,1

    // stop/loop
    imem[15] = enc_j(21'd0, 5'd0, OPCODE_JAL); // jal x0,0

    // ---- subroutine @ PC=64 => imem[16] ----
    // x13 = 7
    imem[16] = enc_i(12'd7,   5'd0, 3'b000, 5'd13,    OPCODE_OP_IMM); // addi x13,x0,7
    // x14 = 8
    imem[17] = enc_i(12'd8,   5'd0, 3'b000, 5'd14,    OPCODE_OP_IMM); // addi x14,x0,8
    // x15 = x13 + x14 = 15
    imem[18] = enc_r(7'b0000000, 5'd14, 5'd13, 3'b000, 5'd15, OPCODE_OP); // add x15,x13,x14
    // return: jalr x0,0(x6)
    imem[19] = enc_i(12'd0,   5'd6, 3'b000, 5'd0,     OPCODE_JALR);   // jalr x0,0(x6)

    // -------------------------
    // reset
    // -------------------------
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    // run enough cycles
    repeat (80) @(posedge clk);

    // -------------------------
    // checks
    // -------------------------
    $display("PC=%08x illegal=%0d misaligned=%0d", dbg_pc, dbg_illegal, dbg_misaligned);
    $display("x4 (LB) =%08x  x5 (LBU)=%08x", dut.u_rf.regs[4], dut.u_rf.regs[5]);
    $display("x9 (LH) =%08x  x11(LHU)=%08x", dut.u_rf.regs[9], dut.u_rf.regs[11]);
    $display("x6 (link)=%08x x15(sub)=%0d x12=%0d",
             dut.u_rf.regs[6], dut.u_rf.regs[15], dut.u_rf.regs[12]);

    // LB/LBU checks
    if (dut.u_rf.regs[4] !== 32'hFFFF_FF80) $fatal(1, "FAIL: LB expected FFFF_FF80, got %08x", dut.u_rf.regs[4]);
    if (dut.u_rf.regs[5] !== 32'h0000_0080) $fatal(1, "FAIL: LBU expected 0000_0080, got %08x", dut.u_rf.regs[5]);

    // LH/LHU checks (0x8001)
    if (dut.u_rf.regs[9]  !== 32'hFFFF_8001) $fatal(1, "FAIL: LH expected FFFF_8001, got %08x", dut.u_rf.regs[9]);
    if (dut.u_rf.regs[11] !== 32'h0000_8001) $fatal(1, "FAIL: LHU expected 0000_8001, got %08x", dut.u_rf.regs[11]);

    // Memory byte/halfword checks
    if (dmem_read_byte(32'd1) !== 8'h80) $fatal(1, "FAIL: mem[1] expected 0x80, got %02x", dmem_read_byte(32'd1));

    // little-endian halfword at addr 2: [2]=0x01, [3]=0x80 => 0x8001
    if (dmem_read_half(32'd2) !== 16'h8001) $fatal(1, "FAIL: mem[2..3] expected 0x8001, got %04x", dmem_read_half(32'd2));

    // JALR link check:
    // jalr at imem[13] => PC = 13*4 = 52, link = PC+4 = 56 (0x38)
    if (dut.u_rf.regs[6] !== 32'h0000_0038) $fatal(1, "FAIL: x6 link expected 0x38, got %08x", dut.u_rf.regs[6]);

    // subroutine result x15 = 15
    if (dut.u_rf.regs[15] !== 32'd15) $fatal(1, "FAIL: x15 expected 15, got %0d", dut.u_rf.regs[15]);

    // post-return marker
    if (dut.u_rf.regs[12] !== 32'd1) $fatal(1, "FAIL: x12 expected 1 after return, got %0d", dut.u_rf.regs[12]);

    if (dbg_illegal)    $fatal(1, "FAIL: illegal instruction flagged");
    if (dbg_misaligned) $fatal(1, "FAIL: misaligned access flagged");

    $display("PASS ? (LB/LBU, LH/LHU, SB/SH, JALR tested)");
    $finish;
  end

endmodule
