// rtl/Pipe_Registers.sv
module Pipe_Registers #(
  parameter type T = logic [31:0],
  parameter T RESET_VAL = '0,
  parameter T FLUSH_VAL = '0
) (
  input  logic clk,
  input  logic rst_n,   // active-low reset (matches your core)

  input  logic stall,  // when 1: hold current value
  input  logic flush,  // when 1: overwrite with FLUSH_VAL (bubble)

  input  T     d,
  output T     q
);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q <= RESET_VAL;
    end
    else if (flush) begin
      q <= FLUSH_VAL;
    end
    else if (!stall) begin
      q <= d;
    end
    // else stall==1 => hold q
  end

endmodule
