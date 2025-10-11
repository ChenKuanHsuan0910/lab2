//========================================================================
// Functional 4-Stage Pipelined Mul/Div Unit (RV32M subset)
// - Stage0: latch request (a,b,fn) when req_val && req_rdy
// - Stage1: perform operation with RV32M corner cases
// - Stage2/3: pipeline dummies
// - No internal bypass
// - External pipeline may stall us via stall_Xhl/Mhl/X2hl/X3hl
// - Backpressure supported via resp_rdy (hold pipeline when output not consumed)
//========================================================================

`ifndef RISCV_PIPE_MULDIV_ITERATIVE_V
`define RISCV_PIPE_MULDIV_ITERATIVE_V

module riscv_CoreDpathPipeMulDiv
(
  input         clk,
  input         reset,

  // request
  input   [2:0] muldivreq_msg_fn,
  input  [31:0] muldivreq_msg_a,
  input  [31:0] muldivreq_msg_b,
  input         muldivreq_val,
  output        muldivreq_rdy,

  // response
  output [63:0] muldivresp_msg_result,
  output        muldivresp_val,
  input         muldivresp_rdy,

  // external stalls to align with 5-stage core
  input         stall_Xhl,
  input         stall_Mhl,
  input         stall_X2hl,
  input         stall_X3hl
);

  // Function code mapping (adjust if your control uses different codes)
  localparam MD_MUL    = 3'd0;
  localparam MD_DIV    = 3'd1;
  localparam MD_DIVU   = 3'd2;
  localparam MD_REM    = 3'd3;
  localparam MD_REMU   = 3'd4;
  localparam MD_MULH   = 3'd5;
  localparam MD_MULHSU = 3'd6;
  localparam MD_MULHU  = 3'd7;

  // Hold when any external stall is asserted or when output cannot be consumed
  wire pipeline_stall = (val3 & ~muldivresp_rdy);

  // Ready to accept a new request when we are not stalling this cycle
  assign muldivreq_rdy = ~pipeline_stall;

  // -----------------------------
  // Stage 0 regs
  // -----------------------------
  reg  [2:0] fn0;
  reg [31:0] a0, b0;
  reg        val0;

  // -----------------------------
  // Stage 1 regs (compute result here)
  // -----------------------------
  reg [63:0] res1;
  reg  [2:0] fn1;
  reg        val1;

  // -----------------------------
  // Stage 2 regs
  // -----------------------------
  reg [63:0] res2;
  reg        val2;

  // -----------------------------
  // Stage 3 regs (output stage)
  // -----------------------------
  reg [63:0] res3;
  reg        val3;

  // Stage 0: enqueue
  always @(posedge clk) begin
    if (reset) begin
      fn0  <= 3'b000;
      a0   <= 32'b0;
      b0   <= 32'b0;
      val0 <= 1'b0;
    end else if (!pipeline_stall) begin
      if (muldivreq_val && muldivreq_rdy) begin
        fn0  <= muldivreq_msg_fn;
        a0   <= muldivreq_msg_a;
        b0   <= muldivreq_msg_b;
        val0 <= 1'b1;
      end else begin
        // bubble if nothing comes in and we are advancing
        val0 <= 1'b0;
      end
    end
  end

  // Common wires for Stage1 math
  wire signed [31:0] a_s = a0;
  wire signed [31:0] b_s = b0;
  wire        [31:0] a_u = a0;
  wire        [31:0] b_u = b0;

  wire div_by_zero     = (b0 == 32'b0);
  wire is_signed_div   = (fn0 == MD_DIV) || (fn0 == MD_REM);
  wire signed_overflow = is_signed_div && (a0 == 32'h8000_0000) && (b0 == 32'hFFFF_FFFF);

  wire signed [63:0] prod_ss = $signed({{32{a0[31]}},a0}) * $signed({{32{b0[31]}},b0});
  wire        [63:0] prod_uu = {32'b0,a0} * {32'b0,b0};
  wire signed [63:0] prod_su = $signed({{32{a0[31]}},a0}) * $signed({32'b0,b0});
  // Helper absolute values for robust signed DIV/REM
  wire        a_neg = a0[31];
  wire        b_neg = b0[31];
  wire [31:0] abs_a = a_neg ? (~a0 + 1'b1) : a0;
  wire [31:0] abs_b = b_neg ? (~b0 + 1'b1) : b0;

  wire [31:0] quot_abs = (abs_b == 0) ? 32'd0 : (abs_a / abs_b);
  wire [31:0] rem_abs  = (abs_b == 0) ? abs_a : (abs_a % abs_b);

  wire [31:0] quot_sgn = (a_neg ^ b_neg) ? (~quot_abs + 1'b1) : quot_abs;
  wire [31:0] rem_sgn  = (a_neg)         ? (~rem_abs  + 1'b1) : rem_abs;


  // Stage 1: compute
  always @(posedge clk) begin
    if (reset) begin
      res1 <= 64'b0;
      fn1  <= 3'b000;
      val1 <= 1'b0;
    end else if (!pipeline_stall) begin
      fn1  <= fn0;
      val1 <= val0;
      case (fn0)
        MD_MUL   : res1 <= { prod_uu[63:32], prod_uu[31:0] };
        MD_MULH  : res1 <= { prod_ss[63:32], 32'b0 };
        MD_MULHSU: res1 <= { prod_su[63:32], 32'b0 };
        MD_MULHU : res1 <= { prod_uu[63:32], 32'b0 };

        MD_DIV   : begin
          if (div_by_zero)        res1 <= { 32'b0, 32'hFFFF_FFFF };
          else if (signed_overflow) res1 <= { 32'b0, 32'h8000_0000 };
          else                    res1 <= { 32'b0, quot_sgn };
        end

        MD_DIVU  : begin
          if (div_by_zero)        res1 <= { 32'b0, 32'hFFFF_FFFF };
          else                    res1 <= { 32'b0, (a_u / b_u) };
        end

        MD_REM   : begin
          if (div_by_zero)        res1 <= { a0, 32'b0 };
          else if (signed_overflow) res1 <= { 32'h0000_0000, 32'b0 };
          else                    res1 <= { rem_sgn, 32'b0 };
        end

        MD_REMU  : begin
          if (div_by_zero)        res1 <= { a0, 32'b0 };
          else                    res1 <= { (a_u % b_u), 32'b0 };
        end

        default  : res1 <= 64'b0;
      endcase
    end
  end

  // Stage 2
  always @(posedge clk) begin
    if (reset) begin
      res2 <= 64'b0;
      val2 <= 1'b0;
    end else if (!pipeline_stall) begin
      res2 <= res1;
      val2 <= val1;
    end
  end

  // Stage 3
  always @(posedge clk) begin
    if (reset) begin
      res3 <= 64'b0;
      val3 <= 1'b0;
    end else if (!pipeline_stall) begin
      res3 <= res2;
      val3 <= val2;
    end
  end

  // Response
  assign muldivresp_msg_result = res3;
  assign muldivresp_val        = val3;

endmodule

`endif
