module eh2_exu_div_ctl_tb;
  import std::*;
  import eh2_pkg::*;

  localparam int CLK_PERIOD = 10;
  initial begin
      $dumpfile("dump.vcd");
      $dumpvars();
  end

  class RandDivPair;
    rand logic signed [`RV_XLEN-1:0] dividend;
    rand logic signed [`RV_XLEN-1:0] divisor;
    rand bit nums_signed;
    rand bit mod_op;
    bit valid;

    constraint c_no_div_by_zero {
      divisor != 0;
    }

    constraint c_no_overflow {
      !(dividend == {1'b1, {(`RV_XLEN-1){1'b0}}} && divisor == -1);
    }
  endclass

  logic                   clk;
  logic                   rst_l;
  logic                   scan_mode;

  logic                   cancel;
  logic                   finish_dly;
  logic signed [`RV_XLEN-1:0]    out;

  eh2_div_pkt_t        dp_in;
  logic signed [`RV_XLEN-1:0] dividend_in;
  logic signed [`RV_XLEN-1:0] divisor_in;

  RandDivPair div_nums;
  logic signed [`RV_XLEN-1:0] exp_div_res;
  int corr_count = 0;
  int NUM_ITER;
  string op_str;

  initial begin
    if (!$value$plusargs("ITER=%d", NUM_ITER))
      NUM_ITER = 100;
  end

  eh2_exu_div_ctl  dut (
    .clk(clk),
    .rst_l(rst_l),
    .scan_mode(scan_mode),

    .dp(dp_in),
    .dividend(dividend_in),
    .divisor(divisor_in),

    .cancel(cancel),
    .finish_dly(finish_dly),
    .out(out)
  );

  always #(CLK_PERIOD/2)  clk = ! clk ;

  initial begin
    div_nums = new;
    div_nums.valid = 0;
    rst_l = 1'b0;
    scan_mode = 1'b0;
    cancel = 1'b0;

    #(10*CLK_PERIOD) rst_l = 1'b1;

    fork : f_apply_stimuli
      apply_stimuli();
    join_none

    repeat(2) @(posedge clk);

    for (int i=0; i<NUM_ITER; i++) begin
      @(negedge clk);
      div_nums.randomize();
      div_nums.valid = 1'b1;
      op_str = div_nums.mod_op ? "mod" : "div";

      // calculate the expected result
      if (div_nums.nums_signed) begin
          exp_div_res = div_nums.mod_op ? (div_nums.dividend % div_nums.divisor) : (div_nums.dividend / div_nums.divisor);
      end else begin
          exp_div_res = div_nums.mod_op ? $unsigned(div_nums.dividend) % $unsigned(div_nums.divisor) : $unsigned(div_nums.dividend) / $unsigned(div_nums.divisor);
      end

      #(CLK_PERIOD) div_nums.valid = 1'b0;

      @(posedge clk iff finish_dly) assert (out == exp_div_res) else $display("[ERROR]: %0d %s %0d = %0d, EXP = %0d", div_nums.dividend, op_str, div_nums.divisor, out, exp_div_res);
      if (out == exp_div_res)
        corr_count += 1;

`ifdef DEBUG
      if (`RV_XLEN == 32)
        $display($sformatf("%12d %s %12d, OBS = %12d vs EXP = %12d (CORRECT = %0d)", div_nums.dividend, op_str, div_nums.divisor, out, exp_div_res, out==exp_div_res));
      else
        $display($sformatf("%22d %s %22d, OBS = %22d vs EXP = %22d (CORRECT = %0d)", div_nums.dividend, op_str, div_nums.divisor, out, exp_div_res, out==exp_div_res));
`endif
      #(2*CLK_PERIOD);
    end

    disable f_apply_stimuli;
    $display("*********************************************************************************************************************");
    `ifdef RV_DIV_NEW
      $display($sformatf("[TEST_DONE] %0d / %0d operations are CORRECT (XLEN = %0d, %0d-bit SRT module used)", corr_count, NUM_ITER, `RV_XLEN, `RV_DIV_BIT));
    `else
      $display($sformatf("[TEST_DONE] %0d / %0d operations are CORRECT (XLEN = %0d, 1bit_cheapshortq module used)", corr_count, NUM_ITER, `RV_XLEN));
    `endif
    if (corr_count == NUM_ITER)
      $display("\t\t\t\t[TEST_PASSED]");
    else
      $display($sformatf("\t\t[TEST_FAILED] %0d incorrect results", NUM_ITER - corr_count));
    $display("*********************************************************************************************************************");
    $finish();
  end

  task automatic apply_stimuli();
    forever begin
      dp_in.tid = 1'b0;
      dp_in.unsign = ~div_nums.nums_signed;
      dp_in.rem = div_nums.mod_op;
      dp_in.valid = div_nums.valid;

      dividend_in = div_nums.dividend;
      divisor_in  = div_nums.divisor;
      @(posedge clk);
    end
  endtask

endmodule
