`timescale 1ns/1ps
module tb_bne;
    reg clk = 0, reset = 1;
    riscv_pipe dut (.clk(clk), .reset(reset));
    always #5 clk = ~clk;
    initial begin $dumpfile("wave_bne.vcd"); $dumpvars(0, tb_bne); end

    // trace: shows each time the bne is in EX and whether it is taken
    always @(posedge clk) if (!reset && dut.idex_branch)
        $display("t=%0t  bne in EX: x1(fwd)=%0d  zero=%b  taken=%b", $time, dut.ex_a, dut.ex_zero, dut.ex_pc_src);

    integer errors = 0;
    task check(input [63:0] got, input [63:0] exp, input [255:0] name);
        if (got !== exp) begin $display("FAIL %0s: got %0d, expected %0d", name, got, exp); errors = errors + 1; end
        else $display("PASS %0s = %0d", name, got);
    endtask
    initial begin
        #12 reset = 0;
        #300;
        check(dut.u_rf.rf[1], 0, "x1 (counter)");
        check(dut.u_rf.rf[2], 6, "x2 (3+2+1)");
        check(dut.u_rf.rf[3], 1, "x3 (after loop)");
        if (errors == 0) $display("ALL TESTS PASSED"); else $display("%0d TEST(S) FAILED", errors);
        $finish;
    end
endmodule
