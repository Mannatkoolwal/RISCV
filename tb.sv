`timescale 1ns/1ps
module tb;
    reg clk = 0, reset = 1;
    riscv_single dut (.clk(clk), .reset(reset));

    always #5 clk = ~clk;

    integer errors = 0;
    task check(input [63:0] got, input [63:0] exp, input [127:0] name);
        if (got !== exp) begin
            $display("FAIL %0s: got %0d, expected %0d", name, got, exp);
            errors = errors + 1;
        end else $display("PASS %0s = %0d", name, got);
    endtask

    initial begin
        #12 reset = 0;      // release reset just after the first clock edge
        #100;               // run 10 instruction cycles
        check(dut.u_rf.rf[1], 10, "x1");
        check(dut.u_rf.rf[2], 20, "x2");
        check(dut.u_rf.rf[3], 30, "x3");
        check(dut.u_rf.rf[4], 30, "x4 (ld)");
        check(dut.u_rf.rf[5],  0, "x5 (skipped)");
        check(dut.u_rf.rf[6],  2, "x6");
        if (errors == 0) $display("ALL TESTS PASSED"); else $display("%0d TEST(S) FAILED", errors);
        $finish;
    end
endmodule
