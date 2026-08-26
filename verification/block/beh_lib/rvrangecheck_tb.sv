// This testbench tests 3 variants of the 64-bit rvrangecheck module

class RandAddr;
    rand logic [63:0] addr;
endclass


module rvrangecheck_tb #(
`include "eh2_param.vh"
);


   initial begin
      $dumpfile("dump.vcd");
      $dumpvars();
   end

    typedef struct packed {
        logic [63:0] start_addr;
        logic [63:0] end_addr;

    } addr_range_t;

    localparam addr_range_t Regions [32] = '{
        '{start_addr: 64'h00000000_00000000, end_addr: 64'h00000000_0FFFFFFF},
        '{start_addr: 64'h00000000_10000000, end_addr: 64'h00000000_1FFFFFFF},
        '{start_addr: 64'h00000000_20000000, end_addr: 64'h00000000_2FFFFFFF},
        '{start_addr: 64'h00000000_30000000, end_addr: 64'h00000000_3FFFFFFF},
        '{start_addr: 64'h00000000_40000000, end_addr: 64'h00000000_4FFFFFFF},
        '{start_addr: 64'h00000000_50000000, end_addr: 64'h00000000_5FFFFFFF},
        '{start_addr: 64'h00000000_60000000, end_addr: 64'h00000000_6FFFFFFF},
        '{start_addr: 64'h00000000_70000000, end_addr: 64'h00000000_7FFFFFFF},
        '{start_addr: 64'h00000000_80000000, end_addr: 64'h00000000_8FFFFFFF},
        '{start_addr: 64'h00000000_90000000, end_addr: 64'h00000000_9FFFFFFF},
        '{start_addr: 64'h00000000_A0000000, end_addr: 64'h00000000_AFFFFFFF},
        '{start_addr: 64'h00000000_B0000000, end_addr: 64'h00000000_BFFFFFFF},
        '{start_addr: 64'h00000000_C0000000, end_addr: 64'h00000000_CFFFFFFF},
        '{start_addr: 64'h00000000_D0000000, end_addr: 64'h00000000_DFFFFFFF},
        '{start_addr: 64'h00000000_E0000000, end_addr: 64'h00000000_EFFFFFFF},
        '{start_addr: 64'h00000000_F0000000, end_addr: 64'h00000000_FFFFFFFF},
        '{start_addr: 64'h00000001_00000000, end_addr: 64'h0FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h10000000_00000000, end_addr: 64'h1FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h20000000_00000000, end_addr: 64'h2FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h30000000_00000000, end_addr: 64'h3FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h40000000_00000000, end_addr: 64'h4FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h50000000_00000000, end_addr: 64'h5FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h60000000_00000000, end_addr: 64'h6FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h70000000_00000000, end_addr: 64'h7FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h80000000_00000000, end_addr: 64'h8FFFFFFF_FFFFFFFF},
        '{start_addr: 64'h90000000_00000000, end_addr: 64'h9FFFFFFF_FFFFFFFF},
        '{start_addr: 64'hA0000000_00000000, end_addr: 64'hAFFFFFFF_FFFFFFFF},
        '{start_addr: 64'hB0000000_00000000, end_addr: 64'hBFFFFFFF_FFFFFFFF},
        '{start_addr: 64'hC0000000_00000000, end_addr: 64'hCFFFFFFF_FFFFFFFF},
        '{start_addr: 64'hD0000000_00000000, end_addr: 64'hDFFFFFFF_FFFFFFFF},
        '{start_addr: 64'hE0000000_00000000, end_addr: 64'hEFFFFFFF_FFFFFFFF},
        '{start_addr: 64'hF0000000_00000000, end_addr: 64'hFFFFFFFF_FFFFFFFF}
    };
    localparam logic [63:0] CCM_SADR = 64'(pt.DCCM_SADR);
    localparam int CCM_REGION = 15;

    logic [63:0] addr;
    logic in_range32, in_region32;
    logic in_range48, in_region48;
    logic in_range128, in_region128;

    RandAddr addr_generator = new;

    rvrangecheck #(
        .CCM_SADR(CCM_SADR),
        .CCM_SIZE(32)
    ) dut32 (
        .addr,
        .in_range(in_range32),
        .in_region(in_region32)
    );

    rvrangecheck #(
        .CCM_SADR(CCM_SADR),
        .CCM_SIZE(48)
    ) dut48 (
        .addr,
        .in_range(in_range48),
        .in_region(in_region48)
    );

    rvrangecheck #(
        .CCM_SADR(CCM_SADR),
        .CCM_SIZE(128)
    ) dut128 (
        .addr,
        .in_range(in_range128),
        .in_region(in_region128)
    );

    logic is_in_region32, is_in_region48, is_in_region128;
    logic is_in_range32, is_in_range48, is_in_range128;

    function void update_ranges (ref logic is_in_range32, is_in_range48, is_in_range28);
        is_in_range32 = (addr >= CCM_SADR) && (addr <= (CCM_SADR + (32 * 1024)));
        is_in_range48 = (addr >= CCM_SADR) && (addr <= (CCM_SADR + (48 * 1024)));
        is_in_range128 = (addr >= CCM_SADR) && (addr <= (CCM_SADR + (128 * 1024)));
    endfunction

    function void assert_all_match (input logic in_region32, in_region48, in_region128, in_range32, in_range48, in_range128,
                                                is_in_region32, is_in_region48, is_in_region128, is_in_range32, is_in_range48, is_in_range128);
        assert (in_region32 == is_in_region32) else $error("[%0t ps] Mismatch in_region32 got: %x, expected: %x", $time, in_region32, is_in_region32);
        assert (in_region48 == is_in_region48) else $error("[%0t ps] Mismatch in_region48 got: %x, expected: %x", $time, in_region48, is_in_region48);
        assert (in_region128 == is_in_region128) else $error("[%0t ps] Mismatch in_region128 got: %x, expected: %x", $time, in_region128, is_in_region128);
        assert (in_range32 == is_in_range32) else $error("[%0t ps] Mismatch in_range32 got: %x, expected: %x", $time, in_range32, is_in_range32);
        assert (in_range48 == is_in_range48) else $error("[%0t ps] Mismatch in_range48 got: %x, expected: %x", $time, in_range48, is_in_range48);
        assert (in_range128 == is_in_range128) else $error("[%0t ps] Mismatch in_range128 got: %x, expected: %x", $time, in_range128, is_in_range128);
    endfunction

    function void assert_all_false (logic in_region32, in_region48, in_region128, in_range32, in_range48, in_range128);
        assert (~in_region32) else $error("[%0t ps] Mismatch in_region32 got: 1, expected: 0", $time);
        assert (~in_region48) else $error("[%0t ps] Mismatch in_region48 got: 1, expected: 0", $time);
        assert (~in_region128) else $error("[%0t ps] Mismatch in_region128 got: 1, expected: 0", $time);
        assert (~in_range32) else $error("[%0t ps] Mismatch in_range32 got: 1, expected: 0", $time);
        assert (~in_range48) else $error("[%0t ps] Mismatch in_range48 got: 1, expected: 0", $time);
        assert (~in_range128) else $error("[%0t ps] Mismatch in_range128 got: 1, expected: 0", $time);
    endfunction

    initial begin
        #10;
        for (int i = 0; i < 32; i++) begin
            // Calculate whether CCM is in selected region
            if (CCM_REGION == i) begin
                is_in_region32 = 1'b1;
                is_in_region48 = 1'b1;
                is_in_region128 = 1'b1;
            end else begin
                is_in_region32 = 1'b0;
                is_in_region48 = 1'b0;
                is_in_region128 = 1'b0;
            end

            // Ensure whether module works for first region address
            addr = Regions[i].start_addr;
            #1 update_ranges(is_in_range32, is_in_range48, is_in_range128);
            assert_all_match(in_region32, in_region48, in_region128, in_range32, in_range48, in_range128,
                             is_in_region32, is_in_region48, is_in_region128, is_in_range32, is_in_range48, is_in_range128);

            // Ensure whether module works for last region address
            addr = Regions[i].end_addr;
            #1 update_ranges(is_in_range32, is_in_range48, is_in_range128);
            assert_all_match(in_region32, in_region48, in_region128, in_range32, in_range48, in_range128,
                             is_in_region32, is_in_region48, is_in_region128, is_in_range32, is_in_range48, is_in_range128);

            // Ensure whether module works for random in-region address
            addr_generator.randomize() with {
                addr > Regions[i].start_addr;
                addr < Regions[i].end_addr;
            };
            addr = addr_generator.addr;
            #1 update_ranges(is_in_range32, is_in_range48, is_in_range128);
            assert_all_match(in_region32, in_region48, in_region128, in_range32, in_range48, in_range128,
                             is_in_region32, is_in_region48, is_in_region128, is_in_range32, is_in_range48, is_in_range128);

            // Ensure whether module works for random out-region address
            addr_generator.randomize() with {
                (addr < Regions[i].start_addr) || (addr > Regions[i].end_addr);
            };
            addr = addr_generator.addr;
            #1 assert_all_false(in_region32, in_region48, in_region128, in_range32, in_range48, in_range128);
        end
    end

endmodule
