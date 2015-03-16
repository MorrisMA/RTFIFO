`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates 
// Engineer:        Michael A. Morris
//
// Create Date:     11:12:57 03/14/2015
// Design Name:     RTFIFO
// Module Name:     C:/XProjects/ISE10.1i/RTFIFO/tb_RTFIFO.v
// Project Name:    RTFIFO
// Target Device:   SRAM-based FPGAs w/ distributed and block RAM  
// Tool versions:   ISE 10.1i SP3
//
// Description: 
//
// Verilog Test Fixture created by ISE for module: RTFIFO
//
// Dependencies:
// 
// Revision:
//
//  0.01    15C14   MAM     Initial Creation
//
//  1.00    15C15   MAM     Completed self-checking testbench implementation
//
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_RTFIFO;

parameter pN = 2;
parameter [3:0] pIDLE = 4'b1000;

reg     Clk;
reg     Rst;

reg     TF_Rst;

reg     TF_Rd;
reg     TF_Wr;
wire    TF_EF;
wire    TF_FF;
reg     [7:0] TDI;
wire    [7:0] TDO;

reg     RF_Rst;

reg     RF_Rd;
reg     RF_Wr;
wire    RF_EF;
wire    RF_FF;
reg     [7:0] RDI;
wire    [7:0] RDO;

//  Simulation Variables

integer i = 0;

// Instantiate the Unit Under Test (UUT)

RTFIFO #(
            .pRTFIFO_Bits(pN)
        ) uut (
            .Clk(Clk), 
            .Rst(Rst),
            
            .TF_Rst(TF_Rst),
            
            .TF_Wr(TF_Wr), 
            .TF_Rd(TF_Rd), 
            .TF_FF(TF_FF), 
            .TF_EF(TF_EF), 
            .TDI(TDI), 
            .TDO(TDO), 

            .RF_Rst(RF_Rst), 

            .RF_Wr(RF_Wr), 
            .RF_Rd(RF_Rd), 
            .RF_FF(RF_FF), 
            .RF_EF(RF_EF), 
            .RDI(RDI), 
            .RDO(RDO)
        );

initial begin
    // Initialize Inputs
    Clk     = 1;
    Rst     = 1;
    
    TF_Rst  = 1;

    TF_Rd   = 0;
    TF_Wr   = 0;
    TDI     = 0;

    RF_Rst  = 1;

    RF_Rd   = 0;
    RF_Wr   = 0;
    RDI     = 0;

    // Wait 100 ns for global reset to finish
    
    #101;
    Rst    = 0;
    TF_Rst = 0;
    RF_Rst = 0;
    
    // Add stimulus here

    // Check that on entry to Idle state, the FIFOs are both marked as empty
    
    @(uut.CS == pIDLE);
    if(~&{TF_EF, RF_EF}) begin
        $display("\tERROR: Expected both FIFOs to be empty!\n");
        $stop;
    end
    
    $display("Start: Test FIFO Writes\n");

    // Check that, when both FIFOs are written simultaneously, the Tx FIFO is
    //      written first. Also check that following the first write, the FIFOs are
    //      no longer empty, and that the output data register holds the initial
    //      value written.
    
    fork
        WrTF(8'h55);
        WrRF(8'hAA);

        begin
            @(posedge uut.Rst_WR_TF);
            if(~RF_EF) begin
                $display("\tERROR: Expected WrTF should have priority over WrRF\n");
                $stop;
            end
        end
    join
    
    @(uut.CS == pIDLE);
    if( &{TF_EF, RF_EF}) begin
        $display("\tERROR: Expected both FIFOs to be not empty!\n");
        $stop;
    end
    if(TDO != 8'h55) begin
        $display("\tERROR: Expected TDO to match data written to empty FIFO\n");
        $stop;
    end
    if(RDO != 8'hAA) begin
        $display("\tERROR: Expected RDO to match data written to empty FIFO\n");
        $stop;
    end
    
    //  Separate the tests
    
    for(i = 0; i < 10; i = i + 1) begin
        @(posedge Clk) #1;
    end

    //  Check that both FIFOs fill when expected, and that the outputs don't
    //      change.
    
    for(i = 0; i < ((2**pN) - 1); i = i + 1) begin
        fork
            begin   // Allow RF to be written before the TF every other write.
                if(i[0]) @(posedge Clk) #1;
                WrTF(i);
            end
            WrRF(~i);
        join
    end
    
    @(uut.CS == pIDLE);
    if(~&{TF_FF, RF_FF}) begin
        $display("\tERROR: Expected both FIFOs to indicate full\n");
        $stop;
    end
    if(TDO != 8'h55) begin
        $display("\tERROR: Expected TDO to match data written to empty FIFO\n");
        $stop;
    end
    if(RDO != 8'hAA) begin
        $display("\tERROR: Expected RDO to match data written to empty FIFO\n");
        $stop;
    end
    
    //  Check that writes to full FIFOs are ignored
    
    fork
        begin : Wr_Full_TF
            WrTF(~0);
            disable WrTF_Timeout;
        end
        begin : WrTF_Timeout
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            
            disable Wr_Full_TF; // WrTF hangs until ACKed, so timeout task
        end
    join
    if(uut.CS != pIDLE) begin
        $display("\tERROR: Unexpected write operation to full Tx FIFO\n");
        $stop;
    end
    
    fork
        begin : Wr_Full_RF
            WrRF(~0);
            disable WrRF_Timeout;
        end
        begin : WrRF_Timeout
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            
            disable Wr_Full_RF; // WrRF hangs until ACKed, so timeout task
        end
    join
    if(uut.CS != pIDLE) begin
        $display("\tERROR: Unexpected write operation to full Rx FIFO\n");
        $stop;
    end
    
    $display("End: Test FIFO Writes -- PASS\n");
    
    //  Separate Tests
    
    for(i = 0; i < 10; i = i + 1) begin
        @(posedge Clk) #1;
    end
    
    $display("Start: Test FIFO Reads\n");

    // Check that, when both FIFOs are read simultaneously, the Rx FIFO is read
    //      first. Also check that following the first write, the FIFOs are no
    //      longer empty, and that the output data register holds the initial
    //      value written.
    
    fork
        RdTF;
        RdRF;

        begin
            @(posedge uut.Rst_RD_RF);
            if(~TF_FF) begin
                $display("\tERROR: Expected RdRF to have priority over RdTF\n");
                $stop;
            end
        end
    join
    
    if( |{TF_FF, RF_FF}) begin
        $display("\tERROR: Expected both FIFOs to be not full!\n");
        $stop;
    end
    if(TDO != 8'h00) begin
        $display("\tERROR: Expected TDO to match data written to empty FIFO\n");
        $stop;
    end
    if(RDO != 8'hFF) begin
        $display("\tERROR: Expected RDO to match data written to empty FIFO\n");
        $stop;
    end
    
    //  Separate the tests
    
    for(i = 0; i < 10; i = i + 1) begin
        @(posedge Clk) #1;
    end

    //  Check that both FIFOs empty when expected, and that the outputs don't
    //      change.
    
    for(i = 0; i < ((2**pN) - 1); i = i + 1) begin
        fork
            RdTF;
            begin   // Allow TF to be read before the RF every other write.
                if(i[0]) @(posedge Clk) #1;
                RdRF;
            end
        join
    end
    
    if(~&{TF_EF, RF_EF}) begin
        $display("\tERROR: Expected both FIFOs to indicate empty\n");
        $stop;
    end
    if(TDO != 8'h55) begin
        $display("\tERROR: Expected TDO to match first data written to TF\n");
        $stop;
    end
    if(RDO != 8'hAA) begin
        $display("\tERROR: Expected RDO to match first data written to RF\n");
        $stop;
    end
    
    //  Check that reads to empty FIFOs are ignored
    
    fork
        begin : Rd_Empty_TF
            RdTF;
            disable RdTF_Timeout;
        end
        begin : RdTF_Timeout
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            
            disable Rd_Empty_TF; // RdTF hangs until ACKed, so timeout task
        end
    join
    if(uut.CS != pIDLE) begin
        $display("\tERROR: Unexpected read operation to empty Tx FIFO\n");
        $stop;
    end
    
    fork
        begin : Rd_Empty_RF
            RdRF;
            disable RdRF_Timeout;
        end
        begin : RdRF_Timeout
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            @(posedge Clk);
            
            disable Rd_Empty_RF; // RdRF hangs until ACKed, so timeout task
        end
    join
    if(uut.CS != pIDLE) begin
        $display("\tERROR: Unexpected read operation to empty Rx FIFO\n");
        $stop;
    end
    
    $display("End: Test FIFO Reads -- PASS\n");
    
    //  Separate Tests
    
    for(i = 0; i < 10; i = i + 1) begin
        @(posedge Clk) #1;
    end
    
    $display("End of Tests -- PASS\n");
end

////////////////////////////////////////////////////////////////////////////////

always #5 Clk = ~Clk;

////////////////////////////////////////////////////////////////////////////////

//  Generate a write to the Transmit FIFO

task WrTF;
    
    input [7:0] DI;
    
begin
    @(posedge Clk) #1 TF_Wr = 1; TDI = DI; 
    @(posedge Clk) #1 TF_Wr = 0;

    @(negedge uut.Rst_WR_TF);   // wait for operation to be completed
end

endtask

//  Generate a read from the Transmit FIFO

task RdTF;
    
begin
    @(posedge Clk) #1 TF_Rd = 1;
    @(posedge Clk) #1 TF_Rd = 0;

    @(negedge uut.WE_TDO);  // wait for operation to be completed.
end

endtask

//  Generate a write to the Receive FIFO

task WrRF;
    
    input [7:0] DI;
    
begin
    @(posedge Clk) #1 RF_Wr = 1; RDI = DI; 
    @(posedge Clk) #1 RF_Wr = 0;

    @(negedge uut.Rst_WR_RF);   // wait for operation to be completed
end

endtask

//  Generate a read from the Receive FIFO

task RdRF;
    
begin
    @(posedge Clk) #1 RF_Rd = 1;
    @(posedge Clk) #1 RF_Rd = 0;
    
    @(negedge uut.WE_RDO);  // wait for operation to be completed.
end

endtask

endmodule
