////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2012, 2015 by Michael A. Morris, dba M. A. Morris & Associates
//
//  All rights reserved. The source code contained herein is publicly released
//  under the terms and conditions of the GNU Lesser Public License. No part of
//  this source code may be reproduced or transmitted in any form or by any
//  means, electronic or mechanical, including photocopying, recording, or any
//  information storage and retrieval system in violation of the license under
//  which the source code is released.
//
//  The souce code contained herein is free; it may be redistributed and/or 
//  modified in accordance with the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either version 2.1 of
//  the GNU Lesser General Public License, or any later version.
//
//  The souce code contained herein is freely released WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
//  PARTICULAR PURPOSE. (Refer to the GNU Lesser General Public License for
//  more details.)
//
//  A copy of the GNU Lesser General Public License should have been received
//  along with the source code contained herein; if not, a copy can be obtained
//  by writing to:
//
//  Free Software Foundation, Inc.
//  51 Franklin Street, Fifth Floor
//  Boston, MA  02110-1301 USA
//
//  Further, no use of this source code is permitted in any form or means
//  without inclusion of this banner prominently in any derived works. 
//
//  Michael A. Morris
//  Huntsville, AL
//
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:         Michael A. Morris 
// Engineer:        M. A. Morris & Associates
// 
// Create Date:     11:10:50 12/02/2012 
// Design Name:     Microprogrammed Controlled Block RAM Receive/Transmit FIFO
// Module Name:     RTFIFO 
// Project Name:    C:\XProjects\ISE10.1i\RTFIFO
// Target Devices:  Spartan-II FPGAs or later 
// Tool versions:   ISE 10.1i SP3
//
// Description:
//
//  This module provides a bidirectional FIFO implementation using Block RAMs
//  for use as the transmit/receive FIFOs of a UART, or similar bidirectional
//  device. The implementation of the module makes several critical assumptions:
//
//      (1) The FIFO is connected to a microprocessor-type parallel bus which
//          operates at a read/write rate significantly slower than the opera-
//          ting clock rate of the module;
//
//      (2) The UART, or similar communications device, is bidirectional, but
//          like the microprocessor, the read/write rate is significantly slower
//          than the operating clock rate of the module;
//
//      (3) The microprocessor read/write operations to the FIFOs are not simul-
//          taneous, and a write to the transmit FIFO is buffered;
//
//      (4) The UART, or similar device, read/write operations to the FIFOs may
//          be simultaneous, and a write to the receive FIFO is buffered;
//
//      (5) The microprocessor bus interface is significantly faster than the
//          UART interface;
//
//      (6) Both FIFOs will operate as first word fall-through FIFOs;
//
//      (7) The FIFO controller will service both FIFOs in a time multiplexed
//          manner, and it will be implemented using a microprogrammed state
//          machine.
//
//  The first two assumptions define the behaviour of the external client inter-
//  faces relative to the FIFO controller. Essentially, these two assumptions
//  establish that neither client interface is operating near the clock rate of
//  the FIFO controller. Assumptions three and four establish the requirement 
//  that there will be at most three simultaneous operations: (1) UART read of
//  transmit FIFO, (2) UART write of receive FIFO, and (3) write of transmit or
//  read of receive FIFO by the microprocessor. The fifth assumption requires
//  that the microprocessor interface will have priority over the UART inter-
//  face. The sixth assumption establishes the requirement that the FIFOs will
//  operate in a manner similar to that of a LUT-based FIFO because writes to an
//  empty FIFO will be advanced to a holding register on the output. This means
//  that the client interfaces can simply read the output registers, and when
//  that operation is complete, the FIFOs will advance the next stored value to
//  the output registers. (This characteristic makes the FIFOs easier to use by
//  simple client interfaces.) The final assumption simply defines that there
//  will only be one microprogrammed FIFO controller to handle both FIFOs.
//
// Dependencies: 
//
// Revision:
//
//  0.00    12L02   MAM     File Created
//
//  0.10    15C14   MAM     Corrected error in the definition of the micropro-
//                          gram ROM. Initially defined as a wire. Corrected to
//                          be defined as a reg. Appears to synthesize correctly
//                          as defined originally, but unable to simulate with
//                          ISim: terminates with an ACCESS_VIOLATION_EXCEPTION.
//
//  0.20    15C15           Corrected error in the definition of Sum. Incorrect-
//                          ly defined as having width related to the address
//                          of the BRAM. It should have defined having a width
//                          related to the depth of the FIFO. The BRAM address
//                          has another bit in its definition in order to hold
//                          both FIFOs in a single BRAM. This error resulted in
//                          the FF flag not being set as expected. Corrected the
//                          delayed output register write enable signal to use
//                          the FIFO RAM Read Enable as designed originally.
//
// Additional Comments: 
//
//  This implementation is based on the RTFIFO implementation in schematic form
//  developed in Foundation 2.1i SP6 in 2005. The RTFIFO of that implementation
//  satisfies the seven assumptions described above. It is uses a custom 16x14
//  microprogram ROM implemented using LUTs. The control fields are simple, and
//  the next state of the controller is directly defined in the microprogram.
//  When the next state indicates a transition to the idle state, a priority
//  encoder is enabled. The priority encoder either allows the transition to the
//  idle state, or it inserts the address of the appropriate microprogram
//  sequence into the current state register. Because a LUT is used for the
//  microprogram ROM, the LUT RAM outputs are the current state control signals
//  for the various logic functions.
//
//  The Word Counter (WCntr), Write Pointer (WPtr), and Read Pointer (RPtr) of
//  each FIFO (Transmit or Receive) are maintained in a LUT RAM instead of inde-
//  pendent counters. These counters/pointers are maintained in two groups of
//  four, which requires at least one 16 by m (where m corresponds at least to
//  the width of the address of the FIFO's block RAM) LUT. Similarly, the logic
//  signal defining transmit/receive operation splits the FIFO's block RAM into
//  two blocks. Only three arithmetic operations are used: (1) increment (for
//  the pointers and the word counter), (2) decrement (for the word counter), 
//  and clear to initialize the counters and pointers.
//
//  The original implementation did not have access to SMRTool, so the micropro-
//  gram was hand-coded and used encoded 2-bit fields to provide easy to remem-
//  ber mnemonics of the control functions represented by each field. This
//  module will use the same encoded field structure to make it easier to test
//  and compare the operation of the HDL module to the schematic-based version.
//
////////////////////////////////////////////////////////////////////////////////

module RTFIFO #(
    parameter pRTFIFO_Bits = 8,                 // Number address bits
    // FIFO Controller Microprogram
    parameter pRTFIFO_uPgm = "Src/Microprogram-Sources/RTFIFO_uPgm.coe",
    // FIFO Register Initialization    
    parameter pRTFIFO_Init = "Src/Microprogram-Sources/RTFIFO_Init.coe",
    // FIFO BRAM Initialization
    parameter pRTFIFO_BRAM = "Src/Microprogram-Sources/RTFIFO_BRAM.coe", 
    parameter pTF_EF_Init  = 1'b1,              // TF Empty Flag Initial State
    parameter pRF_EF_Init  = 1'b1               // RF Empty Flag Initial State
)(
    input   Clk,            // System Clock
    input   Rst,            // System Reset
    
    input   TF_Rst,         // Falling-edge latch of Tx FIFO reset signal

    input   TF_Rd,          // Tx FIFO read pulse, asserted by TxSR Load
    input   TF_Wr,          // Tx FIFO write pulse, asserted by uP IF
    output  TF_EF,          // Tx FIFO Empty Flag
    output  TF_FF,          // Tx FIFO Full Flag
    input   [7:0] TDI,      // Tx FIFO Data In  => from uP bus
    output  reg [7:0] TDO,  // Tx FIFO Data Out => to transmit SR
    
    input   RF_Rst,         // Falling-edge latch of Rx FIFO reset signal
    
    input   RF_Rd,          // Rx FIFO read pulse, asserted by uP interface
    input   RF_Wr,          // Rx FIFO write pulse, asserted by RxSR DataRdy
    output  RF_EF,          // Rx FIFO Empty Flag
    output  RF_FF,          // Rx FIFO Full Flag
    input   [7:0] RDI,      // Rx FIFO Data In  => from receive SR
    output  reg [7:0] RDO   // Rx FIFO Data Out => to uP bus
);

////////////////////////////////////////////////////////////////////////////////
//
//  Local Parameters
//

localparam pBRAM_AddrWidth = (pRTFIFO_Bits + 1);
localparam pBRAM_Size      = (2**pBRAM_AddrWidth);

localparam pReset          = 4'b0000;   // FIFO Control Register Reset Sequence
localparam pWr             = 4'b0100;   // FIFO Write Sequence
localparam pRd             = 4'b0110;   // FIFO Read Sequence
localparam pIdle           = 4'b1000;   // Controller Idle State
localparam pWr_EF          = 4'b1001;   // FIFO Write Sequence when EF asserted

////////////////////////////////////////////////////////////////////////////////
//
//  Module Declarations
//

reg     dTF_Rst, dRF_Rst;                   // FIFO Reset Edge Detection Regs.

reg     RTF, WR_TF, RD_TF;                  // Transmit FIFO Command Registers 
reg     RRF, WR_RF, RD_RF;                  // Receive FIFO Command Registers

reg     [(pRTFIFO_Bits - 1):0] FCR [0:7];   // FIFO Control Registers LUT RAM
wire    [2:0] FCR_A;                        // FIFO Control Registers Address
wire    [(pRTFIFO_Bits - 1):0] FCRI;        // FIFO Control Registers Data In
wire    [(pRTFIFO_Bits - 1):0] FCRO;        // FIFO Control Registers Data Out

wire    [(pRTFIFO_Bits - 1):0] Sum;         // FIFO ALU Sum Output
wire    Z;                                  // FIFO ALU Output Zero Detector

reg     [7:0] FRAM [0:(pBRAM_Size - 1)];    // FIFO Block RAM (Receive/Transmit)
wire    [(pBRAM_AddrWidth - 1):0] FRAM_A;   // FIFO Block RAM Address
wire    [7:0] FRDI;                         // FIFO Block RAM Data In
reg     [7:0] FRDO;                         // FIFO Block RAM Data Out

reg     WE_TDO, WE_RDO;                     // Write Enable for TDO and RDO

reg     FS;                                 // FIFO Select: 0 - RF; 1 - TF;
reg     [3:0] CS;                           // FIFO Controller Current State                           

reg     [13:0] ROM [0:15];                  // FIFO Controller uPgm ROM
wire    [13:0] uPL;                         // FIFO Controller uPgm ROM Output

wire    [3:0] NS;                           // uPgm Next State Field
wire    [1:0] RS;                           // uPgm Register Select Field
wire    [1:0] AS;                           // uPgm ALU Operation Select Field
wire    [1:0] MS;                           // uPgm FIFO RAM Operation Field
wire    [1:0] CC;                           // uPgm Clear Command Field
wire    [1:0] SF;                           // uPgm Set Flag Field

wire    En, Add, FCR_WE;                    // ALU Select Control Field

wire    WE_FRAM, RE_FRAM;                   // Memory Select Control Field

wire    Clr_RTF, R_RDTF, WE_TF;             // Clear Command Control Field - TF              
wire    Clr_RRF, R_RDRF, WE_RF;             // Clear Command Control Field - RF

reg     TEF, TFF;                           // Transmit FIFO Flag Registers
reg     REF, RFF;                           // Receive FIFO Flag Registers

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

//  Implement FIFO Command Registers
//      Since there is a single, time-multiplexed controller all reset, read,
//      and write requests need to be saved in registers until the controller
//      services the request. Each FIFO requires three command registers:
//      (1) Reset FIFO, (2) Write FIFO, and (3) Read FIFO. Thus, there are a
//      total of six command registers. The Reset FIFO commands are falling edge
//      sensitive. The Write FIFO and Read FIFO inputs are pulsed signals, which
//      may be formed by external edge detectors, so they drive the command
//      registers directly, while the Rest FIFOs commands are processed through
//      falling detectors built into the module. An assumption is made that, if
//      required, clock domain crossing logic is applied to all input signals
//      external to this module.

//  Falling Edge Detection for Reset FIFOs

always @(posedge Clk)
begin
    if(Rst)
        {dTF_Rst, dRF_Rst} <= #1 0;
    else
        {dTF_Rst, dRF_Rst} <= #1 {TF_Rst, RF_Rst};
end

assign FE_TF_Rst = ~TF_Rst & dTF_Rst;
assign FE_RF_Rst = ~RF_Rst & dRF_Rst;

//  FIFO Reset Command Registers

assign Set_RTF = (Rst | FE_TF_Rst);

always @(posedge Clk)
begin
    if(Set_RTF)
        RTF <= #1 1;
    else if(Clr_RTF)
        RTF <= #1 0;
end

assign Set_RRF = (Rst | FE_RF_Rst);

always @(posedge Clk)
begin
    if(Set_RRF)
        RRF <= #1 1;
    else if(Clr_RRF)
        RRF <= #1 0;
end

//  Transmit FIFO Command Registers

assign Set_WR_TF = (TF_Wr & ~TFF);
assign Rst_WR_TF = (WE_TF | RTF);

always @(posedge Clk)
begin
    if(Set_WR_TF)
        WR_TF <= #1 1;
    else if(Rst_WR_TF)
        WR_TF <= #1 0;
end

assign Set_RD_TF = (TF_Rd  & ~TEF);
assign Rst_RD_TF = (R_RDTF | RTF);

always @(posedge Clk)
begin
    if(Set_RD_TF)
        RD_TF <= #1 1;
    else if(Rst_RD_TF)
        RD_TF <= #1 0;
end

//  Receive FIFO Command Registers

assign Set_WR_RF = (RF_Wr & ~RFF);
assign Rst_WR_RF = (WE_RF | RRF);

always @(posedge Clk)
begin
    if(Set_WR_RF)
        WR_RF <= #1 1;
    else if(Rst_WR_RF)
        WR_RF <= #1 0;
end

assign Set_RD_RF = (RF_Rd  & ~REF);
assign Rst_RD_RF = (R_RDRF | RRF);

always @(posedge Clk)
begin
    if(Set_RD_RF)
        RD_RF <= #1 1;
    else if(Rst_RD_RF)
        RD_RF <= #1 0;
end

//  Define FIFO Control Register Address

assign FCR_A = {FS, RS};

//  Infer FIFO Control Register (FCR) LUT RAM

initial
    $readmemb(pRTFIFO_Init, FCR, 0, 7);
    
always @(posedge Clk)
begin
    if(FCR_WE)
        FCR[FCR_A] <= #1 FCRI;
end

assign FCRO = FCR[FCR_A];

//  Implement FIFO Control Register Arithmetic Operations

assign Sum  = ((Add) ? (FCRO + 1) : (FCRO - 1));
assign FCRI = ((En)  ? Sum        : 0         );

//  Detect Result of FCR Arithmetic Operations

assign Z = ~|Sum;

//  Generate FIFO BRAM Address

assign FRAM_A = {FS, FCRO};

//  Multiplex FIFO BRAM Data Input

assign FRDI = ((FS) ? TDI : RDI);

//  Infer FIFO Block RAM

initial
    $readmemb(pRTFIFO_BRAM, FRAM, 0, (pBRAM_Size - 1));

always @(posedge Clk)
begin
    if(WE_FRAM) begin
        FRAM[FRAM_A] <= #1 FRDI;
    end
    
    FRDO <= #1 FRAM[FRAM_A];
end

// Delay FIFO RAM Read Enables to generate output register Write Enables

always @(posedge Clk)
begin
    if(Rst) begin
        WE_TDO  <= #1 0;
        WE_RDO  <= #1 0;
    end else begin
        WE_TDO  <= #1 RE_FRAM &  FS;
        WE_RDO  <= #1 RE_FRAM & ~FS;
    end
end

// Implement FIFO output registers

always @(posedge Clk)
begin
    if(Rst)
        TDO <= #1 0;
    else if(WE_TDO)
        TDO <= #1 FRDO;
end

always @(posedge Clk)
begin
    if(Rst)
        RDO <= #1 0;
    else if(WE_RDO)
        RDO <= #1 FRDO;
end

////////////////////////////////////////////////////////////////////////////////
//
//  Begin FIFO Controller SM
//

//  Implement Priority Encoder, FIFO Select (FS), and Current State (CS)
//      Output of priority encoder is the next state
//      If NS field of the microprogram is not (NS==pIdle), then the priority
//      encoder is not active, and it should output the value of uP_NS field.
//

assign Idle = (NS == pIdle);        // Define Idle signal

always @(posedge Clk)
begin
    if(Rst)
        {FS, CS} <= #1 {1'b0, pReset};
    else
        casex({Idle,                    // Highest Priority
              RTF,                      // Reset Transmit FIFO
              RRF,                      // Reset Receive FIFO
              (WR_TF & TF_EF),          // Write Transmit FIFO when  EF
              WR_TF,                    // Write Transmit FIFO when ~EF & ~FF
              RD_RF,                    // Read Receive FIFO   when ~EF
              RD_TF,                    // Read Transmit FIFO  when ~EF
              (WR_RF & RF_EF),          // Write Receive FIFO  when  EF
              WR_RF           })        // Write Receive FIFO  when ~EF & ~FF
            //
            9'b0xx_xx_xx_xx : {FS, CS} <= #1 {  FS, NS    }; // NS uPgm ROM
            9'b11x_xx_xx_xx : {FS, CS} <= #1 {1'b1, pReset}; // Reset TF
            9'b101_xx_xx_xx : {FS, CS} <= #1 {1'b0, pReset}; // Reset RF
            9'b100_1x_xx_xx : {FS, CS} <= #1 {1'b1, pWr_EF}; // Write Empty TF
            9'b100_01_xx_xx : {FS, CS} <= #1 {1'b1, pWr   }; // Write TF
            9'b100_00_1x_xx : {FS, CS} <= #1 {1'b0, pRd   }; // Read RF
            9'b100_00_01_xx : {FS, CS} <= #1 {1'b1, pRd   }; // Read TF
            9'b100_00_00_1x : {FS, CS} <= #1 {1'b0, pWr_EF}; // Write Empty RF
            9'b100_00_00_01 : {FS, CS} <= #1 {1'b0, pWr   }; // Write RF
            9'b100_00_00_00 : {FS, CS} <= #1 {  FS, pIdle }; // Stay in Idle
        endcase
end

//  Infer Microprogram ROM (LUT-based)

initial
    $readmemb(pRTFIFO_uPgm, ROM, 0, 15);
    
assign uPL = ROM[CS];

//  Define Microprogram Fields

assign NS = uPL[13:10];     // Next State
assign RS = uPL[ 9: 8];     // Register Select: 0 - WCntr; 2 - WPtr; 3 - RPtr
assign AS = uPL[ 7: 6];     // ALU Operation Select: 1 - Clr; 2 - Dec; 3 - Inc
assign MS = uPL[ 5: 4];     // FRAM Operation: 2 - Read FRAM; 3 - Write FRAM
assign CC = uPL[ 3: 2];     // Clear Cmd: 1 - Clr_Rst; 2 - Clr_Rd; 3 - Clr_Wr
assign SF = uPL[ 1: 0];     // Flag Set: 1 - Flg_Init; 2 - Set_EF; 3 - Set_FF

//  Decode ALU Control Field

assign En     = AS[1]; 
assign Add    = &AS;
assign FCR_WE = |AS;

//  Decode FRAM Operation Control Field

assign WE_FRAM = MS[1] &  MS[0];
assign RE_FRAM = MS[1] & ~MS[0];

//  Decode Clear Command Control Field

assign Clr_RTF =  FS & (CC == 2'b01);   // Clear Reset TF Flag
assign R_RDTF  =  FS & (CC == 2'b10);   // Generate Delayed CE for TDO
assign WE_TF   =  FS & (CC == 2'b11);   // Write TF

assign Clr_RRF = ~FS & (CC == 2'b01);   // Clear Reset RF Flag
assign R_RDRF  = ~FS & (CC == 2'b10);   // Generate Delayed CE for RDO
assign WE_RF   = ~FS & (CC == 2'b11);   // Write RF

//  Decode Set Flags Control Field

assign Clr_TFlgs =  FS & (SF == 2'b01); // Init TF Empty (1) and Full (0) Flags
assign Set_TEF   =  FS & (SF == 2'b10); // Set TF Empty Flag
assign Set_TFF   =  FS & (SF == 2'b11); // Set TF Full Flag

assign Clr_RFlgs = ~FS & (SF == 2'b01); // Init RF Empty (1) and Full (0) Flags
assign Set_REF   = ~FS & (SF == 2'b10); // Set RF Empty Flag
assign Set_RFF   = ~FS & (SF == 2'b11); // Set RF Full Flag

////////////////////////////////////////////////////////////////////////////////
//
//  Implement FIFO Flags
//

//  Transmit FIFO Empty Flag

assign TEF_Set = (Rst | Clr_TFlgs);

always @(posedge Clk)
begin
    if(TEF_Set)
        TEF <= #1 1;
    else if(Set_TFF)
        TEF <= #1 0;
    else if(Set_TEF)
        TEF <= #1 Z;
end

assign TF_EF = TEF;

//  Transmit FIFO Full Flag

assign TFF_Clr = (Rst | Set_TEF);

always @(posedge Clk)
begin
    if(TFF_Clr)
        TFF <= #1 0;
    else if(Set_TFF)
        TFF <= #1 Z;
end

assign TF_FF = TFF;

//  Receive FIFO Empty Flag

assign REF_Set = (Rst | Clr_RFlgs);

always @(posedge Clk)
begin
    if(REF_Set)
        REF <= #1 1;
    else if(Set_RFF)
        REF <= #1 0;
    else if(Set_REF)
        REF <= #1 Z;
end

assign RF_EF = REF;

//  Receive FIFO Full Flag

assign RFF_Clr = (Rst | Set_REF);

always @(posedge Clk)
begin
    if(RFF_Clr)
        RFF <= #1 0;
    else if(Set_RFF)
        RFF <= #1 Z;
end

assign RF_FF = RFF;

////////////////////////////////////////////////////////////////////////////////
//
//  End of Implementation
//

endmodule
