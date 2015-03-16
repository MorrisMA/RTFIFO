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
//  The source code contained herein is free; it may be redistributed and/or 
//  modified in accordance with the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either version 2.1 of
//  the GNU Lesser General Public License, or any later version.
//
//  The source code contained herein is freely released WITHOUT ANY WARRANTY;
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
//          manner.
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
//  will only be one FIFO controller to handle both FIFOs.
//
// Dependencies: 
//
// Revision:
//
//  0.00    12L02   MAM     File Created
//
//  0.10    15C15   MAM     Realigned implementation to agree with the micro-
//                          programmed SM implementation - RTFIFO.v. Corrected
//                          error in the generation of the FIFO RAMs address.
//                          Had to add the FS signal as the msb of the address.
//
// Additional Comments: 
//
//  This implementation is based on the RTFIFO implementation in schematic form
//  developed in Foundation 2.1i SP6 in 2005. The RTFIFO of that implementation
//  satisfies the seven assumptions described above. The control fields are
//  simple, and the next state of the controller are defined in a linear manner.
//  When the next state indicates a transition to the idle state, a priority
//  encoder is enabled. The priority encoder either allows the transition to the
//  idle state, or it inserts the state of the state sequence that services the
//  next higher priority event request.
//
//  The Word Counter (WCntr), Write Pointer (WPtr), and Read Pointer (RPtr) of
//  each FIFO (Transmit or Receive) are maintained in discrete counters. These
//  counters/pointers are maintained in two groups of three. Similarly, the
//  event request signal being serviced determines which group of registers will
//  be used by the controller to service the request. Only three arithmetic ope-
//  rations are used: (1) increment (for the pointers and the word counter),
//  (2) decrement (for the word counter), and (3) clear to initialize the coun-
//  ters and pointers.
//
////////////////////////////////////////////////////////////////////////////////

module RTFIFO_std #(
    parameter pRTFIFO_Bits = 8,                 // Number address bits
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
localparam pReset_1        = 4'b0001;
localparam pReset_2        = 4'b0010;
localparam pReset_3        = 4'b0011;
//
localparam pWr             = 4'b0100;   // FIFO Write Sequence
localparam pWr_1           = 4'b0101;
//
localparam pRd             = 4'b0110;   // FIFO Read Sequence
localparam pRd_1           = 4'b0111;
//
localparam pIdle           = 4'b1000;   // Controller Idle State
//
localparam pWr_EF          = 4'b1001;   // FIFO Write Sequence when EF asserted
localparam pWr_EF_1        = 4'b1010;
localparam pWr_EF_2        = 4'b1011;
//
localparam pUnused         = 4'b1100;   // Unused states - transitioned to pIdle
localparam pUnused_1       = 4'b1101;
localparam pUnused_2       = 4'b1110;
localparam pUnused_3       = 4'b1111;

//  Define FIFO Counter Select Vector positions

localparam pRF_WCnt = 0;
localparam pRF_WPtr = 1;
localparam pRF_RPtr = 2;
//
localparam pTF_WCnt = 3;
localparam pTF_WPtr = 4;
localparam pTF_RPtr = 5;

//  Definitions for the FIFO State Machine Control Signals

localparam pNOP        = 0;

localparam pWCnt       = 0;
localparam pWPtr       = 2;
localparam pRPtr       = 3;

localparam pClr        = 1;
localparam pDec        = 2;
localparam pInc        = 3;

localparam pRd_FRAM    = 2;
localparam pWr_FRAM    = 3;

localparam pClr_xRF    = 1;
localparam pClr_RDxF   = 2;
localparam pWE_xF      = 3;

localparam pInit_xFlgs = 1;
localparam pSet_xEF    = 2;
localparam pSet_xFF    = 3;

////////////////////////////////////////////////////////////////////////////////
//
//  Module Declarations
//

reg     dTF_Rst, dRF_Rst;                   // FIFO Reset Edge Detection Regs.

reg     RTF, WR_TF, RD_TF;                  // Transmit FIFO Command Registers 
reg     RRF, WR_RF, RD_RF;                  // Receive FIFO Command Registers

wire    [2:0] FCR_A;                        // FIFO Control Register Address
reg     [5:0] CSel;                         // FIFO Counter Register Select

reg     [(pRTFIFO_Bits - 1):0] RF_WCnt;     // Rx FIFO Word Counter
reg     [(pRTFIFO_Bits - 1):0] RF_WPtr;     // Rx FIFO Write Pointer
reg     [(pRTFIFO_Bits - 1):0] RF_RPtr;     // Rx FIFO Read Pointer

reg     [(pRTFIFO_Bits - 1):0] TF_WCnt;     // Tx FIFO Word Counter
reg     [(pRTFIFO_Bits - 1):0] TF_WPtr;     // Tx FIFO Write Pointer
reg     [(pRTFIFO_Bits - 1):0] TF_RPtr;     // Tx FIFO Read Pointer

reg     [(pRTFIFO_Bits - 1):0] FCRO;        // FIFO ALU Control Register Output
wire    [(pRTFIFO_Bits - 1):0] FCRI;        // FIFO ALU Control Register Input
wire    [(pRTFIFO_Bits - 1):0] Sum;         // FIFO ALU Sum
wire    Z;                                  // FIFO ALU Output Zero Detector

reg     [7:0] FRAM [0:(pBRAM_Size - 1)];    // FIFO Block RAM (Receive/Transmit)
reg     [(pBRAM_AddrWidth - 1):0] FRAM_A;   // FIFO Block RAM Address
wire    [7:0] FRDI;                         // FIFO Block RAM Data In
reg     [7:0] FRDO;                         // FIFO Block RAM Data Out

reg     WE_TDO, WE_RDO;                     // Write Enable for TDO and RDO

reg     [3:0] CS;                           // FIFO Controller Current State                           

reg     [3:0] Nxt_Evnt;                     // Next Event/State
reg     [3:0] NS;                           // Next State Field

reg     Nxt_FIFO;                           // Next FIFO Select
reg     NF;
reg     FS;                                 // FIFO Select: 0 - RF; 1 - TF;

reg     [1:0] RS;                           // Register Select Field
reg     [1:0] AS;                           // Operation Select Field
reg     [1:0] MS;                           // FIFO RAM Operation Field
reg     [1:0] CC;                           // Clear Command Field
reg     [1:0] SF;                           // Set Flag Field

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
//
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

//  Decode the Counter Register Select

always @(*)
begin
    case(FCR_A)
        3'b000  : CSel <= 6'b000_001;   // Rx FIFO Wordcount Register
        3'b010  : CSel <= 6'b000_010;   // Rx FIFO Write Pointer Register
        3'b011  : CSel <= 6'b000_100;   // Rx FIFO Read Pointer Register
        3'b100  : CSel <= 6'b001_000;   // Tx FIFO Wordcount Register
        3'b110  : CSel <= 6'b010_000;   // Tx FIFO Write Pointer Register
        3'b111  : CSel <= 6'b100_000;   // Tx FIFO Read Pointer Register
        default : CSel <= 6'b000_000;   // No FIFO Counter Registers Selected
    endcase
end

//  Implement Receive FIFO Registers

assign CE_RF_WCnt = CSel[pRF_WCnt] & FCR_WE;

always @(posedge Clk)
begin
    if(Rst)
        RF_WCnt <= #1 0;
    else if(CE_RF_WCnt)
        RF_WCnt <= #1 FCRI;
end

assign CE_RF_RPtr = CSel[pRF_RPtr] & FCR_WE;

always @(posedge Clk)
begin
    if(Rst)
        RF_RPtr <= #1 0;
    else if(CE_RF_RPtr)
        RF_RPtr <= #1 FCRI;
end

assign CE_RF_WPtr = CSel[pRF_WPtr] & FCR_WE;

always @(posedge Clk)
begin
    if(Rst)
        RF_WPtr <= #1 0;
    else if(CE_RF_WPtr)
        RF_WPtr <= #1 FCRI;
end

//  Implement Transmit FIFO Registers

assign CE_TF_WCnt = CSel[pTF_WCnt] & FCR_WE;

always @(posedge Clk)
begin
    if(Rst)
        TF_WCnt <= #1 0;
    else if(CE_TF_WCnt)
        TF_WCnt <= #1 FCRI;
end

assign CE_TF_RPtr = CSel[pTF_RPtr] & FCR_WE;

always @(posedge Clk)
begin
    if(Rst)
        TF_RPtr <= #1 0;
    else if(CE_TF_RPtr)
        TF_RPtr <= #1 FCRI;
end

assign CE_TF_WPtr = CSel[pTF_WPtr] & FCR_WE;

always @(posedge Clk)
begin
    if(Rst)
        TF_WPtr <= #1 0;
    else if(CE_TF_WPtr)
        TF_WPtr <= #1 FCRI;
end

//  Select FIFO Arithmetic Unit Input

always @(*)
begin
    case(CSel)
        // Rx FIFO
        6'b000_001 : FCRO <= RF_WCnt;
        6'b000_010 : FCRO <= RF_WPtr;
        6'b000_100 : FCRO <= RF_RPtr;
        // Tx FIFO
        6'b001_000 : FCRO <= TF_WCnt;
        6'b010_000 : FCRO <= TF_WPtr;
        6'b100_000 : FCRO <= TF_RPtr;
        // No FIFO
        default    : FCRO <= 0;
    endcase
end

//  Implement FIFO Register Arithmetic Unit

assign Sum  = ((Add) ? (FCRO + 1) : (FCRO - 1));
assign FCRI = ((En)  ? Sum        : 0         );

//  Detect Zero for FIFO Register Arithmetic Operations

assign Z = ~|Sum;

//  Generate FIFO BRAM Address

always @(*)
begin
    case(FCR_A)
        // Rx FIFO
        3'b010  : FRAM_A <= {FS, RF_WPtr};
        3'b011  : FRAM_A <= {FS, RF_RPtr};
        // Tx FIFO
        3'b110  : FRAM_A <= {FS, TF_WPtr};
        3'b111  : FRAM_A <= {FS, TF_RPtr};
        // No FIFO 
        default : FRAM_A <= 0;
    endcase
end

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

// Delay FIFO Read Enables to generate FIFO output register Write Enables

always @(posedge Clk)
begin
    if(Rst) begin
        WE_TDO <= #1 0;
        WE_RDO <= #1 0;
    end else begin
        WE_TDO <= #1 RE_FRAM &  FS;
        WE_RDO <= #1 RE_FRAM & ~FS;
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

//  Implement Priority Encoder - Select Next Event to Process, or goto Idle
//      Output of priority encoder is the next state

always @(*)
begin
    casex({RTF,                     // Reset Transmit FIFO
           RRF,                     // Reset Receive FIFO
           (WR_TF & TF_EF),         // Write Transmit FIFO when  EF
           WR_TF,                   // Write Transmit FIFO when ~EF & ~FF
           RD_RF,                   // Read Receive FIFO   when ~EF
           RD_TF,                   // Read Transmit FIFO  when ~EF
           (WR_RF & RF_EF),         // Write Receive FIFO  when  EF
           WR_RF           })       // Write Receive FIFO  when ~EF & ~FF
        //
        8'b1x_xx_xx_xx : Nxt_Evnt <= pReset;    // Reset TF
        8'b01_xx_xx_xx : Nxt_Evnt <= pReset;    // Reset RF
        8'b00_1x_xx_xx : Nxt_Evnt <= pWr_EF;    // Write Empty TF
        8'b00_01_xx_xx : Nxt_Evnt <= pWr;       // Write TF
        8'b00_00_1x_xx : Nxt_Evnt <= pRd;       // Read RF
        8'b00_00_01_xx : Nxt_Evnt <= pRd;       // Read TF
        8'b00_00_00_1x : Nxt_Evnt <= pWr_EF;    // Write Empty RF
        8'b00_00_00_01 : Nxt_Evnt <= pWr;       // Write RF
        8'b00_00_00_00 : Nxt_Evnt <= pIdle;     // Stay in Idle
    endcase
end

//  Next State Transition Equations

always @(*)
begin
    case(CS)
        // Initialize/Reset FIFO
        4'b0000 : NS <= pReset_1;           // Clear Word Counter
        4'b0001 : NS <= pReset_2;           // Dummy operation
        4'b0010 : NS <= pReset_3;           // Clear Write Pointer, Set EF
        4'b0011 : NS <= Nxt_Evnt;           // Clear Read Pointer
        // Write FIFO
        4'b0100 : NS <= pWr_1;              // Write FIFO, Inc Wr Pointer
        4'b0101 : NS <= Nxt_Evnt;           // Increment Word Count, Set FF
        // Read FIFO
        4'b0110 : NS <= pRd_1;              // Decrement Word Count, Set EF
        4'b0111 : NS <= Nxt_Evnt;           // Read FIFO, Inc Rd Pointer
        // Idle state
        4'b1000 : NS <= Nxt_Evnt;           // Select next event to service
        // Write to FIFO when EF asserted
        4'b1001 : NS <= pWr_EF_1;           // Write FIFO, Inc Wr Pointer
        4'b1010 : NS <= pWr_EF_2;           // Read FIFO, Inc Rd Pointer
        4'b1011 : NS <= Nxt_Evnt;           // Increment Word Counter, Set FF
        // Unused States
        4'b1100 : NS <= Nxt_Evnt;           // Select next event to service
        4'b1101 : NS <= Nxt_Evnt;           // Select next event to service
        4'b1110 : NS <= Nxt_Evnt;           // Select next event to service
        4'b1111 : NS <= Nxt_Evnt;           // Select next event to service
    endcase
end

//  Implement Current State Register

always @(posedge Clk)
begin
    if(Rst)
        CS <= #1 pReset;
    else
        CS <= #1 NS;
end

// Implement the FIFO Select Logic

//  Implement Priority Encoder - select next FIFO to process, or goto Idle
//      Output of priority encoder is the next state

always @(*)
begin
    casex({RTF,                     // Reset Transmit FIFO
           RRF,                     // Reset Receive FIFO
           (WR_TF & TF_EF),         // Write Transmit FIFO when  EF
           WR_TF,                   // Write Transmit FIFO when ~EF & ~FF
           RD_RF,                   // Read Receive FIFO   when ~EF
           RD_TF,                   // Read Transmit FIFO  when ~EF
           (WR_RF & RF_EF),         // Write Receive FIFO  when  EF
           WR_RF           })       // Write Receive FIFO  when ~EF & ~FF
        //
        8'b1x_xx_xx_xx : Nxt_FIFO <= 1;     // Reset TF
        8'b01_xx_xx_xx : Nxt_FIFO <= 0;     // Reset RF
        8'b00_1x_xx_xx : Nxt_FIFO <= 1;     // Write Empty TF
        8'b00_01_xx_xx : Nxt_FIFO <= 1;     // Write TF
        8'b00_00_1x_xx : Nxt_FIFO <= 0;     // Read RF
        8'b00_00_01_xx : Nxt_FIFO <= 1;     // Read TF
        8'b00_00_00_1x : Nxt_FIFO <= 0;     // Write Empty RF
        8'b00_00_00_01 : Nxt_FIFO <= 0;     // Write RF
        8'b00_00_00_00 : Nxt_FIFO <= FS;    // Stay in Idle
    endcase
end

//  Next FIFO Transition Equations

always @(*)
begin
    case(CS)
        4'b0011 : NF <= Nxt_FIFO;           // End of FIFO Reset Sequence
        4'b0101 : NF <= Nxt_FIFO;           // End of FIFO Write Sequence
        4'b0111 : NF <= Nxt_FIFO;           // End of FIFO Read Sequence
        4'b1000 : NF <= Nxt_FIFO;           // Idle State
        4'b1011 : NF <= Nxt_FIFO;           // End of Empty FIFO Write Sequence
        4'b1100 : NF <= Nxt_FIFO;           // Unused State
        4'b1101 : NF <= Nxt_FIFO;           // Unused State
        4'b1110 : NF <= Nxt_FIFO;           // Unused State
        4'b1111 : NF <= Nxt_FIFO;           // Unused State
        default : NF <= FS;
    endcase
end

//  Implement FIFO Select Register

always @(posedge Clk)
begin
    if(Rst)
        FS <= #1 0;
    else
        FS <= #1 NF;
end

//  Implement Control Signals as a function of Current State

always @(*)
begin
    case(CS)
        pReset    : // 00_01_00_00_00
            begin
                RS <= pWCnt;            // Select FIFO Word Counter
                AS <= pClr;             // Clear Register
                MS <= pNOP;             
                CC <= pNOP;
                SF <= pNOP;
            end
        pReset_1  : // 00_00_00_00_00
            begin
                RS <= pNOP;             // Dummy cycle
                AS <= pNOP;
                MS <= pNOP;
                CC <= pNOP;
                SF <= pNOP;
            end
        pReset_2  : // 10_01_00_01_01
            begin
                RS <= pWPtr;            // Select FIFO Write Pointer
                AS <= pClr;             // Clear Register
                MS <= pNOP;         
                CC <= pClr_xRF;         // Clear FIFO Reset Flag
                SF <= pInit_xFlgs;      // Initialize FIFO Flags
            end
        pReset_3  : // 11_01_00_00_00
            begin
                RS <= pRPtr;            // Select FIFO Read Pointer
                AS <= pClr;             // Clear Register
                MS <= pNOP;
                CC <= pNOP;
                SF <= pNOP;
            end
        //
        pWr       : // 10_11_11_11_00
            begin
                RS <= pWPtr;            // Select FIFO Write Pointer
                AS <= pInc;             // Increment Register
                MS <= pWr_FRAM;         // Write FIFO Block RAM
                CC <= pWE_xF;           // Clear FIFO Write Request Flag
                SF <= pNOP;
            end
        pWr_1     : // 00_11_00_00_11
            begin
                RS <= pWCnt;            // Select FIFO Word Counter
                AS <= pInc;             // Increment Register
                MS <= pNOP;         
                CC <= pNOP;
                SF <= pSet_xFF;         // Update FIFO Flag (Set Full Flag)
            end
        //
        pRd       : // 00_10_00_10_10
            begin
                RS <= pWCnt;            // Select FIFO Word Counter
                AS <= pDec;             // Decrement Register
                MS <= pNOP;
                CC <= pClr_RDxF;        // Clear FIFO Read Request Flag
                SF <= pSet_xEF;         // Update FIFO Flag (Set Empty Flag)
            end
        pRd_1     : // 11_11_10_00_00
            begin
                RS <= pRPtr;            // Select FIFO Read Pointer
                AS <= pInc;             // Increment Register
                MS <= pRd_FRAM;         // Read FIFO Block RAM
                CC <= pNOP;             
                SF <= pNOP;
            end
        //
        pIdle     : // 00_00_00_00_00
            begin
                RS <= pNOP;             // Idle Cycle
                AS <= pNOP;
                MS <= pNOP;
                CC <= pNOP;
                SF <= pNOP;
            end
        //
        pWr_EF    : // 10_11_11_11_00
            begin
                RS <= pWPtr;            // Select Write Pointer
                AS <= pInc;             // Increment Register
                MS <= pWr_FRAM;         // Write FIFO Block RAM
                CC <= pWE_xF;           // Clear Write FIFO Request Flag
                SF <= pNOP;
            end
        pWr_EF_1  : // 11_11_10_10_00
            begin
                RS <= pRPtr;            // Select Read Pointer
                AS <= pInc;             // Increment Register
                MS <= pRd_FRAM;         // Read FIFO Block RAM
                CC <= pClr_RDxF;        // Clear Read FIFO Request Flag
                SF <= pNOP;
            end
        pWr_EF_2  : // 00_11_00_00_11
            begin
                RS <= pWCnt;            // Select FIFO Word Count
                AS <= pInc;             // Increment Register
                MS <= pNOP;
                CC <= pNOP;
                SF <= pSet_xFF;         // Set FIFO Flag (Set Full Flag)
            end
        //
        pUnused,
        pUnused_1,
        pUnused_2,
        pUnused_3 : // 00_00_00_00_00
            begin
                RS <= pNOP;             // Dummy Cycles
                AS <= pNOP;
                MS <= pNOP;
                CC <= pNOP;
                SF <= pNOP;
            end
        //
        default   : // 00_00_00_00_00
            begin
                RS <= pNOP;             // Default Cycle
                AS <= pNOP;
                MS <= pNOP;
                CC <= pNOP;
                SF <= pNOP;
            end
    endcase
end

//  Decode ALU Control Field

assign En     = AS[1]; 
assign Add    = &AS;
assign FCR_WE = |AS;

//  Decode FRAM Operation Control Field

assign WE_FRAM = MS[1] &  MS[0];
assign RE_FRAM = MS[1] & ~MS[0];

//  Decode Clear Command Control Field

assign Clr_RTF   =  FS & (CC == 2'b01); // Clear Reset TF Flag
assign R_RDTF    =  FS & (CC == 2'b10); // Generate Delayed CE for TDO
assign WE_TF     =  FS & (CC == 2'b11); // Write TF

assign Clr_RRF   = ~FS & (CC == 2'b01); // Clear Reset RF Flag
assign R_RDRF    = ~FS & (CC == 2'b10); // Generate Delayed CE for RDO
assign WE_RF     = ~FS & (CC == 2'b11); // Write RF

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
