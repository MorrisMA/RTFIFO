Microprogrammed Rx/Tx Block RAM FIFO
=======================

Copyright (C) 2012, Michael A. Morris <morrisma@mchsi.com>.
All Rights Reserved.

General Description
-------------------

This project provides a microprogrammed implementation of dual 
receive/transmit FIFO which uses Block RAM for the storage element. The 
present implementation is based on a Foundation 2.1i SP6 macro designed and 
implemented 8 years ago. That design and implementation has been successfully 
employed in several projects as the FIFO component of 16C550-compatible UARTs. 
This project is an effort to reimplement that implementation in Verilog HDL like 
the UARTs.

Implementation
--------------

The implementation of the core provided consists of a single Verilog source file 
and several memory initialization files:

    RTFIFO.v                - Top level module
    
    RTFIFO_uPgm.coe         - RTFIFO Microprogram
    RTFIFO_Init.coe         - RTFIFO FIFO Control Register (LUT) Initialization
    RTFIFO_BRAM.coe         - RTFIFO Block RAM Initialization

    RTFIFO.ucf              - User Constraints File: period and pin LOCs
    RTFIFO.tcl              - Project settings file
    
    tb_RTFIFO.v             - Completed core testbench with test RAM (under development)
    
    RTFIFO_uPgm.txt         - Memory configuration file of M65C02 test program

Synthesis
---------

The objective for the core is to synthesize such that the FF-FF speed is 100 MHz
or higher in a Xilinx XC2S150-6PQ208 FPGA using Xilinx ISE 10.1i SP3. In that
regard, the core provided meets and exceeds that objective. Using the settings
provided in the RTFIFO.tcl file, ISE 10.1i tool implements the design and
reports that the 9.796 ns period (102 MHz) constraint is satisfied.

The ISE 10.1i SP3 implementation results are as follows:

    Number of Slice FFs:            24
    Number of 4-input LUTs:         96
    Number of Occupied Slices:      65
    Total Number of 4-input LUTs:   104 (8 used as single port LUT-based RAM)

    Number of BUFGMUXs:             1
    Number of RAMB16BWEs            1   (RTFIFO_BRAM.coe)

    Best Case Achievable:           9.796 ns ()

Status
------

Design is complete, and testbench development and verification is currently 
underway.

Release Notes
-------------
