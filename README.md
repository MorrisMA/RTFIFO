Microprogrammed Rx/Tx Block RAM FIFO
=======================

Copyright (C) 2012, 2015 Michael A. Morris <morrisma@mchsi.com>.
All Rights Reserved.

General Description
-------------------

This project provides a microprogrammed implementation of dual 
receive/transmit FIFO which uses Block RAM for the storage element. The 
present implementation is based on a Foundation 2.1i SP6 macro designed and 
implemented 8 years ago. That design and implementation has been successfully 
employed in several projects as the FIFO component of 16C550-compatible UARTs.
 
This project is an effort to reimplement that Rx/Tx FIFO implementation in 
Verilog HDL like the UARTs.

Implementation
--------------

The implementation of the core provided consists of a single Verilog source file 
and several memory initialization files:

    RTFIFO.v                - Top level module
    
    RTFIFO_uPgm.coe         - RTFIFO Microprogram ROM Memory Configuration File
    RTFIFO_Init.coe         - RTFIFO FIFO Control Register (LUT) Initialization
    RTFIFO_BRAM.coe         - RTFIFO Block RAM Initialization

    RTFIFO.ucf              - User Constraints File: period and pin LOCs
    RTFIFO.tcl              - Project settings file
    
    tb_RTFIFO.v             - Completed core testbench with test RAM (under development)
    
    RTFIFO_uPgm.txt         - Microprogram Source File

Synthesis
---------

The objective for the core is to synthesize such that the FF-FF speed is 100 MHz
or higher in a Xilinx XC2S150-6PQ208 FPGA using Xilinx ISE 10.1i SP3. In that
regard, the core provided meets and exceeds that objective. Using the settings
provided in the RTFIFO.tcl file, the ISE 10.1i tool implements the design and
reports that the 9.761 ns period (102 MHz) constraint is satisfied.

The ISE 10.1i SP3 implementation results are as follows:

    Number of Slice FFs:            24
    Number of 4-input LUTs:         97  (14 : LUT-based uPgm ROM)
    Number of Occupied Slices:      66
    Total Number of 4-input LUTs:   105 ( 8 : SP LUT-based RAM  )

    Number of BUFGMUXs:             1
    Number of BlockRAMs             1   (RTFIFO_BRAM.coe)

    Best Case Achievable:           9.704 ns (0.296 ns SETUP; 2.060 ns HOLD )

Status
------

Design is complete, and testbench development and verification is currently 
underway.

Release Notes
-------------

In addition to providing an example of a microprogrammed Rx/Tx FIFO 
controller, the repository includes an example of a non-microprogrammed, 
standard controller for a Rx/Tx FIFO. The example of a standard controller 
demonstrates how resource effecient a microprogrammed Rx/Tx FIFO controller can
be in a RAM-based FPGA.

The standard Rx/Tx FIFO controller provided is not a contrived example. With 
the the exception that it uses the same number of registers as the 
microprogrammed implementation, the standard state machine approach uses an 
RTL implementation for the registers (word counter, read pointer, and write 
pointer) and for the state machine. Although specific encoding of the states 
of the controller has been used, the synthesizer has not been constrained in 
any manner. Thus, after analysis of the RTL implementation, the synthesizer 
has opted to convert the sequential state assignments provided in the code 
into a one-hot implementation of the state machine. Furthermore, two of the 
encoded control fields have also been synthesized as one-hot fields.

These two optimizations, automatically applied by the synthesizer, will result 
in state transition and control logic equations which are more efficient than 
a similar implementation which uses encoded states and control fields. Thus, 
no special effort was expended to keep the synthesizer from optimizing the
standard state machine implementation as much as possible.

The following table summarizes the resource utilization for the optimized, 
standard state machine approach:

The ISE 10.1i SP3 implementation results are as follows:

    Number of Slice FFs:            105
    Number of 4-input LUTs:         178
    Number of Occupied Slices:      146
    Total Number of 4-input LUTs:   179 (1 used for route-through)

    Number of BUFGMUXs:             1
    Number of BlockRAMs             1   (RTFIFO_BRAM.coe)

    Best Case Achievable:           11.316 ns (N/A ns SETUP; 2.939 ns HOLD)

Comparing the two results shows that the microprogrammed implementation has 
some distinct advantages over the RTL-only approach. It is possible to take 
advantage of some of the benefits of the microprogrammed state machine 
approach without resorting to a microprogrammed state machine. For example, it 
is possible to use a LUT-based, distributed RAM to implement the the 
registers. This optimization alone will reduce the number of registers/FFs 
from 105 to approximately 57. It will not, however, come close to matching the 
better than 4:1 reduction in registers that the microprogrammed approach 
provides. Neither will it achieve the better than 2:1 in the number of 
occupied slices that the microprogrammed approach achieves.

The microprogrammed Rx/Tx FIFO provided in this repo is a design approach that 
should be carefully considered when using RAM-based FPGAs. In many 
situtations, a standard, RTL-based approach fails to take full advantage of the 
features of RAM-based FPGAs, and will result in inefficient use of the FPGA. 
Given the cost of implementing any design in an FPGA and the cost of the 
device itself, designers should seek to maximize the amount of circuitry/logic 
that can be incorporated into an FPGA. In many cases, a microprogrammed 
approach will result in a faster, more resource efficient implementation. This 
alone may be sufficient to reduce the size of the device shipped in a product, 
or it may allow an existing product to include additional features prior to or 
after shipment. 

As a final comment. Given the dual port nature of most FPGA block RAMs, or the 
ease with which a dual-port capability can be incorporated into distributed, 
LUT-based RAM, it is possible to incorporate some form of writable control 
store in virtually any microprogrammed state machine implementation. With some 
forethought and planning, FPGAs incorporating complex state machines can 
easily be configured to be in-circuit re-writable. The potential flexibility 
that a writable microprogram control store provides can not be easily 
quantified. In other words, it is potentially "priceless".

Corrections
--------

##1 Update 1 - 14 Mar 2015
While building a simulation, found an error in the definition of the ROM that 
holds the microprogram. In the original release, ROM was defined as a wire. 
Synthesis of the module did not report any unexpected errors or warnings 
regardin ROM. However, ISim would terminate with an access violation 
exception. After a bit of work to isolate the issue, the definition of ROM was 
changed from a wire to a reg. This change cleared up the ISim access violation 
exception. Resynthesizing to module yielded the same results. Apparently 
synthesis is more tolerant than ISim.

## Update 2 - 15 Mar 2015
Completed the self-checking testbench. Found two errors in the microprogrammed 
SM implementation: (1) width of the summer set to match the width of the BRAM 
address instead of the required width of the FIFO address; (2) used to wrong 
signal to create the delayed output register write strobe. Also found one 
error in the standard SM version, after synchronization with the 
microprogrammed version: the FIFO select signal had been left out of the 
address for the FIFO RAM. Both versions now pass the testbench, and have a 
state diagram that matches the original implementations.
