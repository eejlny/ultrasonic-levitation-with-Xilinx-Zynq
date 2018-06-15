# Ultrasonic levitation with Xilinx Zynq 

Department of Electrical Electronic Engineering, 
Bristol University

This project is a collaboration between 2 undergraduate project students
Daniel Connolly-Taylor and William Beasley, one MSc student Brenda Gatusch with supervisors:
Asier Marzo and Jose Nunez-Yanez.

The project aims at creating a low-cost and versatile ultrasonic levitation platform exploting the features
of the Xilinx Zynq device. The features the project uses are: hardware acceleration for the phase delay
calculations, large number of parallel I/Os connected through the FPGA Mezzanine connector (FMC), 
integrated ADC capabilities to capture echo signals and ease of programmability due to a C-based design flow 
for both CPU and FPGA.


The following directories are available in this project:


doc : contains a short and focus report that describes the technology.Also a longer report is a available if more details are necessary.
hw : contains a hardware bit file for zed board
ip : containts the project IP
pcb : contains the pcb schematic files for the phase arrays that are attached to the ZedBoard
in the ultrasonic levitation application
src : containts sources for IP blocks and CPU.
vivado : containts a ready to use Vivado project for version 2017.1

To test the design it is necessary to have access to the created ultrasonic phase arrays
but the following links show different demonstration videos with a circular pattern 
https://youtu.be/GP1x6JX_aTI, echo detection https:// youtu.be/4r_L2R9H_Ws, XY plane movement https:
//youtu.be/q1u2oCEx3Nk and speed test https://youtu.be/pNvneg7r7fM. 
The following github link contains the necessary materials to recreate the project using
a xilinx Zedboard https://github.com/eejlny/ ultrasonic-levitation-with-Xilinx-Zynq including
source code and schematics.