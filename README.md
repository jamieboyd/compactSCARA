# compactSCARA
<H1>Documentation for the Compact SCARA</H1>
This document describes the construction, programming, and operation of the compact Selective Complicance Robot Arm (SCARA). By analogy with a human arm, in the following discussion L1 refers to the upper arm segment, and L2 refers to the lower arm segment. The proximal end of L1 is fixed, and its distal end forms an elbow joint with L2. The distal end of L2 holds a tool of some sort, and most of the function of the robot arm involves moving the tool to defined locations. Each arm segment is driven by a separate motor, and its angular position is measured via quadrature decoding 
<H2>MSP430 Resources</H2>
The compact SCARA is controlled by the msp430 f5529 microcontroller on an MSP-EXP430F5529 launchpad evaluaton board. The following is a list of the timers, pins, and interrupts used by the compact SCARA.

<H3>Motors</H4>
Pulse Width Modulation (PWM) is used to control the drive strength to the motors. Pulse width timing is done with timer A 0, using its associated interrupt. Thus, these pins need to be able to output a timing signal.
<ul>
<li>L1 is controled by CCR3 on P1.4</li>
<li>L2 is controlled by CCR4 on P1.5</li>
</ul>
Direction of movement is controlled by outputs to the selection pins on the H-bridge motor driver. When INA is high and INB is low, the connected motor turns counter-clockwise; when INA is low and INB is high, the motor turns clockwise. Be aware that the motor for L2 is mounted upside down relative to the motor for L1. In the code, the meaning of clockwise and counter-clockwise for L2 is inverted so that clockwise and counter-clockwise are always defined relative to a position above the SCARA arm. INA and INB for both motors are done using pins connected to port 3.
<ul>
<li>INA L1 on P3.0</li>
<li>INB L1 on P3.1</li>
<li>INA L2 on P3.2</li>
<li>INB L2 o P3.3</li>
</ul>

<H4>Quadrature Decoding</H4>
Quadrature decoding is done using pins on port2. Each motor has two quadrature channels, channel A and channel B, which are monitored on the port 2 interrupt.  As for the motors, the inversion of the encoder for L2 is accounted for in the code so that, for both motors, counts become more positive with counter-clockwise rotation defined from a perspective above the SCARA arm.
<ul>
<li>Chan A L1 on P2.2</li>
<li>Chan B L1 on P2.3</li>
<li>Chan A L2 on P2.6</li>
<li>Chan B L2 on P2.7</li>
</ul>
The 4 states of the quadrature encoder are defined by the high and low states of channel A and channel B:
<pre>
	Encoder State = 0 to 3
	A ____|--------|________|----
	
	B _________|--------|________
	    0   1    2   3    0    1
</pre>

<H4>Z-axis Control</H4>
The Z-axis is controlled using a steper motor with a motor driver that has STEP, DIRECTION, and ENABLE inputs. The steps are generated using Timer A 1 and uses the Timer A 1 interrupt to track position. The direction and enable are controlled with GPIO outputs.
<ul>
	<li>Z-axis STEP on 2.0</li>
	<li>Z-axis DIRECTION on P3.4</li>
	<li>Z-axis ENABLE on P3.5</li>
</ul>

<H4>Emergency Stop and Limit Switches</H4>
The emergency stop system consists of four limit switches, a physical emergency stop button, and an optoisolator used as a GPIO-driven switch triggerable from software, all connected in series to a physical latching circuit. When any switch in the series is interrupted, even momentarily, output is latched low until a reset input is pulsed from high to low to high. The output of the latch goes directly to the enable outputs of the arm drive motors and the Z stepper motor, so they are immediately stopped. The stopped signal from the latch is reset by a GPIO output from the microcontroller. The emergency stop input is done on port 1, using the port 1 interrupt. The other signals are done on port 6
<ul>
<li>Emergency Stop input on P1.6</li>
<li>Emergency Stop reset on P6.5</li>
<li>Software Stop on P6.6</li>
<li>Emergency Stop LED on P6.1</li>
</ul>

<H4>Movement Timing Loop</H4>
The timing loop uses no pins, but does make use of Timer B.

<H4>Tool Control</H4>
Given that it is difficult to predict what tools might be used with the compact SCARA, care was taken to leave as many resources free for tool usage as possible. Interrupt enabled pins are available on port1 and port 2. Timer A 2 and its pins are left untouched for tool use. The pins for the I2C port have not been used for GPIO and are available for I2C controlled hardware on a tool.<br>
	
<IMG SRC="msp430PinUsage.png" ALT="msp430 launchpad pin usage for compact SCARA" HEIGHT =481 WIDTH =1174><br>
Figure 1: Pin usage on the MSP-EXP430F5529 launchpad evaluaton board for the different functional modules of the compact SCARA. Pins used are circled with a colour corresponding to the functional module (See legend). Pins using PWM ouput are marked with an asterisk, and pins using an interrupt are marked with an exclamation point. Uncircled pins are available for use by a tool. Modified from <a href = "https://www.ti.com/lit/ug/slau536/slau536.pdf">Texas Instruments slau536</a>.

<H3>Code Modules</H3>
The code that runs compact SCARA is written in C in Code Composer Studio. It is organized nto the folowing modules which roughly correspond to the functional modules.
<table cellspacing="0" cellpadding="0" border = "1">
	<caption>C Code Modules for Compact SCARA</caption>
<tbody>
	<tr>
	      <td>SCARA</td>
		  <td>Contains the main function, global variables, and constants</td>
	  </tr>
	  <tr>
		  <td>UcsControl</td>
		  <td>Uses the msp430 universal clock system to set frequency of main clock to 20 MHz</td>
	  </tr>
	  <tr>
		  <td>libUART1A</td>
		  <td>Interrupt-driven code to send and receive data over the serial port at 115200 baud</td>
	  </tr>
	  <tr>
		  <td>BinaryCmdInterp</td>
		  <td>Uses libUART1A to get commands from a host computer and return results</td>
	  </tr>
	  <tr>
		  <td>motorsPWM</td>
		  <td>Uses Timer A 0 to generate PWM signals to drive the motors, and sets direction with GPIO outputs to the motor driver's direction inputs</td>
	  </tr>
	  <tr>
		  <td>quadEncDec</td>
		  <td>Interrupt functions to decode quadrature inputs for the two motors into positions</td>
	  </tr>
	  
</tbody>
</table>
