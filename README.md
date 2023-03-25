# Delco/Bose radio corvette 87-89, brain replace...

What is it, and what does it do.
================================

It is a new implementation of the control functions of the delco/bose radio
for early C4 Corvettes (87-89) Radio.

The code runs on them stm411ce board called blackpill.

But why?

Mainly the radio has US 200Khz spacing, and EU needs 100Khz spacing.
There is a paper about solder a diode to the logic pcb to enable
100Khz spacing, but the cpu DM246 was replaced with DM270
and the board was redesigned around 1987,
So that trick did not work.
Also the diode config also limited upper freq. to 104 Mhz.

Then I will toss the cassette and put bluetooth receiver in its place.
It would be nice to integrate the control of it. with the main controller.
