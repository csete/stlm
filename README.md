Sapphire Telemetry System
-------------------------

This repository contains the hardware and software for the Sapphire Telemetry System originally built for Copenhagen Suborbitals for their Sapphire test flight (mission video: http://www.youtube.com/watch?v=kcF5xNrb3HA).

The telemetry system consists of a 2.4 GHz transmitter built around the ADF7242 and a receiver built around a USRP1 with GNU Radio SDR software.

The project repository is organized in the following subdirectories:

* transmitter -- transmitter design files, including complete Eagle project and firmware source code
* receiver -- receiver design files organized as follows:
  * decoder -- The data data decoder application
  * strx -- The main GNU Radio application that acts as the software receiver
  * strx-mon -- Qt-based remote control and monitoring application for strx
  * hardware -- data sheets of varous hardware component (USRP, LNB, etc.)
* doc -- main project documentation written in asciidoc
* system -- some additional system level documentation (should also be avaialble via doc)


License and Copyright
---------------------

The transmitter is Copyright 2013 by Peter Scott, OZ2ABA.

The receiver is Copyright 2013 by Peter Scott, OZ2ABA, and Alexandru Csete, OZ9AEC.

The Viterbi decoder is Copyright Phil Karn, KA9Q,

Both the transmitter and the receiver are licensed under GNU General Public License version 3. For the transmitter other licenses can be negotiated.
