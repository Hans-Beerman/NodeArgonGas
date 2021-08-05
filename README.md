**NodeArgonGas**

NodeArgonGas is a node, used at the MakerSpace Leiden, to verify the state of the Argon Gas bottle, used for welding at the MakerSpace.

See for more information about this node:

[https://wiki.makerspaceleiden.nl/mediawiki/index.php/Project\_Monitoring\_Gasflessen](https://wiki.makerspaceleiden.nl/mediawiki/index.php/Project_Monitoring_Gasflessen)

This repository has the following subdirs:

**Drawings:**

- A schematic drawing, showing the wiring between the PCB used for this node and the external components, like push buttons, the pressure sensor, the RFid reader etc. (in MS Visio, png and pdf format);
- A \*.jpg picture of the PCB;
- Specification of the box in which the PCB and the other components are installed;
- A state diagram of software for this node (in MS Visio and pdf format).

**KiCad\_files:**

The design of the PCB used for the Node/Controller. For this node the NodeStandard PCB V0.8 is used. With less and other components than described for the NodeStandard. The changes are given in the MS Excel file in the root of this directory. There are also 2 subdirectories:

- _NodeArgonGas:_ This directory contains the schematic drawing for the NodeArgonGas and the PCB as used for this node. The design of the PCB in this directory is only made to show how the components for the node are placed on the NodeStandard PCB.
- _NodeStandard:_ This directory contains the design of the NodeStandard V0.8.

**Photos:**

Some photos of the Node made.

**PlatformIO\_Files:**

The source code for the Node. For the development of this software PlatformIO is used, an Extension on Visual Studio Code.