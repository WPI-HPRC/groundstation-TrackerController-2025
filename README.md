# Ground Station Tracker Controller

This repo contains code for the embedded platforms responsible for running WPI HPRC Ground Station's custom antenna tracking platform.

## There are 3 different deployment bases for the code. 
The `real` base is for the actual hardware tracking system, using a Polaris flight computer and a Teensy Micromod.

The `subscale` base is for the ~1/6th scale development / showcase platform, using a Teensy 3.5.

The `sim` base is currently unimplemented, but is intended to in the future serve hardware-in-the-loop simulation functionality.

**The three different deployment bases can be chosen between using the `Project Environment` configuration in PlatformIO.**
- In VSCode (or derivatives) with the PlatformIO extension, the `Project Environment` can be chosen in the center of the bottom bar.
- If using PlatformIO from the command line, the environment can be selected with the `-e` or `--environment` flags.