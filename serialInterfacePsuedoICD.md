# Tracker Controller "ICD"

This document will define the Serial communications that should be used to communicate to the motion controller on the antenna tracker.

## Cooordinate frames & terminology

"Pose" describes a currently desired location/position for the overall tracker.

"Axis" describes information that is specific to one degree of freedom of the system.
These axes are:
* Azimuth [Pan] - Rotation around the Z axis in a standard cartesian coordinate global frame.
* Elevation [Pitch] - Rotation around the X axis in a standard cartesian coordinate frame based on the section rotated around the 
    azimuth axis.

Within the tracker, the "body" frame describes the orientation of the clamping assembly that mounts the antenna.
* Y+ is defined as the direction that will align with the maximum directionality of the mounted antenna.
* X+ is defined along the direction of the axle that the clamping assembly rotates with, in the opposite direction as the potentiometer used to provide position feedback.
* Z+ is defined in respect to the previous two axes, in a standard right-hand cartesian coordinate frame. 
    * The plane of the clamping assembly that is in the direction of Z+ should be the plane that any antenna should be attached to.
* The origin of this coordinate system is placed in the center of the axle that the clamping assembly rides on in X & Z, and placed center within the clamping assembly in Y.
    

## Seperation of responsibilities:

**The tracker controller will be responsible for:**
* Providing GPS and IMU data to the connecting system when requested.
    * The intent is that this will be requested once the tracker is setup and secured physically, then this data can be used to determine the location of the tracker, as well as it's current orientation to correct for any unlevelness in the installation.
* Receiving desired pose information in a format laid out further down in this document.
* Driving the system (closing the loop) to the desired pose.
* Providing positional feedback on the current pose of the system when requested.

**The interfacing system will be responsible for:**
* Determing the target pose for the tracker, given any and all input data.
    * __This includes performing all transforms based on the GPS or IMU data provided by the tracker controller.__



# Serial Interface

**Note: Responses to commands may not be the next response received.**

## Setting Desired Pose:
To set the desired pose, the following message format should be used:
```
S;P;XXXXX,YYYYY;E
```
* `XXXXX` represents the desired reading in degrees for the azimuth axis, multiplied by 100 to achieve a fixed point accuracy of 1/100th of a degree. 
* `YYYYY` represents the desired reading in degrees for the elevation axis, multiplied by 100 to achieve a fixed point accuracy of 1/100th of a degree. 

Both axes are positive only, with individual maximum ranges and zero points defined per axis below.
The axes are the same as are used in the get command.

This will then return a message at earliest convinence in the following form:
```
R;P;E
```
This messages acknowledges that the new desired pose has been read, and has been set as the new goal.

**TO:DO: determine and document maximum ranges for both axes, and define them in this document.**

## Getting Current Pose:
To receieve the current pose/location, the following message format should be used:
```
G;L;E
```
This will then return a message at earliest convinence in the following form:
```
D;L;XXXXX,YYYYY;E
```
* `XXXXX` represents the desired reading in degrees for the azimuth axis, multiplied by 100 to achieve a fixed point accuracy of 1/100th of a degree. 
* `YYYYY` represents the desired reading in degrees for the elevation axis, multiplied by 100 to achieve a fixed point accuracy of 1/100th of a degree. 

Both axes are positive only, with individual maximum ranges and zero points defined per axis below.
The axes are the same as are used in the set command.


## Getting GPS Data
To receieve the current GPS data, the following message format should be used:
```
G;G;E
```
This will then return a message at earliest convience in the following form:
```
D;G;XXXXX,YYYYY,ZZZZZ;E
```
* `XXXXX` represents the longitude in ????
* `YYYYY` represents the latitude in ????
* `ZZZZZ` represents the altitude in ????

**TO:DO: Figure out logical and easy-to-use units for these. Units cannot incude `;` to prevent parsing errors.**


## Getting IMU Data
To receieve the current data from the IMU in the form of a gravity vector and a compass heading, the following message format should be used:
```
G;I;E
```
This will return a message at earliest convinence in the following form:
```
D;I;XXXXX,YYYYY,ZZZZZ,HHHHH;E
```
* `XXXXX` represents the X component of the gravity vector in ????
* `YYYYY` represents the Y component of the gravity vector in ????
* `ZZZZZ` represents the Z component of the gravity vector in ????

**TO:DO: Figure out logical and easy-to-use units for these. Units cannot incude `;` to prevent parsing errors.**

## These following methods both can fall under a software "E-Stop" functionality. 

### **Please have a careful understanding of the difference between the two methods of stopping movement before using/running the tracker.**
***In general;*** the __braking__ method should be used as an "E-stop" method. 

The current intended use for the __coasting__ method is preventing damage to the tracker mechanisms in high wind conditions, by allowing the tracker to act in a similar fashion as a wind vane. 

### Braking the Tracker
**To cause the tracker to stop all Zcurrent movements and use the motors to hold the current position**, the following message format should be used:
```
G;B;E
```
This will return a message **as soon as possible** in the following form:
```
D;B;E
```
Acknowleging the brake command and notifiying that the tracker is attempting to stop movement.

### Coasting the Tracker
**To cause the tracker to stop all current movements and allow mechanisms to rotate freely**, the following message format should be used:
```
G;C;E
```
This will return a message **as soon as possible** in the following form:
```
D;C;E
```
Acknowleging the coast command and notifiying that the tracker has stopped commanding motors.


P
L
G
I
B
C