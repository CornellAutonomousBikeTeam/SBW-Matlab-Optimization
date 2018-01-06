Arundathi Sharma (ams692@cornell.edu)
May 20, 2016
Cornell University Biorobotics and Locomotion Lab
Mechanical Engineering
Autonomous Bicycle Project

findController:
The script findController should be modified to test a variety of controllers 
(the user can select the ranges for k1, k2, k3 and the script builds a matrix 
of all controller combinations)
over a number of initial conditions (same deal--the user can select the 
different lean, steer, and lean rate values and the script will assemble a matrix).
 Alternatively, of course, the
user can manually create these two matrices (one for controllers and one for 
initial conditions).
Ultimately, the script will produce plots of the behavior of the "best"
 controller over many initial conditions, and it will show the user the 
"best controllers" (the number 
of "best controllers" can be set as well). The scoring right now is very 
rudimentary (a simple integral of the bicycle's state); however, later, this should be 
modified to reflect more sophisticated ranking of controllers based on which
 initial conditions they could recover a bike from.