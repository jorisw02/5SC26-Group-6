The files in this GitHub Repository are used to generate Figures in the report for Group 6 of the course 5SC26 Systems and Control Integration Project. 

Section 2 of the report is about the First Principle Model of a quadcopter. In this Section, three figures are given where the non-linear and linear model are 
compared with each other around an equilibrium point. This is given in file: Comparison_Nonlinear_Linear_model.m

Section 3 is about system identification of a quadcopter. For this part, the folder Simulink Based Controllers/standardControlStruct is used for this. Using
main_GeneralController.m, in combination with the file nonLin_model_std_V2 the system could be run. Here numerous functions such as makeLinModle are used and 
makeNoise.m. 

Section 4 uses Figures for controller design and comparison. The different controllers are givven in the same folder as for Section 3 in the subfolder _Controllers,
except the MPC controller. This was done by a different team member and uses, therefore, its own folder.

Section 5, swarmflying is investigated. Three different control techniques are compared where the resulting figures are made using the files in Formation_Control.
The ROS implementation to use swarmflying on an actual setup can be found in the subfolder ROS_simulation.
