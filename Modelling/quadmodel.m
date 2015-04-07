function [Xout] = quadmodel(Xin)
b=0.001;
d=0.02;
g=9.81;
craft_m= 0.27;
%SYSDYN This function computes ANGEL system response given a state vector and PWM
%input
% Xout = [X_ddot Y_ddot Z_ddot Roll_ddot Pitch_ddot Yaw_ddot X_dot Y_dot
% Z_dot Roll_dot Pitch_dot Yaw_dot]
%
% Xin = [X_dot Y_dot Z_dot Roll_dot Pitch_dot Yaw_dot X Y Z Roll Pitch
% Yaw F_PWM R_PWM B_PWM L_PWM]
%
%
% this loads the necessary constants into the workspace angel;
%---------STATE DEFINITIONS-------------------
X_dot=Xin(1); %X-axis Velocity (m/s)
Y_dot=Xin(2); %Y-axis Velocity (m/s)
Z_dot=Xin(3); %Z-axis Velocity (m/s)
Roll_dot=Xin(4); %Roll Velocity (rad.s^-1)
Pitch_dot=Xin(5); %Pitch Velocity (rad.s^-1)
Yaw_dot=Xin(6); %Yaw Velocity (rad.s^-1)
X=Xin(7); %X position (earth) (m)
Y=Xin(8); %Y position (earth) (m)
Z=Xin(9); %Z position (earth) (m)
Roll=Xin(10); %Roll Angle
Pitch=Xin(11); %Pitch Angle
Yaw=Xin(12); %Yaw Angle
%---------INPUT DEFINITIONS-----------
F = Xin(13); %Front Motor Speed
R = Xin(14); %Right Motor Speed
B = Xin(15); %Back Motor Speed
L = Xin(16); %Left Motor Speed
%--------THRUST CONVERSIONS----------
TF = b*F^2; % front thrust calculation (N)
TR = b*R^2; % right thrust calculation (N)
TB = b*B^2; % back thrust calculation (N)
TL = b*L^2; % left thrust calculation (N)
D = d*(-F+R-B+L);
%--------SYSTEM DYNAMICS------------
% X_ddot = -(1/craft_m)*(cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw))*(TF+TR+TL+TB);
% Y_ddot = -(1/craft_m)*(cos(Roll)*sin(Pitch)*sin(Yaw) + sin(Roll)*cos(Yaw))*(TF+TR+TL+TB);
% z_craft_component = (1/craft_m)*cos(Roll)*cos(Pitch)*(TF+TR+TL+TB)
% Z_ddot = g - z_craft_component;
% Roll_ddot = Pitch_dot*Yaw_dot*(Iyy-Izz)/Ixx + (arm_l/Ixx)*(-TR+TL);
% Pitch_ddot = Roll_dot*Yaw_dot*(Izz-Ixx)/Iyy + (arm_l/Iyy)*(TF-TB);

% Yaw_ddot = Roll_dot*Pitch_dot*(Ixx-Iyy)/Izz + (D)/Izz;
%---------SIMPLIFIED MODEL FOR CONTROL USE (LINEARIZED)
X_ddot = -(1/craft_m)*(cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw))*(TF+TR+TL+TB);
Y_ddot = -(1/craft_m)*(cos(Roll)*sin(Pitch)*sin(Yaw) + sin(Roll)*cos(Yaw))*(TF+TR+TL+TB);
z_craft_component = (1/craft_m)*cos(Roll)*cos(Pitch)*(TF+TR+TL+TB);
Z_ddot = g - z_craft_component;
Roll_ddot = (arm_l/Ixx)*(-TR+TL);
Pitch_ddot = (arm_l/Iyy)*(TF-TB);
Yaw_ddot = (D)/Izz;
%--------FUNCTION OUTPUT-------------
Xout = [X_ddot Y_ddot Z_ddot Roll_ddot Pitch_ddot Yaw_ddot X_dot Y_dot Z_dot Roll_dot
Pitch_dot Yaw_dot];
end