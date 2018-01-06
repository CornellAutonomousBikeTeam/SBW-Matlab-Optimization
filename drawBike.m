function [COG_hand,SH_hand,CPfw_hand,CPrw_hand]=drawBike(x,y,z,yaw,roll,steer,p)

%function taken directly from Kate's animation code, based on Diego's
%animation code--meant to accurately draw the bicycle in the animation
%simulation, based on bicycle and motor parameters

%% Bicycle Constants

roll=-roll; %why?? into the board vs. out of the board

CONST=BikeAndMotorConstants;

w = p.l;

c = p.c;


% w=CONST.w;         %Wheelbase (m)
% c=CONST.c;         %Trail (m)
% tilt=CONST.tilt;     %tile (rad) ? What is tilt?

%Rear Wheel R

r_R=0.1905;         %radius (m)

%Rear Body and Frame Assembly B

% x_B=CONST.x_B;
% z_B=CONST.z_B;       %Position of center of mass (m)

%Front Handlebar and Fork Assembly H

% x_H=CONST.x_H;
% z_H=CONST.z_H;       %Position of center of mass (m)

%Front Wheel F

r_F=r_R;        %radius (m)

%Whole Bike
% center of mass
x_T = CONST.x_T;   %Total center of mass
z_T = CONST.z_T;   %(wrt contact point P)

% Bike Constants
Handh=0.5;
HandW=0.3;

%%  Dynamic Variables

% x=2;
% y=0;
% z=0;
% yaw=0;
% 
% roll=pi/8;
% steer=-pi/4;

% Coordinate Transformation Matrices

A_OP=[cos(yaw),-sin(yaw),0,x;...
      sin(yaw),cos(yaw),0,y;...
      0,0,1,z;...
      0,0,0,1];
  
A_PG=[1,0,0,x_T;...
     0,cos(roll),-sin(roll),-abs(z_T)*sin(roll);...
     0,sin(roll),cos(roll),abs(z_T)*cos(roll);...
     0,0,0,1];
 
A_GWF=[cos(steer),-sin(steer),0,(-x_T+w);...
       sin(steer),cos(steer),0,0;...
       0,0,1,(-abs(z_T)+r_F);...
       0,0,0,1]...
       *...
       [1,0,0,0;...   
       0,cos(pi/2),-sin(pi/2),0;...
       0,sin(pi/2),cos(pi/2),0;...
       0,0,0,1];
   
    
A_WFS=[1,0,0,0;...   
       0,cos(pi/2),-sin(pi/2),Handh;...
       0,sin(pi/2),cos(pi/2),0;...
       0,0,0,1];
   

A_GWR=[1,0,0,-x_T;...   
       0,cos((pi/2)),-sin((pi/2)),0;...
       0,sin((pi/2)),cos((pi/2)),-abs(z_T)+r_R;...
       0,0,0,1];
   

%% Plot Bicycle Parts

%Draw Center of Gravity

COG_G=[0;0;0;1];

COG_P=A_PG*COG_G;
COG_O=A_OP*COG_P;

COG_hand=plot3(COG_O(1),COG_O(2),COG_O(3),'*b');

%Draw Steering Wheel

SH_S=[[0;(HandW/2);0;1],[0;(-HandW/2);0;1]];

SH_WF=A_WFS*SH_S;
SH_G=A_GWF*SH_WF;
SH_P=A_PG*SH_G;
SH_O=A_OP*SH_P;

SH_hand=plot3(SH_O(1,:),SH_O(2,:),SH_O(3,:),'k');

%Draw Front Wheel [WF coordinate]

CPfw=CircleAboutZ(r_F);

CPfw=[CPfw;ones(size(CPfw(1,:)))];

CPfw_G=A_GWF*CPfw;
CPfw_P=A_PG*CPfw_G;
CPfw_O=A_OP*CPfw_P;

CPfw_hand=plot3(CPfw_O(1,:),CPfw_O(2,:),CPfw_O(3,:),'k');


%Draw Back Wheel [WR coordinate]

CPrw=CircleAboutZ(r_R);

CPrw=[CPrw;ones(size(CPrw(1,:)))];

CPrw_G=A_GWR*CPrw;
CPrw_P=A_PG*CPrw_G;
CPrw_O=A_OP*CPrw_P;

CPrw_hand=plot3(CPrw_O(1,:),CPrw_O(2,:),CPrw_O(3,:),'k');

end