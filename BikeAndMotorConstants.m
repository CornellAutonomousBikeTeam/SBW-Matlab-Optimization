% Constants

function CONST=BikeAndMotorConstants()
%% Constants

%%%%%%%% Bicycle %%%%%%%

w=1.02;         %Wheelbase (m)
c=0.08;         %Trail (m)
tilt=pi/10;     %tilt (rad)
g=9.81;         %gravity (m/(s^2))
%v=6;            %forward velocity (m/s)
v=3.57;            %forward velocity (m/s)

%Rear Wheel R

r_R=0.1905;         %radius (m)
m_R=6.5;           %mass (kg)
I_Rxx=0.0603*m_R/2;    %Moment of inertia(kg m^2)
I_Ryy=0.12*m_R/2;      %Moment of inertia(kg m^2)

%Rear Body and Frame Assembly B

x_B=0.3;
z_B=-0.9;       %Position of center of mass (m)

m_B=15.65;         %mass (kg)

I_Bxx=9.2*m_B/85;      %kg m^2
I_Byy=11*m_B/85;       %kg m^2
I_Bzz=2.8*m_B/85;      %kg m^2
I_Bxz=2.4*m_B/85;      %kg m^2

I_B=[I_Bxx,0,I_Bxz;...
     0,I_Byy,0;...
     I_Bxz,0,I_Bzz];

%Front Handlebar and Fork Assembly H

x_H=0.9;
z_H=-0.7;       %Position of center of mass (m)

m_H=4;          %mass (kg)

I_Hxx=0.05892;      %kg m^2
I_Hyy=0.06;         %kg m^2
I_Hzz=0.00708;      %kg m^2
I_Hxz=-0.00756;     %kg m^2

I_H=[I_Hxx,0,I_Hxz;...
     0,I_Hyy,0;...
     I_Hxz,0,I_Hzz];

%Front Wheel F

r_F=0.1905;        %radius (m)
m_F=1.81;           %mass (kg)
I_Fxx=0.1405*m_F/3;    %Moment of inertia(kg m^2)
I_Fyy=0.28*m_F/3;      %Moment of inertia(kg m^2)

%Whole Bike

m_T = m_R + m_B + m_H + m_F;            %Total mass

x_T = (m_B*x_B + m_H*x_H + m_F*w)/m_T;               %Total center of mass
z_T = (-r_R*m_R + z_B*m_B + z_H*m_H -r_F*m_F)/m_T;   %(wrt contact point P)

%%%%%%%%% Motor %%%%%%%%%

%Motor Constants
b=1.5*(60*0.000720077887)/(1000*2*pi);     %damping (kg*m*s/rad)
J=6*(0.000720077887*10^-3);     %Inertia (kg*m*s^2)
Ra=5.88;    %Armature Resistance (ohm)
Kb=20/(1000*2*pi/60);    %Back Emf Constant (V*s/rad)
La=0.008;    %Armature Inductance (Henies)
Km=27.1*(0.000720077887);    %Motor Torque Constant (kg*m/amp)

%%%%%%%%% Gear Ratio %%%%%%%%%%

%Gear Ratio
n_s=2;
n_m=1;

%% Put Into Struct

%%%%%% Bicycle %%%%%%%

CONST.w=w;         %Wheelbase (m)
CONST.c=c;         %Trail (m)
CONST.tilt=tilt;     %tile (rad)
CONST.g=g;         %gravity (m/(s^2))
CONST.v=v;            %forward velocity (m/s)

%Rear Wheel R

CONST.r_R=r_R;         %radius (m)
CONST.m_R=m_R;           %mass (kg)
CONST.I_Rxx=I_Rxx;    %Moment of inertia(kg m^2)
CONST.I_Ryy=I_Ryy;      %Moment of inertia(kg m^2)

%Rear Body and Frame Assembly B

CONST.x_B=x_B;
CONST.z_B=z_B;       %Position of center of mass (m)

CONST.m_B=m_B;         %mass (kg)

CONST.I_Bxx=I_Bxx;      %kg m^2
CONST.I_Byy=I_Byy;       %kg m^2
CONST.I_Bzz=I_Bzz;      %kg m^2
CONST.I_Bxz=I_Bxz;      %kg m^2

CONST.I_B=I_B;

%Front Handlebar and Fork Assembly H

CONST.x_H=x_H;
CONST.z_H=z_H;       %Position of center of mass (m)

CONST.m_H=m_H;          %mass (kg)

CONST.I_Hxx=I_Hxx;      %kg m^2
CONST.I_Hyy=I_Hyy;         %kg m^2
CONST.I_Hzz=I_Hzz;      %kg m^2
CONST.I_Hxz=I_Hxz;     %kg m^2

CONST.I_H=I_H;

%Front Wheel F

CONST.r_F=r_F;        %radius (m)
CONST.m_F=m_F;           %mass (kg)
CONST.I_Fxx=I_Fxx;    %Moment of inertia(kg m^2)
CONST.I_Fyy=I_Fyy;      %Moment of inertia(kg m^2)

%Whole Bike

CONST.m_T = m_T;            %Total mass
CONST.x_T = x_T;               %Total center of mass
CONST.z_T = z_T;   %(wrt contact point P)

%%%%%%%% Motor %%%%%%%%

CONST.b=b;     %damping (kg*m*s/rad)
CONST.J=J;     %Inertia (kg*m*s^2)
CONST.Ra=Ra;    %Armature Resistance (ohm)
CONST.Kb=Kb;    %Back Emf Constant (V*s/rad)
CONST.La=La;    %Armature Inductance (Henies)
CONST.Km=Km;    %Motor Torque Constant (kg*m/amp)

%%%%%%% Gear Ratio %%%%%%%%

CONST.n_s=n_s;
CONST.n_m=n_m;

end
