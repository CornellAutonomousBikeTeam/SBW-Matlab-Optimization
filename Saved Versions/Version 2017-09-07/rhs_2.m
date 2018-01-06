function [zdot,u]=rhs(state,steerD,p,time,ks)

%unpack parameters
g=p.g; l=p.l; b=p.b; h=p.h;

if nargin == 4
    c1=p.k1; c2=p.k2; c3=p.k3;
elseif nargin == 5
    c1 = ks(1); c2 = ks(2); c3 = ks(3);
end

%unpack state
xB=state(1);
yB=state(2);
phi = state(3);

%At every second, add a sensor error to lean angle of 1 degree or pi/180
%rad.
if rem(ceil(time),2) == 1
    phi = phi + pi/180;
end

Psi=state(4);
delta=state(5);
w_r=state(6);
v=state(7);

%what are our gains?
% c1=75;
% c2=13;
% c3=-11;

%assign values to time derivatives of all initial conditions, use state
%variables (inputs) to define these. %that, as well as control output u, 
%is the function output; this information will go into the simulation 
%function, which will integrate using Euler's method and describe the 
%behavior of the bike over time. 

%c3 should have a different sign from the other gains.
u=c1*phi+c2*w_r+c3*(delta-steerD);

%set limit for maximum allowable steer rate
if u>10
    u=10; %rad/s         
elseif u<-10
    u=-10;
end

% if abs(u)<2
%     u=0;
% end

xdot=v*cos(Psi);
ydot=v*sin(Psi);
phi_dot=w_r;
psi_dot=(v/l)*(tan(delta)/cos(phi));
delta_dot=u;
v_dot=0;
wr_dot=(-v^2*delta-b*v*u+g*l*phi)/(h*l);
%wr_dot=(1/h)*(g*sin(phi) - tan(delta).*(v.^2/l + b*v_dot/l + tan(phi).*((b/l)*v.*phi_dot-(h/l^2)*v.^2.*tan(delta)))-b*v.*delta_dot./(l*cos(delta).^2));

%now we have all our terms in a vector representing the rhs
zdot=[xdot,ydot,phi_dot,psi_dot,delta_dot,wr_dot,v_dot];


