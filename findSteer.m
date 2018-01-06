function steerD=findSteer(xB,yB,xC,yC,xD,yD,thetaB,v,p)
%xB & yB - bike's actual position
%xC & yC - position of current segment
%xD & yD - position of next segment

l=p.l;
g=p.g;

k1=.9; %gain for angle correction
k2=.1; %gain for distance correction  (0.5)
%k3=0;
k3=.1; %gain to prevent oscillations about desired path
%k1 was 2, k3 was 5

%the oscillations parameter is useless because we have the theta-gain,
%which is basically the same thing for small angles (sin(theta) vs theta)
%making k1 larger causes the bicycle to make larger oscillations
%k1=2 gave better results than k1=10;

%what is the angle the bicycle's orientation makes with the desired path?
thetaCD=atan2(yD-yC,xD-xC);
theta=thetaB-thetaCD;

%angle theta should be a number between -pi and pi
if theta<-pi
    theta=theta+2*pi;
elseif theta>pi
    theta=theta-2*pi;
end

%what is the direction of the desired path?
unitPath=[xD-xC,yD-yC,0]/sqrt((xD-xC)^2+(yD-yC)^2);

%where is the bike, if we set point C as the origin?
bikeLoc=[xB-xC,yB-yC,0];

%the distance (shortest path) between the bicycle and the desired path is
%some perpendicular distance d, which we find by taking the cross product
%between the unit vector of the desired path and the vector from the
%bicycle to the origin A 
d=cross(unitPath,bikeLoc);
d=d(3);

%the y-direction velocity for the segment should be minimized
v_y=v*sin(theta);

%the desired steer angle we want the bike to achieve so that it reaches the 
%desired path is dependent on the angle theta and the distance d from the 
%desired path 
steerD=k1*theta+k2*d+k3*v_y;

%we want to set boundaries on the range of steer angles the bike can have
%so that it doesn't have any crazy behavior
maxSteer=pi/3;

if steerD>maxSteer
    steerD=maxSteer;
elseif steerD<-maxSteer
    steerD=-maxSteer;
end

phi=(v^2/(g*l))*steerD;

%if abs(phi)>pi/4
    


end