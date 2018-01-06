function [success, p, steerCheck, state, tarray, steerDvect] = mainNavigationArundathi(x,y,v,delta0,phi0,graph,nav)
   %given a set of waypoints that the bicycle has to pass through, and a
   %set of velocities that we expect the bicycle to have in each of the
   %segments between the waypoints, we want to find out what command to
   %send to the steering motor so that the bicycle follows the path and
   %stays balanced
   
   %define parameters
   p.g = 9.81; p.l = 0.935; %p.b=0.5; p.h=0.5;
   p.b = 0.3; p.h = 0.5156;
   p.c = 0;   %trail is 0
   
   %gains
   p.k1 = 10;
   p.k2 = 70;
   p.k3 = 0;
   %k3 should have a different sign from other two gains.
   
   p.timestep = 1/60;
   p.pause = p.timestep;
    
   %assume that we start out in the first segment
   tstep=p.timestep;
   tarray=[];
   currentSegment=1;
   xC=x(currentSegment); yC=y(currentSegment); xD=x(currentSegment+1); yD=y(currentSegment+1);
   
   
   %once segment is chosen...
   %check whether bike has crossed boundary

%% Check continually whether the threshold within a segment has been passed; when that happens, the next segment becomes target
    
   %get information from sensors about state of bicycle
   xB=0;          %xB and yB represent the current position of the bike
   yB=0;
   thetaB = 0;    %thetaB=pi/8; xB,yB,thetaB taken from GPS data
   yaw0=0;        %heading information not actually necessary, but we keep it and treat the desired path (if straight line on x-axis) as psi=0
   phiDot0=0;
   steerD = 0;
   initialConditions=[xB,yB,phi0,yaw0,delta0,phiDot0,v];
   state=initialConditions;
   
%% This bit cycles as the program updates the location and updates the controller to fix behavior of the bicycle

    %decides which segment it is in
    %how do we choose a segment?
    %how do we know if we are off course at all?
    [dis,ind]=min((xB-x).^2+(yB-y).^2); %take the minimum distance between the location of the bicycle and any waypoint
    currentSegment=ind;
    xC=x(currentSegment); yC=y(currentSegment); xD=x(currentSegment+1); yD=y(currentSegment+1);

    k=1;
    tarray(1)=0;
    
    %Set success = 1 to start. Assuming the test will be successful,
    %unless the bike crashes.
    success = 1;
    
    %Used to check convergence of actual steer angle to desired steer
    %angle.
    steerCheck = 0;
    steerDvect = [];
    
   %Uncomment to choose desired while loop setting:
    while sqrt(xB^2 + yB^2) < x(end) %"while the actual distance of the 
                                     %bike is less than the maximum x distance" 
                                     %(used for balance control in straight line).
                                     
    %while k <= 1000    %use for quick, constant integration time for all trials
    
    %while currentSegment<length(x)-1  
    
    %for k=1:length(x)/tstep
        steerDvect(k) = steerD;
        
        xB = state(k,1);
        yB = state(k,2);
        phiB = state(k,3);
        deltaB = state(k,5);
        
        if abs(steerD - deltaB) <= 0.001
            steerCheck = 1;
        end
        
        %if the lean angle is too high, the test should count as a failure
        if abs(phiB)>=pi/4
            if graph == 1
                fprintf('bike has fallen; test failure\n')
            end
            success = 0;
            break;
        end
        
        thetaB=state(k,4); %***CHANGE THETA TO PSI. WTF IS THETA
        v=state(k,7);
                
        %if boundary crossed, choose next segment
        threshold=1;  
        

        %figure out if we've crossed threshold; take the dot product between
        %the unit path and the distance from the bike to the origin of the
        %target segment
        pathLength=sqrt((xD-xC)^2+(yD-yC)^2);
        unitPath=[xD-xC,yD-yC,0]/pathLength;
        bikeLoc=[xB-xC,yB-yC,0];    %???
        dist=dot(unitPath,bikeLoc); %???
        
        %if that threshold is passed, the bicycle should aim for the next
        %segment
        if dist>pathLength-threshold
            currentSegment=currentSegment+1;
            if currentSegment==length(x)
                break;
                fprintf('reached end of path');
            end
            xC=x(currentSegment); yC=y(currentSegment); xD=x(currentSegment+1); yD=y(currentSegment+1);
        end


       %update time in time array
       time = k*tstep;
       tarray(k) = time;
       
       %calculates desired steer, based on our target segment
       if nav == 1
          steerD=findSteer(xB,yB,xC,yC,xD,yD,thetaB,v,p);
       else
          steerD=0;
       end
       navCommands(k)=steerD;

       %use all the sensor data and first feedback controller output (desired
       %steer angle) to find u, using the rhs function below:
       [zdot,u]=rhs(state(k,:),steerD,p,time);
       motCommands(k)=u;

       %add the current state to the array of states that the bike has experienced
       %over the testing period
       state(k+1,:) = state(k,:) + zdot*tstep;
       
       k=k+1;
       
    end

   close all
   
   if graph == 1
       simulateBike(state,tarray,x,y,p,navCommands,motCommands);
       %update xB,yB,other state variables (unpack state(end))
   end
   
end
   