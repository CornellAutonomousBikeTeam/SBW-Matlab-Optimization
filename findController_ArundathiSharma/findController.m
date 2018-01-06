%have MATLAB cycle through many different controllers (allow range of
%0-200 for each gain value)

%robustness evaluation: make a list of errors: bike parameters (b, l, h),
%sensor errors (implement a DC offset), actuator errors (some linear error,
%i.e., constant+command*constant2)
%make sure bicycle does not fall down under any of these circumstances

% mostStable=inf;
% bestController=[0 0 0];
clear all
close all
tstep=1/100; %establish our time step
tf=5;
tspan=linspace(0,tf,tf/tstep); %we maintain that time step and analyze behavior over some period of time
p.g=9.81; p.deltaD=0;
p.l= 1.02; p.b=0.3; p.h=0.9; %bike geometry parameters, taken from rear frame for this model


%each row is: xD,yD,psi_0,phi_0,delta_0,vD,phi_dot0
% phis=[pi/100, pi/10 pi/8, pi/6, pi/4];
% deltas=[pi/100, pi/10, pi/8, pi/6, pi/4];
% vs=[1.5 2 2.5 3 3.57];
% phidots=[0 0.5 1 1.5 2];

% phis=[pi/100, pi/8, pi/6];
% deltas=[0, pi/10, pi/8];
% vs=[3.57, 3, 2];
% phidots=[0, 0.5, 1];

%search space was 174; choose top 70 and compare with Kate's
%also run on mass-scale
%compare results (scores), choose the best 10



%create matrix of initial conditions to test over (varying size of
%disturbances)

initialSet=[0 0 pi/6 0 0 0 3.57;
            0 0 pi/8 0 0 1 3.57;
            0 0 pi/8 0 pi/5 0 3.57;
            0 0 pi/10 0 0 0 2];

%initialSet=[];
% for i1=1:length(phis)
%     for i2=1:length(deltas)
%         for i3=1:length(phidots)
%             for i4=1:length(vs)
%                 initialSet=[initialSet; 0 0 phis(i1) 0 deltas(i2) phidots(i3) vs(i4)];
%             end
%         end
%     end
% end
% initialSet=[initialSet; 0 0 pi/8 0 pi/4 0 3.57];


%create matrix of controllers to test

%for the purpose of comparing a more refined set of controllers that we had
%come up with (either by pole-placement, or mass-testing), we store the
%gains in an excel file and access them through the script.
%ks=xlsread('compareControllers.xlsx',1,'E1:G348');


%we can test controllers on a mass scale by constructing an array 
%representing sets of gains using for-loops 
% ks=[];                
% for k1=20:200
%     for k2=1:30
%         for k3=-20:-1
%             ks=[ks; k1 k2 k3]; 
%         end
%     end
% end

ks=[72, 23, -20;
    74,26,-20];

%this is the dumb way: loop all the initial conditions over all the
%controllers to find one that works the best
score=[]; %the array in which we store the scores that each controller gets for its performance
for i=1:size(ks,1)
%the set of states is, at time=0, the initial conditions
state=initialSet;
j=1;
commands=zeros(size(state,1),1); %the steering motor command should be 0 at time=0
    %test one controller for all the initial conditions
    while j<=length(tspan)
        phi(:,j)=state(:,3);
        delta(:,j)=state(:,5);
        v(:,j)=state(:,7);
        phidot(:,j)=state(:,6);
        pass=1;
   %ensure lean never exceeds pi/4 and steer never exceeds pi/3
        if max(abs(phi(:,j)))>pi/4
            pass=0;
            disp('lean fail')
            break
        elseif max(abs(delta(:,j)))>pi/3
            pass=0;
            disp('steer fail')
            break
        end

        %add the current state (based on behavior from all initial
        %conditions) to the arrays representing how those state variables
        %change over time
        
        [stateDot,u]=rhs(state,ks(i,:),p);
        %use rhs function to find the rate of change of state variables&u
        
        state=stateDot*tstep+state; %find the new state
        commands(:,j+1)=u; %add u to array of motor commands over time
        
        j=j+1; %update loop variable
    end
    
    %% evaluate controller performance--assign score
    
    %we want to reward good behavior and penalize bad behavior. We expect
    %the bicycle to stabilize--that is, the values of phi, delta, and
    %phidot should all go to zero if the bicycle successfully
    %drives upright in a straight line. So, we assign a score based on an
    %integration of these state variables over the examined timespan
  
    if pass==0 %did this controller pass?
      [failSteer,worstIC(i)]=max(abs(delta(:,j)));
      disp(failSteer)
      [failLean,worstIC_lean(i)]=max(abs(phi(:,j)));
      disp(failLean)
      score(i)=inf; %if no, infinite score
    else %if yes, evaluate score
      phi=abs(phi); delta=abs(delta); phidot=abs(phidot);
      score(i)=sqrt(sum(sum(phi)+sum(delta)+sum(phidot)));
    end
    %right now the score is based only on this....anything else? any other
    %ways we can evaluate the quality of the controller performance?
    
end
%% rank controllers based on scores

%choose the five controllers with the lowest scores by sorting the scores,
%noting the indices, and using the indices to find the the corresponding
%best controllers
    %bestFrac=0.04*size(ks,1);
    [scores,inds]=sort(score);
    bestScoreInds=inds(1:2);
    
    %these are the best five controllers
    bestControllers=ks(bestScoreInds,:);
    kBest=bestControllers(1,:);
    %% run the best controller for all initial conditions
    %have the script run a simulation again for the best controller, under
    %our set of initial conditions, and then plot the results
    
    state=initialSet;
    commands=zeros(size(state,1),1);
    j=1;
    while j<=length(tspan)
        phi(:,j)=state(:,3);
        delta(:,j)=state(:,5);
        v(:,j)=state(:,7);
        phidot(:,j)=state(:,6);
        %add the current state (based on behavior from all initial
        %conditions) to the arrays representing how those state variables
        %change over time
        
        [stateDot,u]=rhs(state,kBest,p);
        %use rhs function to find the rate of change of state variables&u
        
        state=stateDot*tstep+state; %find the new state
        commands(:,j+1)=u; %add u to array of motor commands over time
        
        j=j+1; %update loop variable
    end
    
    
    %% plot the behavior of the bicycle for the best controller under all initial conditions

    close all
    figure(1)
    disp(bestControllers)
    
    for n=1:size(phi,1)
    subplot(2,2,1)
    hold on
    plot(tspan,phi(n,:));
    title('lean vs. time');
    xlabel('time');
    ylabel('phi');
    subplot(2,2,2)
    hold on
    plot(tspan,phidot(n,:));
    title('lean rate vs. time');
    xlabel('time');
    ylabel('phi-dot');
    subplot(2,2,3)
    hold on
    plot(tspan,delta(n,:));
    title('steer vs. time');
    xlabel('time');
    ylabel('delta');
    subplot(2,2,4);
    hold on
    plot(tspan,commands(n,1:length(tspan)));
    title('steer commands vs. time');
    xlabel('time');
    ylabel('delta-dot');
    end
    
    %% below: old script stuff; this script only checked for quick decay and did not rank controllers based on size of disturbances they could recover from
    
    %bikeSimulation(bestController);

% [lean,leanRate,steer,heading]=bikeSimulation(initialSet(in,:),testController,params);
% if max(lean)>=pi/4
%     stability=inf;
% end


% stability=lean^2+leanRate^2+steer^2+heading^2;
%if the sum of squares of the state variables is zero, we know the bike has
%achieved stability
    %we want the least sum of squares, which represents highest stability
% if stability<mostStable
%     mostStable=stability;
%     bestController=testController;
% elseif stability==mostStable
    %if more than one controller achieves the same sum of squares, we want
    %to save them and then actually see which one stabilizes fastest--this
    %is the one we want
%     mostStable=[mostStable stability];
%     bestController=[bestController; testController];
% end
       