%--------------------------------------------------------------------------
%PLOT OF TEST SUCCESS AS A FUNCTION OF STEER RADIUS AND VELOCITY FOR
%CURRENT OPTIMIZED GAINS.

tic

%Choose waypoints such that desired path for all cases is a straight line.
x = [0 9 10];
y = [0 0 0];

%Change gain values you wish to plot:
ks = [36, 23, -1];


%Initialize steer angle and velocity ranges.
delta0 = [-pi/2:pi/50:pi/2];
v = [0:0.1:20];

%Choose an initial lean angle close to but not equal to 0 in order to account for
%physical imperfections. You will never actually be able to achieve 0 lean
%angle even if you start it "perfectly straight." Think about trying to
%balance a piece of paper on its edge, it works in MATLAB but not the real
%world.
phi0 = 0.01;

%Records where the bike crashes as a function of velocity and input desired 
%steer angle.
success_points = zeros(length(v),length(delta0));    

%Records whether the bike's actual steer angle ever reaches the input
%desired steer angle as a function of velocity and input desired steer
%angle.
steerCheck_points = zeros(length(v),length(delta0));


%Record check points for all test matrices for 0<=v<=25 and 0<=steerD<=2pi.
nv = length(v);
nd = length(delta0);
success_v = [];
ind = 1;
for i=1:length(v)
    for j=1:length(delta0)
        [success, p] = mainNavigation(x,y,v(i),delta0(j),phi0,ks,0);
        success_points(i,j) = success;
        if success == 1 && delta0(j) == 0
            success_v(ind) = v(i);
            ind = ind + 1;
        end
    end
end

slowest_v = min(success_v);
l = plot(delta0, slowest_v*ones(length(delta0)),'w','LineWidth',5);
fprintf('Slowest possible velocity at zero turning angle using Arundathi gains:')
fprintf(' %d m/s\n', slowest_v)


%Scatter plot for bike crash as a function of velocity and desired steer
%angle for Arundathi's Optimized Gains.
figure(1)
hold on
axis fill

surf(success_points)
view(2)
xlabel('Steer Angle (radians)')
ylabel('Velocity (m/s)')


n1 = length(delta0);
xticks([0 n1/4 n1/2 3*n1/4 n1])
xticklabels({'-pi/2','-pi/4','0','pi/4','pi/2'})

n2 = length(v);
yticks([0 n2/4 n2/2 3*n2/4 n2])
l1 = sprintf('%d',v(1));
l2 = sprintf('%d',v(end)/4);
l3 = sprintf('%d',v(end)/2);
l4 = sprintf('%d',3*v(end)/4);
l5 = sprintf('%d',v(end));
yticklabels({l1,l2,l3,l4,l5})


s1 = sprintf('Success of the Balance Control as a Function of Velocity and Steer Angle\n');
s2 = sprintf('Using Gains: k1 = %.2f, k2 = %.2f, k3 = %.2f\n', ks(1), ks(2), ks(3));
s3 = sprintf('Number of successes normalized by number of tests: %f\n', sum(sum(success_points))/(nv*nd));
s4 = sprintf('Slowest possible speed = %.2f m/s\n',slowest_v);
title([s1 s2 s3 s4])


toc