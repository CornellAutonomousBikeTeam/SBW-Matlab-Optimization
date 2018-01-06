%This script is used to run tests on the bike to determine balance control.
%Our goal is to optimize the gains such that the bike can safely travel at 
%any speed and can recover from disturbances.

tic
% %--------------------------------------------------------------------------
% %PLOT OF TEST SUCCESS AS A FUNCTION OF STEER RADIUS AND VELOCITY FOR
% %CURRENT OPTIMIZED GAINS.
% 
%Choose waypoints such that desired path for all cases is a straight line.
x = [0 9 10];
y = [0 0 0];
% 
% %Start timer so we know our run-time.
% 
% 
% %Initialize steer angle and velocity ranges.
% delta0 = [-pi:pi/50:pi];
% v = [0:0.1:20];
% 
% %Choose an initial lean angle close to but not equal to 0 in order to account for
% %physical imperfections. You will never actually be able to achieve 0 lean
% %angle even if you start it "perfectly straight." Think about trying to
% %balance a piece of paper on its edge, it works in MATLAB but not the real
% %world.
% phi0 = 0.01;
% 
% %Records where the bike crashes as a function of velocity and input desired 
% %steer angle.
% success_points = zeros(length(v),length(delta0));    
% 
% %Records whether the bike's actual steer angle ever reaches the input
% %desired steer angle as a function of velocity and input desired steer
% %angle.
% steerCheck_points = zeros(length(v),length(delta0));
% 
% 
% %Record check points for all test matrices for 0<=v<=25 and 0<=steerD<=2pi.
% nv = length(v);
% nd = length(delta0);
% success_v = [];
% ind = 1;
% for i=1:length(v)
%     for j=1:length(delta0)
%         [success, p] = mainNavigationArundathi(x,y,v(i),delta0(j),phi0,0,0);
%         success_points(i,j) = success;
%         if success == 1 && delta0(j) == 0
%             success_v(ind) = v(i);
%             ind = ind + 1;
%         end
%     end
% end
% 
% slowest_v = min(success_v);
% l = plot(delta0, slowest_v*ones(length(delta0)),'w','LineWidth',5);
% fprintf('Slowest possible velocity at zero turning angle using Arundathi gains:')
% fprintf(' %d m/s\n', slowest_v)
% 
% 
% %Scatter plot for bike crash as a function of velocity and desired steer
% %angle for Arundathi's Optimized Gains.
% figure(1)
% hold on
% axis fill
% 
% surf(success_points)
% view(2)
% xlabel('Steer Angle (radians)')
% ylabel('Velocity (m/s)')
% 
% 
% n1 = length(delta0);
% xticks([0 n1/4 n1/2 3*n1/4 n1])
% xticklabels({'-pi','-pi/2','0','pi/2','pi'})
% 
% n2 = length(v);
% yticks([0 n2/4 n2/2 3*n2/4 n2])
% l1 = sprintf('%d',v(1));
% l2 = sprintf('%d',v(end)/4);
% l3 = sprintf('%d',v(end)/2);
% l4 = sprintf('%d',3*v(end)/4);
% l5 = sprintf('%d',v(end));
% yticklabels({l1,l2,l3,l4,l5})
% 
% s = sprintf('Slowest velocity = %d m/s', slowest_v);
% legend(l, s)
% 
% s1 = sprintf('Success of the Balance Control as a Function of Velocity and Steer Angle\n');
% s2 = sprintf('Using Gains: k1 = %.2f, k2 = %.2f, k3 = %.2f\n',p.k1, p.k2, p.k3);
% s3 = sprintf('Number of successes normalized by number of tests: %f\n', sum(sum(success_points))/(nv*nd));
% s4 = sprintf('Slowest possible speed = %.2f m/s\n',slowest_v);
% title([s1 s2 s3 s4])


%--------------------------------------------------------------------------
%USE GAIN SCHEDULING TO DETERMINE OPTIMAL GAINS TO DRIVE AT SLOWEST
%POSSIBLE VELOCITY.


delta0 = 0;
phi0 = 0.01;
v = 0.5;

%k3 should be the opposite sign of k1 and k2.
k1 = [240:250];
k2 = [120:130];
k3 = [-5:-1];  

result = zeros(length(k1)*length(k2)*length(k3),5);
trial = 1;

n1 = length(k1);
n2 = length(k2);
n3 = length(k3);

for a=1:n1
    for b=1:n2
        for c=1:n3

            ks = [k1(a) k2(b) k3(c)];

            [success, state] = mainNavigation(x,y,v,delta0,phi0,ks,0);  
            phi = abs(state(:,3));
            delta = abs(state(:,5));
            phidot = abs(state(:,6));
            
            result(trial,1) = success;
            result(trial,2) = sqrt(sum(phi) + sum(delta) + sum(phidot));
            result(trial,3) = k1(a);
            result(trial,4) = k2(b);
            result(trial,5) = k3(c);
            disp(trial)
            trial = trial + 1;

        end
    end
end

success = result(:,1);
score = result(:,2);
k_1 = result(:,3);
k_2 = result(:,4);
k_3 = result(:,5);

T = table(success,score,k_1,k_2,k_3)
m = table2array(T);
m = sortrows(m,2);
ind = find(m(:,1));
best = m(ind(1),:);

fprintf('Best gain values for v = %fm/s :\n',v)
fprintf('k1 = %d\nk2 = %d\nk3 = %d\n',best(3),best(4),best(5))
fprintf('score = %f\n', best(2))
fprintf('success = %0.f\n', best(1))


toc














