%Plot the velocity over time of the bike in lab to prove that the velocity
%is properly discretized.

data = csvread('bike_2017-10-26~~06-38-35-PM.csv', 1, 1);
v = data(:,8);
t = [0:length(v)-1];

%Filter outliers
for i=1:length(v)
    if(abs(v(i)) >= 6)
        v(i) = 0;
    end
end

clf
plot(t, v)
xlabel('Timestep (units???)')
ylabel('Velocity (m/s)')
title('Velocity vs. Time (in lab test 10/26/17)')