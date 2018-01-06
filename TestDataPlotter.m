%Plot the data from a real and compare it to the results of a simulation.

data = csvread('bike_2017-11-01~~06-35-46-PM.csv', 1, 1);
ks = [175.6 33.6 -1.11]; %Gain values corresponding to this data set

%Plot velocity throughout the real test. The reason for doing this is to
%ensure that our test is recording data at a constant velocity for accurate
%compairson with the simulation.
dataV = data(4353:5100,8);
ind = [0:length(dataV)-1];

%Filter outliers
for i=1:length(dataV)
    if(abs(dataV(i)) >= 6)
        dataV(i) = 0;
    end
end

figure(1)
clf
plot(ind, dataV)
xlabel('Timestep (units???)')
ylabel('Velocity (m/s)')
title('Velocity vs. Time (Test 3: 11/02/17)')

%Choose average velocity from the section of the graph that appears constant.
testV = mean(dataV); %m/s

%Collect data from sim using same velocity and gains as test.
[success, state] = mainNavigation([0 15],[0 0],testV,0,0,ks,0);

%Extract and plot yaw angle from test and sim.
dataPsi = data(4353:5100,11); %rad
simPsi = state(:,4);
tstepSim = [0:length(simPsi)-1];

%map ind to be the same size as tstep.
ind = ind*length(tstepSim)/length(ind);

%map simPsi to be the same size as dataPsi.
simPsi = simPsi*length(dataPsi)/length(simPsi);

figure(2)
clf
hold on
plot(ind, dataPsi, 'r');
plot(tstepSim, simPsi, 'b--', 'LineWidth', 1);
legend('Test', 'Sim')
ylabel('Lean Angle (rad)')
xlabel('Time')
title('Yaw: Test vs Sim')

size(dataPsi)
size(simPsi)

psiScore = (sum(abs(dataPsi)) - sum(abs(simPsi)))/(sum(abs(simPsi)));
fprintf('Path Score: %f\n', psiScore)



%Use psi to convert to x and y and plot compared to x and y of the sim.
xTest = dataV*cos(dataPsi);
yTest = dataV*sin(dataPsi);
