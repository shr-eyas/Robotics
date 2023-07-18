% ILC from Arimoto's 1984 paper

clc
L = 1;
g = 9.81;

stopTime = 10;
ts = 0.1;
time = 0:ts:stopTime;

maxIterations = 10;

theta = zeros(maxIterations, numel(time));
thetaD = zeros(maxIterations, numel(time));

theta(:, 1) = 0;
thetaD(:, 1) = 0;

gammaILC = 1; 
lambdaILC = 1;

[x, ~] = traj(0, stopTime, 0, pi, 0, 0);

desiredTheta = pi*ones(maxIterations, numel(time));

syms t 

for k = 1:maxIterations
    [x_k, ~] = traj(0, stopTime, 0, pi, 0, 0);  
    desiredTheta(k, :) = subs(x_k, t, time);
end

tauILC = zeros(maxIterations, numel(time));

figure; 
hold on;

for k = 2:maxIterations
    error = desiredTheta(k, :) - theta(k, :);
    for i = 2:numel(time)
        
        tauILC(k, i) = lambdaILC*tauILC(k-1, i-1) + gammaILC*(error(i-1));
        thetaDD = tauILC(k, i) - (g / L) * (theta(k, i-1));
        thetaD(k, i) = thetaD(k, i-1) + thetaDD * ts;
        theta(k, i) = theta(k, i-1) + thetaD(k, i) * ts;
    end
    plot(time, theta(k, :));
end

plot(time,desiredTheta,'k');
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
title('Pendulum Angle for All Iterations');
legend('Iteration 1','Iteration 2', 'Iteration 3', 'Iteration 4', 'Iteration 5', 'Iteration 6', 'Iteration 7', 'Iteration 8', 'Iteration 9','Desired');
axis equal;
