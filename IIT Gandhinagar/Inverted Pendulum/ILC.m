clc
L = 1;
g = 9.81;

stopTime = 10;
ts = 0.1;
time = 0:ts:stopTime;

numIterations = 10;

theta = zeros(numIterations, numel(time));
thetaD = zeros(numIterations, numel(time));

theta(:, 1) = 0;
thetaD(:, 1) = 0;

gammaILC = 1; 
lambdaILC = 1;
gammaFF = 1;
lambdaFF = 1;
lamdaModel = 1;
gammaModel = 1;

A = [0.5 0   0.05   0;
     0   0.5 0      0.05;
     0   0   0.4795 0.0205;
     0   0   0.0205 0.3641];

B = [0       0;
     0       0;
     4.1096 -4.1096;
    -4.1096  27.1865];

C = [1 0 0 0;
     0 1 0 0;];

Mp = 1;

[x, ~] = traj(0, stopTime, 0, pi, 0, 0);

desiredTheta = pi*ones(numIterations, numel(time));

syms t 

for k = 1:numIterations
    [x_k, ~] = traj(0, stopTime, 0, pi, 0, 0);  
    desiredTheta(k, :) = subs(x_k, t, time);
end

tauILC = zeros(numIterations, numel(time));
tauFF = zeros(numIterations, numel(time));
moeError = zeros(numIterations, numel(time));
tau = zeros(numIterations, numel(time));
thetaModel = zeros(numIterations, numel(time));
taskError = zeros(numIterations, numel(time));
speError = zeros(numIterations, numel(time));

figure; 
hold on;

for k = 2:numIterations          
    for t = 2:numel(time)
                
        tau(k, t) = tauILC(k, t) + tauFF(k, t);
        tauILC(k, t) = lambdaILC*tauILC(k-1, t) + gammaILC*tanh(taskError(t));
        tauFF(k, t) = lambdaFF*tauFF(k-1,t) + gammaFF*Mp*tanh(moeError(k-1, t));
        thetaModel(k, t) = lamdaModel*thetaModel(k-1, t) + gammaModel*tanh(speError(k-1, t));
        taskError(k-1, t) = desiredTheta(k-1, t) - theta(k-1, t);
        speError(k-1, t) = theta(k-1, t) - thetaModel(k-1, t);
        moeError = Mp*tauILC(k-1, t) - thetaModel;
        
        % Model dynamics
        thetaDD = tauILC(k, t) - (g / L) * (theta(k, t-1));
        
        % Euler integration
        thetaD(k, t) = thetaD(k, t-1) + thetaDD * ts;
        theta(k, t) = theta(k, t-1) + thetaD(k, t) * ts;
    end
    plot(time, theta(k, :));
end

plot(time,desiredTheta,'k');
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
title('Pendulum Angle for All Iterations');
axis equal;
