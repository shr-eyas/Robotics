L = 1;
g = 9.81;

stopTime = 10;
ts = 0.01;
time = 0:ts:stopTime;

theta = zeros(10, numel(time));
thetaD = zeros(10, numel(time));

theta(:, 1) = 0;
thetaD(:, 1) = 0;

Kp = 10;
Kd = 0;
gammaILC = 0.01; 
lambdaILC = 1;

[x, ~] = traj(0, stopTime, 0, pi, 0, 0);

desiredTheta = zeros(10, numel(time));

for k = 1:10
    [x_k, ~] = traj(0, stopTime, 0, pi, 0, 0);  
    desiredTheta(k, :) = subs(x_k, t, time);
end

tauILC = zeros(10, numel(time));

figure; 
hold on;

for k = 2:10
    error = desiredTheta(k, :) - theta(k, :);


    for i = 2:numel(time)

        tauILC(k, i) = lambdaILC*tauILC(k-1, i-1) + gammaILC*(error(i-1)/ts);

        
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
