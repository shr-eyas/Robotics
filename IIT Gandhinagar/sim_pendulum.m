L = 1;
g = 9.81;

stopTime = 10;
ts = 0.01;
time = 0:ts:stopTime;

theta = zeros(size(time));
thetaD = zeros(size(time));
theta(1) = 0;
thetaD(1) = 0;

tau = zeros(size(time));

Kp = 10;
Kd = 0;
error = zeros(size(time));

[x, ~] = traj(0,stopTime,0,pi,0,0);

syms t 
desiredTheta = subs(x,t,time);

for i = 2:numel(time)
    
    error(i) = desiredTheta(i) - theta(i);

    tau(i) = Kp * error(i) + Kd * ((error(i) - error(i-1))/ts);
 
    thetaDD = tau(i) - (g/L) * (theta(i-1));

    thetaD(i) = thetaD(i-1) + thetaDD * ts;
    theta(i) = theta(i-1) + thetaD(i) * ts;

end


plot(time, theta,'r');
hold on
plot(time,desiredTheta,'k')
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');

axis equal
