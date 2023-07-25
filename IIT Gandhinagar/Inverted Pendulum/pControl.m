% Pendulum parameters
m = 1;                      % Mass = 1kg
L = 1;                      % Length = 1m

% Time parameters
run_time = 10;
t_span = [0, run_time];     

% Initial state [position; angular velocity]
x0 = [0; 0];     

% PID Gains
Kp = 300;

% Trajectory planning to generate desired position as a function of time
[theta_function, ~] = traj(0, run_time, 0, pi, 0, 0);
syms t

% Using ODE45 to solve
ode = @(T, x) pendulumDynamics(T, x, m, L, Kp * (double(subs(theta_function, t, T)) - x(1)));
[T, x] = ode45(ode, t_span, x0);

theta_actual = x(:, 1); 

figure;
plot(T,double(subs(theta_function, t, T)))
hold on
plot(T, theta_actual);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Pendulum Angle vs. Time');
legend('Desired','Actual')
