m = 1;              % Mass = 1kg
L = 1;              % Length = 1m
t_span = [0, 10];   % Time (t=0 to t=10 seconds)

% Initial state [position; angular velocity]
initial_state = [0; 0]; 

% Torque input
tau = 0;  

% x is the state vector [position; angular velocity]
[t, x] = ode45(@(t, x) pendulumDynamics(t, x, m, L, tau), t_span, initial_state);


figure;
subplot(2, 1, 1);
plot(t, x(:, 1));
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Position');

subplot(2, 1, 2);
plot(t, x(:, 2));
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity');
