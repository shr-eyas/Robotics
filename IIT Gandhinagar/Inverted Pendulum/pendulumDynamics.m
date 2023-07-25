function dx = pendulum_dynamics(t, x, tau)
    m = 1;
    L = 0.5;
    g = 9.81;

    dx = [x(2); -(g/L)*x(1)] + [0; (1/m*l^2)*tau];
end
