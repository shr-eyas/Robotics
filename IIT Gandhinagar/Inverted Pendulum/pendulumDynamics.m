function dx = pendulum_dynamics(~, x, m, L, tau)
    g = 9.81;
    dx = [x(2); -(g/L)*sin(x(1))] + [0; (1/m*L^2)*tau];
end

