function dx = pendulumDynamics(t,x)
m = 1;
l = 0.5;
g = 9.81;

dx = [x(2,1);
    -(g/l)*sin(x(1,1))];
end
