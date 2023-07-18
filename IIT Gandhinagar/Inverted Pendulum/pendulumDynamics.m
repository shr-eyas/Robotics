function dx = pendulumDynamics(t,x)
m = 1;
l = 0.5;
g = 9.81;

dx = [x(2,1);
    -(g/l)*(x(1,1))];  % Using sin(x) will make it non linear
end
