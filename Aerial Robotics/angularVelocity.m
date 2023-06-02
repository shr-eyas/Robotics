% angular velocity for transpose about z axis
syms theta

R = [1   0            0;
     0   cos(theta)  -sin(theta);
     0   sin(theta)   cos(theta)];

RT = transpose(R);
dR = diff(R, theta);
omega = RT * dR;
disp(omega)


