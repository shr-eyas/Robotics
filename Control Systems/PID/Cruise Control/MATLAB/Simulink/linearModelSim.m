m = 1000;
b = 50;
u = 500;
[A,B,C,D] = linmod('linearModel');
cruise_ss = ss(A,B,C,D);
step(u*cruise_ss)
