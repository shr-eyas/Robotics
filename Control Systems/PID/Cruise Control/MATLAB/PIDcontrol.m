m = 1000;
b = 50;
r = 10;

s = tf('s');
P_cruise = 1/(m*s + b);

Kp = 800;
Ki = 40;
Kd = 1;
C = pid(Kp,Ki,Kd);

T = feedback(C*P_cruise,1);

step(r*T,t)
axis([0 20 0 10])
