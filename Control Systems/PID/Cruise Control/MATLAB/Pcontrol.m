m = 1000;
b = 50;
r = 10;

s = tf('s');
P_cruise = 1/(m*s + b);

Kp = 5000;
C = pid(Kp);

T = feedback(C*P_cruise,1);

step(r*T,t)
axis([0 20 0 10])
