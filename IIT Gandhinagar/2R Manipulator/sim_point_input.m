clc
clear all

xi = 5;
yi = 5;
xf = 20;
yf = 0;

[x,vx] = traj(0,2,xi,xf,0,0);
[y,vy] = traj(0,2,yi,yf,0,0);

syms t

[q1, q2] = IK(x,y,10,10);


t_val = linspace(0,2,100);

q1a = subs(q1,t,t_val);
q2a = subs(q2,t,t_val);

[x1,y1,x2,y2] = FK(q1a,q2a,10,10);

x0 = 0;
y0 = 0;

for i = 1:length(q1a)
    t1 = double(q1a(i));
    t2 = double(q2a(i));
    [x1,y1,x2,y2] = FK(t1,t2,10,10);        
    plot([x0 x1],[y0 y1],[x1 x2],[y1 y2],'linewidth',2)
    axis([-25 25 -25 25])
    pause(0.1)
end
