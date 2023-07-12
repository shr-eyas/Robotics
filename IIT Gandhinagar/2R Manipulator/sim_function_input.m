clc

xi =  0.001;    yi = 05;
xf = 20;        yf = 0;

l1 = 10;
l2 = 10;

x0 = 0;
y0 = 0;

syms t

[x,vx] = traj(0,2,xi,xf,0,0);
y = 5*sin(x);

[q1, q2] = IK(x,y,l1,l2);

t_val = linspace(0,2,50);

q1a = subs(q1,t,t_val);
q2a = subs(q2,t,t_val);


for i = 1:length(q1a)
    t1 = double(q1a(i));
    t2 = double(q2a(i));
    [x1,y1,x2,y2] = FK(t1,t2,10,10); 

    cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi)),y0 + r*sin(linspace(0,2*pi)),'-');
    cplot(l1+l2,x0,y0)
    hold on

    plot(linspace(0,19.64),5*sin(linspace(0,19.64)),'-')
    hold on

    plot([x0 x1],[y0 y1],[x1 x2],[y1 y2],'linewidth',2)
    hold off
    axis equal

    pause(0.1)
end
