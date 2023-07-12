%% constants

xE = 10; yE = 0;  % end effector position
l1 = 10; l2 = 10; % link length
x0 = 0; y0 = 0;   % origin

%% testing IK 

[q1, q2] = IK(xE,yE,10,10);

t1 = q1*(180/pi);
t2 = q2*(180/pi);
disp(t1)
disp(t2)

%% testing FK

[x1,y1,x2,y2] = FK(q1,q2,10,10);

plot([x0 x1],[y0 y1],[x1 x2],[y1 y2],'linewidth',2)
axis([-25 25 -25 25])
grid on 
