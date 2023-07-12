function [x1, y1, x2, y2] = FK(q1, q2, l1, l2)

x1 = l1*cos(q1);
y1 = l1*sin(q1);

x2 = x1 + l2*cos(q1+q2);
y2 = y1 + l2*sin(q1+q2);

end

