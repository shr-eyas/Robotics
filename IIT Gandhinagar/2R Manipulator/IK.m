function [q1, q2] = IK(X,Y,l1,l2)

% a = (l1^2+l2^2-X^2-Y^2)/(2*l1*l2);
% q2 = pi - atan2(sqrt(1-a^2),a);
% q1 = atan2(Y,X) - atan2((l2*sin(q2)),(l1+(l2*cos(q2))));

d = sqrt((X^2)+(Y^2));
cbeta = ((d*d)+(l1*l1)-(l2*l2))/(2*l1*d);
sbeta = sqrt(1-(cbeta^2));
beta = atan2(sbeta,cbeta);
q1 = atan2(Y,X) - beta;
a = (l1^2+l2^2-X^2-Y^2)/(2*l1*l2);
q2 = pi - atan2(sqrt(1-a^2),a);

end
