function [x,v] = traj(ti,tf,xi,xf,vi,vf)

syms t

q = [xi;vi;xf;vf];

t0 = ti;

b = [1  t0 t0*t0  t0*t0*t0;
     0  1  2*t0   3*t0*t0;
     1  tf tf*tf  tf*tf*tf;
     0  1  2*tf   3*tf*tf];

a = b\q;

a0 = a(1,1);
a1 = a(2,1);
a2 = a(3,1);
a3 = a(4,1);

x = a0 + (a1*t) + (a2*t*t) + (a3*t*t*t);
v = a1 + (2*a2*t) + (3*a3*t*t);

end

