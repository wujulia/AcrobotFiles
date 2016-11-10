function [dx, y] = AcrobotIDModel(t,x,u,m1,m2,l1,lc1,lc2,b1,b2,I1,I2)
% Parameterized model for system identification
% Outputs 
%   dx = [qd;qdd]
%   y = q (the measurements)
% Inputs
%   x = [q;qd]
%   u = torque
%   m1,m2 = Link masses
%   l1 = link 1 length (link 2 length does not effect dynamics)
%   lc1,lc2 = link COM position
%   b1,b2 = damping terms
%   I1,I2 = link inertias

g = 9.81;
y = [x(1);x(2)];

dx(1) = x(3);
dx(2) = x(4);

q = [x(1);x(2)];
qd = [x(3);x(4)];

m2l1lc2 = m2*l1*lc2;  % occurs often!

c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
h12 = I2 + m2l1lc2*c(2);
H = [ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12; h12, I2 ];
      
C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
            
% accumate total C and add a damping term:
C = C*qd + G + [b1;b2].*qd;

B = [0; 1];

tau = B*u;
qdd = H\(tau - C);


dx(3) = qdd(1);
dx(4) = qdd(2);