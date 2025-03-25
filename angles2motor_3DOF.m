function [m1, m2, m3] = angles2motor_3DOF(q1,q2,q3)


qa = [0; 10; 10];
qm2 = [0; 14.5; -14.836];
qm3 = [0; -.5; -15.111];
qk = [0; 5; 40];
ab = [0; -13; 0];
ac = [0; -8; 5];
kf = [0; -6; 0];
kd = [0; 4.5; 5];
kh = [0; 0; 30];
hg = [0; 4.3; 0];
ak = qk-qa;
ht = [0; 0; 25]; % end effector

% rod end lengths
l2 = 25.24;
l3 = 25.24;
%straight link lengths
l4 = 30.92;
l5 = 31.72;

m1 = q1;
R1 = [cos(q1), 0, sin(q1); 0, 1, 0; -sin(q1), 0, cos(q1)];


% q2 to m2

a = qa;
P2 = qm2;
l2 = norm(qa - qm2);
R2 = R1*[1, 0, 0; 0, cos(q2), -sin(q2); 0, sin(q2), cos(q2)];
W2 = P2 - R2*a;

m2 = -W2(3) + sqrt(l2^2 - W2(1)^2 - W2(2)^2);
if m2 > 5
    m2 = -W2(3) - sqrt(l2^2 - W2(1)^2 - W2(2)^2);
end


% q3 to beta

u1 = ac;
u2 = ak;
u3 = kd;
l4 = norm(u2+u3-u1);

R3 = [1, 0, 0; 0, cos(q3), -sin(q3); 0, sin(q3), cos(q3)];
W3 = u2 + R3*u3;
S = (norm(W3)^2 + norm(u1)^2 - l4^2 )/2;

A = u1(2)*W3(3) - u1(3)*W3(2);
B = u1(2)*W3(2) + u1(3)*W3(3);

beta = atan2(A, B) + acos(S/norm([A B]));
if beta > deg2rad(90) || beta < -deg2rad(45)
    beta = atan2(A, B) - acos(S/norm([A B]));
end


% beta to m3

b = ab;
P3 = qm3;
l3 = norm(qa+ab-qm3);

R_beta = R2*[1, 0, 0; 0, cos(beta), -sin(beta); 0, sin(beta), cos(beta)];
W4 = P3 - R2*a - R_beta*b;

m3 = -W4(3) + sqrt(l3^2 - W4(1)^2 - W4(2)^2);
if m3 > 5
    m3 = -W4(3) - sqrt(l3^2 - W4(1)^2 - W4(2)^2);
end

% m1 = round(m1,3);
% m2 = round(m2,3);
% m3 = round(m3,3);
end

