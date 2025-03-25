function [necessary_motor_values, necessary_angles] = IK_3DOF(Target)

% Dimensions

qa = [0; 10; 10];
qm2 = [0; 14.5; -9.488];
qm3 = [0; .5; -9.686];
qk = [0; 5; 40];
ab = [0; -13; 0];
ac = [0; -8; 5];
kf = [0; -6; 0];
kd = [0; 4.5; 5];
kh = [0; 0; 30];
hg = [0; 4.3; 0];
gt = [0; 0; 25]; % end effector

% rod end lenhths
l2 = 20;
l3 = 20;
%straight link lenhths
l4 = 30.92;
l5 = 31.72;

ak = qk - qa;
d1 = qk;
d2 = norm(kh);
d3x = 0;
d3y = norm(gt);

distance = norm(Target);
[q3, q4] = R2q3(distance, kf, kh, hg, l5, d1, d2, d3x, d3y);


Y = d1(2) - d2*sin(q3) - d3x*sin(q3+q4-pi/2) - d3y*sin(q3+q4);
Z = d1(3) + d2*cos(q3) + d3x*cos(q3+q4-pi/2) + d3y*cos(q3+q4);

q1 = atan2(Target(1), Target(3));
if q1 > pi/2
    q1 = q1 - pi;
elseif q1 < -pi/2
    q1 = q1 + pi;
end

T_p = [cos(-q1) 0 sin(-q1); 0 1 0; 
       -sin(-q1) 0 cos(-q1)] * Target;
theta1 = atan2(Z, Y);
theta2 = atan2(T_p(3), T_p(2));
q2 = theta2 - theta1;

necessary_angles = [q1 q2 q3, q4];

% q1 to m1

m1 = q1;

R1 = [cos(q1), 0, sin(q1); 0, 1, 0; -sin(q1), 0, cos(q1)];

% q2 to m2

a = qa;
P2 = qm2;
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
R_beta = R2*[1, 0, 0; 0, cos(beta), -sin(beta); 0, sin(beta), cos(beta)];
W4 = P3 - R2*a - R_beta*b;

m3 = -W4(3) + sqrt(l3^2 - W4(1)^2 - W4(2)^2);
if m3 > 5
    m3 = -W4(3) - sqrt(l3^2 - W4(1)^2 - W4(2)^2);
end

necessary_motor_values = round([m1; m2; m3], 3);


function [angle3, angle4] = R2q3(target_distance, kf, kh, hg, l5, d1, d2, d3x, d3y)

q3_n = -pi/3;
q4_n = q3_2_q4(q3_n, kf, kh, hg, l5);

R_n = Range(q3_n, q4_n, d1, d2, d3x, d3y);
error = target_distance - R_n;

iterration = 0;
while abs(error) > 0.01
    iterration = iterration+1;
    f_n = R_n - target_distance;
    diff_f_n = dR_dq3(R_n, q3_n, q4_n, d1, d2, d3x, d3y, kf, kh, hg);
    q3_n = q3_n - f_n/diff_f_n;
    q4_n = q3_2_q4(q3_n, kf, kh, hg, l5);

    R_n = Range(q3_n, q4_n, d1, d2, d3x, d3y);
    error = target_distance - R_n;
end

angle3 = q3_n;
angle4 = q4_n;
end


function q4 = q3_2_q4(q3, kf, kh, hg, l5)

v1 = kf;
v2 = kh;
v3 = hg;

R_q3 = [1 0 0; 0 cos(q3) -sin(q3); 0 sin(q3) cos(q3)];
V = R_q3*v2 - v1;
S5 = (l5^2 - norm(V)^2 - norm(v3)^2) / 2;
p = V(3)*v3(2) - V(2)*v3(3);
q = V(2)*v3(2) + V(3)*v3(3);
r = -S5;

q4 = atan2(p, q) + acos(-r / sqrt(p^2+q^2)) - q3;
if q4>30*pi/180 || q4<-120*pi/180
    q4 = atan2(p, q) - acos(-r / sqrt(p^2+q^2)) - q3;
end

end

function sol = Range(q3, q4, d1, d2, d3x, d3y)


R = norm(d1)^2 + d2^2 + d3x^2 + d3y^2 ...
    +2*d2*d3x*sin(q4) + 2*d2*d3y*cos(q4)...
    -2*d2*d1(2)*sin(q3) + 2*d2*d1(3)*cos(q3)...
    +(2*d3x*d1(3)-2*d3y*d1(2))*sin(q3+q4) + (2*d3x*d1(2)+2*d3y*d1(3))*cos(q3+q4);

sol = sqrt(R);

end

function sol = dR_dq3(R, q3, q4, d1, d2, d3x, d3y, kf, kh, hg)

v1 = norm(kf);
v2 = norm(kh);
v3 = norm(hg);

rhs = -2*v1*v2*cos(q3) - 2*v1*v3*sin(q3 + q4);
lhs = -2*v2*v3*cos(q4) + 2*v1*v3*sin(q3 + q4);
dq4 = rhs/lhs;

sol = (2*d2*d3x*cos(q4)*dq4 - 2*d2*d3y*sin(q4)*dq4 ...
     -2*d2*d1(2)*cos(q3) - 2*d2*d1(3)*sin(q3) ...
     +(2*d3x*d1(3) - 2*d3y*d1(2)) * cos(q3 + q4) * (1+dq4)...
     -(2*d3x*d1(2) + 2*d3y*d1(3))*sin(q3 + q4)*(1+dq4)  ) /(2*R);
end
end
