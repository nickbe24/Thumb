function q4 = q3_to_q4(q3)

q3 = deg2rad(q3);
v1 = [0;-6;0];
v2 = [0;0;30];
v3 = [0;4.3;0];
l5 = 31.72;

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
q4 = rad2deg(q4);
end