function position = angles2Position(q1, q2, q3, q4,d1,d2,d3)
q1 = deg2rad(q1);
q2 = deg2rad(q2);
q3 = deg2rad(q3);
q4 = deg2rad(q4);

d1 = [0; 5; d1]; d2 = [0; 0; d2]; d3 = [0; 0; d3];

T1 = [cos(q1), 0, sin(q1), 0;
      0, 1, 0, 0;
      -sin(q1), 0, cos(q1), 0;
      0, 0, 0, 1];

T2 = [1, 0, 0, 0;
      0, cos(q2), -sin(q2), 0;
      0, sin(q2), cos(q2), 0;
      0, 0, 0, 1];

T3 = [1, 0, 0, 0;
      0, cos(q3), -sin(q3), d1(2);
      0, sin(q3), cos(q3), d1(3);
      0, 0, 0, 1];

T4 = [1, 0, 0, 0;
      0, cos(q4), -sin(q4), d2(2);
      0, sin(q4), cos(q4), d2(3);
      0, 0, 0, 1];

T5 = [1, 0, 0, 0;
      0, 1, 0, d3(2);
      0, 0, 1, d3(3);
      0, 0, 0, 1];

T = T1*T2*T3*T4*T5;

position = T(1:3, 4);

end