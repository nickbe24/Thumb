clear


dt = .01;

% orientation of hand
%index
index_x_ang = 0;
R_index = [1 0 0; 0 cosd(index_x_ang) -sind(index_x_ang); 0 sind(index_x_ang) cosd(index_x_ang)];
P_index = [0;0;0];
%middle
R_middle = eye(3);
P_middle = [0;0;32];
%ring
ring_x_ang = 5;
R_ring = [1 0 0; 0 cosd(ring_x_ang) -sind(ring_x_ang); 0 sind(ring_x_ang) cosd(ring_x_ang)];
P_ring = [0;-5;59];
%pinkie
pinkie_x_ang = 10;
R_pinkie = [1 0 0; 0 cosd(pinkie_x_ang) -sind(pinkie_x_ang); 0 sind(pinkie_x_ang) cosd(pinkie_x_ang)];
P_pinkie = [0;-10;86];
%thumb
thumb_x_ang = -70;
thumb_y_ang = -50;
rt1 = [1 0 0 0; 0 cosd(thumb_x_ang) -sind(thumb_x_ang) 0; 0 sind(thumb_x_ang) cosd(thumb_x_ang) 0; 0 0 0 1];
rt2 = [ cosd(thumb_y_ang) 0 sind(thumb_y_ang) 0; 0 1 0 0; -sind(thumb_y_ang) 0 cosd(thumb_y_ang) 0; 0 0 0 1];
thumb_adjuster = [1 0 0 12.25; 0 1 0 0; 0 0 1 0; 0 0 0 1];
P_thumb = [1 0 0 5;0 1 0 -120;0 0 1 -20; 0 0 0 1];
T_thumb = P_thumb*thumb_adjuster*rt1*rt2*[1 0 0 -35.417; 0 1 0 0; 0 0 1 0; 0 0 0 1]; %premult to rotate about w, post to rotate about b

%% figure making
t3 = 1;

[f1m1, f1m2, f1m3, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %index
[f2m1, f2m2, f2m3, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %middle
[f3m1, f3m2, f3m3, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %ring
[f4m1, f4m2, f4m3, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %pinkie
[thm1, thm2, thm3, thm4] = joint_control_4DOF([0 0 0 0], [0 0 0 0], t3, dt); %thumb

%% side pinch p1

t3 = .1;

[f1m1_0, f1m2_0, f1m3_0, ~] = joint_control_3DOF([0 0 0],[deg2rad(6) 0 0],t3,dt); %index
[f2m1_0, f2m2_0, f2m3_0, ~] = joint_control_3DOF([0 0 0],[deg2rad(-5) 0 0],t3,dt); %middle
[f3m1_0, f3m2_0, f3m3_0, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %ring
[f4m1_0, f4m2_0, f4m3_0, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %pinkie
[thm1_0, thm2_0, thm3_0, thm4_0] = joint_control_4DOF([0 0 0 0], [0 0 0 0], t3, dt); %thumb


%% side pinch p2

t4 = 3.5;

[f1m1_1, f1m2_1, f1m3_1, ~] = joint_control_3DOF([deg2rad(6) 0 0],[0 0 0],t4,dt); %index
[f2m1_1, f2m2_1, f2m3_1, ~] = joint_control_3DOF([deg2rad(-5) 0 0],[0 0 0], t4, dt); %middle
[f3m1_1, f3m2_1, f3m3_1, ~] = joint_control_3DOF([0 0 0],[0 0 0],t4,dt); %ring
[f4m1_1, f4m2_1, f4m3_1, ~] = joint_control_3DOF([0 0 0],[0 0 0],t4,dt); %pinkie
[thm1_1, thm2_1, thm3_1, thm4_1] = joint_control_4DOF( [0 0 0 0], [0 0 0 0], t4, dt); %thumb

%% top pinch p1

t3 = 3.5;

[f1m1_2, f1m2_2, f1m3_2, ~] = joint_control_3DOF([0 0 0],[deg2rad(20) deg2rad(-70) deg2rad(0)],t3,dt); %index
[f2m1_2, f2m2_2, f2m3_2, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %middle
[f3m1_2, f3m2_2, f3m3_2, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %ring
[f4m1_2, f4m2_2, f4m3_2, ~] = joint_control_3DOF([0 0 0],[0 0 0],t3,dt); %pinkie
[thm1_2, thm2_2, thm3_2, thm4_2] = joint_control_4DOF([0 0 0 0], [0 0 0 0], t3, dt); %thumb


%% top pinch p2

t4 = 3.5;

[f1m1_3, f1m2_3, f1m3_3, ~] = joint_control_3DOF([deg2rad(20) deg2rad(-70) deg2rad(0)],[deg2rad(20) deg2rad(-70) deg2rad(0)],t4,dt); %index
[f2m1_3, f2m2_3, f2m3_3, ~] = joint_control_3DOF([0 0 0],[deg2rad(-25) deg2rad(-35) deg2rad(-30)], t4, dt); %middle
[f3m1_3, f3m2_3, f3m3_3, ~] = joint_control_3DOF([0 0 0],[0 0 0],t4,dt); %ring
[f4m1_3, f4m2_3, f4m3_3, ~] = joint_control_3DOF([0 0 0],[0 0 0],t4,dt); %pinkie
[thm1_3, thm2_3, thm3_3, thm4_3] = joint_control_4DOF( [0 0 0 0], [0 0 0 0], t4, dt); %thumb

%% top pinch p3

t5 = 3.5;

[f1m1_4, f1m2_4, f1m3_4, ~] = joint_control_3DOF([deg2rad(20) deg2rad(-70) deg2rad(0)],[0 0 0],t5,dt); %index
[f2m1_4, f2m2_4, f2m3_4, ~] = joint_control_3DOF([deg2rad(-25) deg2rad(-35) deg2rad(-30)],[0 0 0],t5,dt); %middle
[f3m1_4, f3m2_4, f3m3_4, ~] = joint_control_3DOF([0 0 0],[0 0 0],t5,dt); %ring
[f4m1_4, f4m2_4, f4m3_4, ~] = joint_control_3DOF([0 0 0],[0 0 0],t5,dt); %pinkie
[thm1_4, thm2_4, thm3_4, thm4_4] = joint_control_4DOF([0 0 0 0], [0 0 0 0], t5, dt); %thumb
%% thumb to index tip p1

t5 = 3.5;

[f1m1_5, f1m2_5, f1m3_5, ~] = joint_control_3DOF([0 0 0],[deg2rad(2) deg2rad(-75) deg2rad(-28)],t5,dt); %index
[f2m1_5, f2m2_5, f2m3_5, ~] = joint_control_3DOF([0 0 0],[0 0 0],t5,dt); %middle
[f3m1_5, f3m2_5, f3m3_5, ~] = joint_control_3DOF([0 0 0],[0 0 0],t5,dt); %ring
[f4m1_5, f4m2_5, f4m3_5, ~] = joint_control_3DOF([0 0 0],[0 0 0],t5,dt); %pinkie
[thm1_5, thm2_5, thm3_5, thm4_5] = joint_control_4DOF([0 0 0 0], [deg2rad(5) deg2rad(-55) deg2rad(-30) deg2rad(-20)], t5, dt); %thumb

%% thumb to index tip p2
t6 = 3.5;

[f1m1_6, f1m2_6, f1m3_6, ~] = joint_control_3DOF([deg2rad(2) deg2rad(-75) deg2rad(-28)],[0 0 0],t6,dt); %index
[f2m1_6, f2m2_6, f2m3_6, ~] = joint_control_3DOF([0 0 0],[0 0 0],t6,dt); %middle
[f3m1_6, f3m2_6, f3m3_6, ~] = joint_control_3DOF([0 0 0],[0 0 0],t6,dt); %ring
[f4m1_6, f4m2_6, f4m3_6, ~] = joint_control_3DOF([0 0 0],[0 0 0],t6,dt); %pinkie
[thm1_6, thm2_6, thm3_6, thm4_6] = joint_control_4DOF([deg2rad(5) deg2rad(-55) deg2rad(-30) deg2rad(-20)],[0 0 0 0], t6,dt); %thumb

%% thumb to middle tip pt1
t7 = 3.5;

[f1m1_7, f1m2_7, f1m3_7, ~] = joint_control_3DOF([0 0 0],[0 0 0],t7,dt); %index
[f2m1_7, f2m2_7, f2m3_7, ~] = joint_control_3DOF([0 0 0],[deg2rad(10) deg2rad(-85) deg2rad(-30)],t7,dt); %middle
[f3m1_7, f3m2_7, f3m3_7, ~] = joint_control_3DOF([0 0 0],[0 0 0],t7,dt); %ring
[f4m1_7, f4m2_7, f4m3_7, ~] = joint_control_3DOF([0 0 0],[0 0 0],t7,dt); %pinkie
[thm1_7, thm2_7, thm3_7, thm4_7] = joint_control_4DOF([0 0 0 0],[deg2rad(-25) deg2rad(-67) deg2rad(-35) deg2rad(-15)],t7,dt);

%% thumb to middle tip pt 2
t8 = 3.5;

[f1m1_8, f1m2_8, f1m3_8, ~] = joint_control_3DOF([0 0 0],[0 0 0],t8,dt); %index
[f2m1_8, f2m2_8, f2m3_8, ~] = joint_control_3DOF([deg2rad(10) deg2rad(-85) deg2rad(-30)],[0 0 0],t8,dt); %middle
[f3m1_8, f3m2_8, f3m3_8, ~] = joint_control_3DOF([0 0 0],[0 0 0],t8,dt); %ring
[f4m1_8, f4m2_8, f4m3_8, ~] = joint_control_3DOF([0 0 0],[0 0 0],t8,dt); %pinkie
[thm1_8, thm2_8, thm3_8, thm4_8] = joint_control_4DOF([deg2rad(-25) deg2rad(-67) deg2rad(-35) deg2rad(-15)], [0 0 0 0], t8, dt); %thumb

%% thumb to ring tip pt1
t9 = 3.5;

[f1m1_9, f1m2_9, f1m3_9, ~] = joint_control_3DOF([0 0 0],[0 0 0],t9,dt); %index
[f2m1_9, f2m2_9, f2m3_9, ~] = joint_control_3DOF([0 0 0],[0 0 0],t9,dt); %middle
[f3m1_9, f3m2_9, f3m3_9, ~] = joint_control_3DOF([0 0 0],[deg2rad(15) deg2rad(-85) deg2rad(-37)],t9,dt); %ring
[f4m1_9, f4m2_9, f4m3_9, ~] = joint_control_3DOF([0 0 0],[0 0 0],t9,dt); %pinkie
[thm1_9, thm2_9, thm3_9, thm4_9] = joint_control_4DOF([0 0 0 0],[deg2rad(-25) deg2rad(-73) deg2rad(-50) deg2rad(-5)],t9,dt);

%% thumb to ring tip pt2
t10 = 3.5;

[f1m1_10, f1m2_10, f1m3_10, ~] = joint_control_3DOF([0 0 0],[0 0 0],t10,dt); %index
[f2m1_10, f2m2_10, f2m3_10, ~] = joint_control_3DOF([0 0 0],[0 0 0],t10,dt); %middle
[f3m1_10, f3m2_10, f3m3_10, ~] = joint_control_3DOF([deg2rad(15) deg2rad(-85) deg2rad(-37)],[0 0 0],t10,dt); %ring
[f4m1_10, f4m2_10, f4m3_10, ~] = joint_control_3DOF([0 0 0],[0 0 0],t10,dt); %pinkie
[thm1_10, thm2_10, thm3_10, thm4_10] = joint_control_4DOF([deg2rad(-25) deg2rad(-75) deg2rad(-49) deg2rad(-5)], [0 0 0 0], t10, dt); %thumb

%% thumb to pinkie tip pt1

t11 = 3.5;

[f1m1_11, f1m2_11, f1m3_11, ~] = joint_control_3DOF([0 0 0],[0 0 0],t11,dt); %index
[f2m1_11, f2m2_11, f2m3_11, ~] = joint_control_3DOF([0 0 0],[0 0 0],t11,dt); %middle
[f3m1_11, f3m2_11, f3m3_11, ~] = joint_control_3DOF([0 0 0],[0 0 0],t11,dt); %ring
[f4m1_11, f4m2_11, f4m3_11, ~] = joint_control_3DOF([0 0 0],[deg2rad(25) deg2rad(-90) deg2rad(-45)],t11,dt); %pinkie
[thm1_11, thm2_11, thm3_11, thm4_11] = joint_control_4DOF([0 0 0 0],[deg2rad(-25) deg2rad(-77) deg2rad(-57) deg2rad(-10)],t11,dt);


%% thumb to pinkie tip pt2
t12 = 3.5;

[f1m1_12, f1m2_12, f1m3_12, ~] = joint_control_3DOF([0 0 0],[0 0 0],t12,dt); %index
[f2m1_12, f2m2_12, f2m3_12, ~] = joint_control_3DOF([0 0 0],[0 0 0],t12,dt); %middle
[f3m1_12, f3m2_12, f3m3_12, ~] = joint_control_3DOF([0 0 0],[0 0 0],t12,dt); %ring
[f4m1_12, f4m2_12, f4m3_12, ~] = joint_control_3DOF([deg2rad(25) deg2rad(-90) deg2rad(-45)],[0 0 0],t12,dt); %pinkie
[thm1_12, thm2_12, thm3_12, thm4_12] = joint_control_4DOF([deg2rad(-25) deg2rad(-80) deg2rad(-55) deg2rad(-10)], [0 0 0 0], t12, dt); %thumb

%% thumb to index j4 pt1
t13 = 3.5;

[f1m1_13, f1m2_13, f1m3_13, ~] = joint_control_3DOF([0 0 0],[0 deg2rad(-90) deg2rad(-20)],t13,dt); %index
[f2m1_13, f2m2_13, f2m3_13, ~] = joint_control_3DOF([0 0 0],[0 0 0],t13,dt); %middle
[f3m1_13, f3m2_13, f3m3_13, ~] = joint_control_3DOF([0 0 0],[0 0 0],t13,dt); %ring
[f4m1_13, f4m2_13, f4m3_13, ~] = joint_control_3DOF([0 0 0],[0 0 0],t13,dt); %pinkie
[thm1_13, thm2_13, thm3_13, thm4_13] = joint_control_4DOF([0 0 0 0],[deg2rad(35) deg2rad(-37) deg2rad(-50) deg2rad(-20)],t13,dt);

%% thumb to index j4 pt2

t14 = 3.5;

[f1m1_14, f1m2_14, f1m3_14, ~] = joint_control_3DOF([0 deg2rad(-90) deg2rad(-20)],[0 0 0],t14,dt); %index
[f2m1_14, f2m2_14, f2m3_14, ~] = joint_control_3DOF([0 0 0],[0 0 0],t14,dt); %middle
[f3m1_14, f3m2_14, f3m3_14, ~] = joint_control_3DOF([0 0 0],[0 0 0],t14,dt); %ring
[f4m1_14, f4m2_14, f4m3_14, ~] = joint_control_3DOF([0 0 0],[0 0 0],t14,dt); %pinkie
[thm1_14, thm2_14, thm3_14, thm4_14] = joint_control_4DOF([deg2rad(35) deg2rad(-37) deg2rad(-50) deg2rad(-20)], [0 0 0 0], t14, dt); %thumb


%% stitch path together
pt1 = 1;
pt2 = 1;
pt3 = 1;
pt4 = 1;
pt5 = 1;
pt6 = 1;

f1m1 = stitch({f1m1_0, paws(f1m1_0,1,dt), f1m1_1, paws(f1m1_1,1,dt), f1m1_2, paws(f1m1_2,1,dt), f1m1_3, paws(f1m1_3,pt3,dt), f1m1_4, paws(f1m1_4,pt3,dt), f1m1_5, paws(f1m1_5,pt4,dt), f1m1_6, paws(f1m1_6,pt4,dt), f1m1_7, paws(f1m1_7,pt5,dt), f1m1_8, paws(f1m1_8,pt5,dt), f1m1_9, paws(f1m1_9,pt6,dt), f1m1_10, paws(f1m1_10,pt6,dt), f1m1_11, paws(f1m1_11,pt6,dt), f1m1_12, paws(f1m1_12,pt6,dt), f1m1_13, paws(f1m1_13,pt6,dt), f1m1_14, paws(f1m1_14,pt6,dt)},dt);
f1m2 = stitch({f1m2_0, paws(f1m2_0,1,dt), f1m2_1, paws(f1m2_1,1,dt), f1m2_2, paws(f1m2_2,1,dt), f1m2_3, paws(f1m2_3,pt3,dt), f1m2_4, paws(f1m2_4,pt3,dt), f1m2_5, paws(f1m2_5,pt4,dt), f1m2_6, paws(f1m2_6,pt4,dt), f1m2_7, paws(f1m2_7,pt5,dt), f1m2_8, paws(f1m2_8,pt5,dt), f1m2_9, paws(f1m2_9,pt6,dt), f1m2_10, paws(f1m2_10,pt6,dt), f1m2_11, paws(f1m2_11,pt6,dt), f1m2_12, paws(f1m2_12,pt6,dt), f1m2_13, paws(f1m2_13,pt6,dt), f1m2_14, paws(f1m2_14,pt6,dt)},dt);
f1m3 = stitch({f1m3_0, paws(f1m2_0,1,dt), f1m3_1, paws(f1m3_1,1,dt), f1m3_2, paws(f1m3_2,1,dt), f1m3_3, paws(f1m3_3,pt3,dt), f1m3_4, paws(f1m3_4,pt3,dt), f1m3_5, paws(f1m3_5,pt4,dt), f1m3_6, paws(f1m3_6,pt4,dt), f1m3_7, paws(f1m3_7,pt5,dt), f1m3_8, paws(f1m3_8,pt5,dt), f1m3_9, paws(f1m3_9,pt6,dt), f1m3_10, paws(f1m3_10,pt6,dt), f1m3_11, paws(f1m3_11,pt6,dt), f1m3_12, paws(f1m3_12,pt6,dt), f1m3_13, paws(f1m3_13,pt6,dt), f1m3_14, paws(f1m3_14,pt6,dt)},dt);

f2m1 = stitch({f2m1_0, paws(f2m1_0,1,dt), f2m1_1, paws(f2m1_1,1,dt), f2m1_2, paws(f2m1_2,1,dt), f2m1_3, paws(f2m1_3,pt3,dt), f2m1_4, paws(f2m1_4,pt3,dt), f2m1_5, paws(f2m1_5,pt4,dt), f2m1_6, paws(f2m1_6,pt4,dt), f2m1_7, paws(f2m1_7,pt5,dt), f2m1_8, paws(f2m1_8,pt5,dt), f2m1_9, paws(f2m1_9,pt6,dt), f2m1_10, paws(f2m1_10,pt6,dt), f2m1_11, paws(f2m1_11,pt6,dt), f2m1_12, paws(f2m1_12,pt6,dt), f2m1_13, paws(f2m1_13,pt6,dt), f2m1_14, paws(f2m1_14,pt6,dt)},dt);
f2m2 = stitch({f2m2_0, paws(f2m2_0,1,dt), f2m2_1, paws(f2m2_1,1,dt), f2m2_2, paws(f2m2_2,1,dt), f2m2_3, paws(f2m2_3,pt3,dt), f2m2_4, paws(f2m2_4,pt3,dt), f2m2_5, paws(f2m2_5,pt4,dt), f2m2_6, paws(f2m2_6,pt4,dt), f2m2_7, paws(f2m2_7,pt5,dt), f2m2_8, paws(f2m2_8,pt5,dt), f2m2_9, paws(f2m2_9,pt6,dt), f2m2_10, paws(f2m2_10,pt6,dt), f2m2_11, paws(f2m2_11,pt6,dt), f2m2_12, paws(f2m2_12,pt6,dt), f2m2_13, paws(f2m2_13,pt6,dt), f2m2_14, paws(f2m2_14,pt6,dt)},dt);
f2m3 = stitch({f2m3_0, paws(f2m3_0,1,dt), f2m3_1, paws(f2m3_1,1,dt), f2m3_2, paws(f2m3_2,1,dt), f2m3_3, paws(f2m3_3,pt3,dt), f2m3_4, paws(f2m3_4,pt3,dt), f2m3_5, paws(f2m3_5,pt4,dt), f2m3_6, paws(f2m3_6,pt4,dt), f2m3_7, paws(f2m3_7,pt5,dt), f2m3_8, paws(f2m3_8,pt5,dt), f2m3_9, paws(f2m3_9,pt6,dt), f2m3_10, paws(f2m3_10,pt6,dt), f2m3_11, paws(f2m3_11,pt6,dt), f2m3_12, paws(f2m3_12,pt6,dt), f2m3_13, paws(f2m3_13,pt6,dt), f2m3_14, paws(f2m3_14,pt6,dt)},dt);

f3m1 = stitch({f3m1_0, paws(f3m1_0,1,dt), f3m1_1, paws(f3m1_1,1,dt), f3m1_2, paws(f3m1_2,1,dt), f3m1_3, paws(f3m1_3,pt3,dt), f3m1_4, paws(f3m1_4,pt3,dt), f3m1_5, paws(f3m1_5,pt4,dt), f3m1_6, paws(f3m1_6,pt4,dt), f3m1_7, paws(f3m1_7,pt5,dt), f3m1_8, paws(f3m1_8,pt5,dt), f3m1_9, paws(f3m1_9,pt6,dt), f3m1_10, paws(f3m1_10,pt6,dt), f3m1_11, paws(f3m1_11,pt6,dt), f3m1_12, paws(f3m1_12,pt6,dt), f3m1_13, paws(f3m1_13,pt6,dt), f3m1_14, paws(f3m1_14,pt6,dt)},dt);
f3m2 = stitch({f3m2_0, paws(f3m2_0,1,dt), f3m2_1, paws(f3m2_1,1,dt), f3m2_2, paws(f3m2_2,1,dt), f3m2_3, paws(f3m2_3,pt3,dt), f3m2_4, paws(f3m2_4,pt3,dt), f3m2_5, paws(f3m2_5,pt4,dt), f3m2_6, paws(f3m2_6,pt4,dt), f3m2_7, paws(f3m2_7,pt5,dt), f3m2_8, paws(f3m2_8,pt5,dt), f3m2_9, paws(f3m2_9,pt6,dt), f3m2_10, paws(f3m2_10,pt6,dt), f3m2_11, paws(f3m2_11,pt6,dt), f3m2_12, paws(f3m2_12,pt6,dt), f3m2_13, paws(f3m2_13,pt6,dt), f3m2_14, paws(f3m2_14,pt6,dt)},dt);
f3m3 = stitch({f3m3_0, paws(f3m3_0,1,dt), f3m3_1, paws(f3m3_1,1,dt), f3m3_2, paws(f3m3_2,1,dt), f3m3_3, paws(f3m3_3,pt3,dt), f3m3_4, paws(f3m3_4,pt3,dt), f3m3_5, paws(f3m3_5,pt4,dt), f3m3_6, paws(f3m3_6,pt4,dt), f3m3_7, paws(f3m3_7,pt5,dt), f3m3_8, paws(f3m3_8,pt5,dt), f3m3_9, paws(f3m3_9,pt6,dt), f3m3_10, paws(f3m3_10,pt6,dt), f3m3_11, paws(f3m3_11,pt6,dt), f3m3_12, paws(f3m3_12,pt6,dt), f3m3_13, paws(f3m3_13,pt6,dt), f3m3_14, paws(f3m3_14,pt6,dt)},dt);

f4m1 = stitch({f4m1_0, paws(f4m1_0,1,dt), f4m1_1, paws(f4m1_1,1,dt), f4m1_2, paws(f4m1_2,1,dt), f4m1_3, paws(f4m1_3,pt3,dt), f4m1_4, paws(f4m1_4,pt3,dt), f4m1_5, paws(f4m1_5,pt4,dt), f4m1_6, paws(f4m1_6,pt4,dt), f4m1_7, paws(f4m1_7,pt5,dt), f4m1_8, paws(f4m1_8,pt5,dt), f4m1_9, paws(f4m1_9,pt6,dt), f4m1_10, paws(f4m1_10,pt6,dt), f4m1_11, paws(f4m1_11,pt6,dt), f4m1_12, paws(f4m1_12,pt6,dt), f4m1_13, paws(f4m1_13,pt6,dt), f4m1_14, paws(f4m1_14,pt6,dt)},dt);
f4m2 = stitch({f4m2_0, paws(f4m2_0,1,dt), f4m2_1, paws(f4m2_1,1,dt), f4m2_2, paws(f4m2_2,1,dt), f4m2_3, paws(f4m2_3,pt3,dt), f4m2_4, paws(f4m2_4,pt3,dt), f4m2_5, paws(f4m2_5,pt4,dt), f4m2_6, paws(f4m2_6,pt4,dt), f4m2_7, paws(f4m2_7,pt5,dt), f4m2_8, paws(f4m2_8,pt5,dt), f4m2_9, paws(f4m2_9,pt6,dt), f4m2_10, paws(f4m2_10,pt6,dt), f4m2_11, paws(f4m2_11,pt6,dt), f4m2_12, paws(f4m2_12,pt6,dt), f4m2_13, paws(f4m2_13,pt6,dt), f4m2_14, paws(f4m2_14,pt6,dt)},dt);
f4m3 = stitch({f4m3_0, paws(f4m3_0,1,dt), f4m3_1, paws(f4m3_1,1,dt), f4m3_2, paws(f4m3_2,1,dt), f4m3_3, paws(f4m3_3,pt3,dt), f4m3_4, paws(f4m3_4,pt3,dt), f4m3_5, paws(f4m3_5,pt4,dt), f4m3_6, paws(f4m3_6,pt4,dt), f4m3_7, paws(f4m3_7,pt5,dt), f4m3_8, paws(f4m3_8,pt5,dt), f4m3_9, paws(f4m3_9,pt6,dt), f4m3_10, paws(f4m3_10,pt6,dt), f4m3_11, paws(f4m3_11,pt6,dt), f4m3_12, paws(f4m3_12,pt6,dt), f4m3_13, paws(f4m3_13,pt6,dt), f4m3_14, paws(f4m3_14,pt6,dt)},dt);

thm1 = stitch({thm1_0, paws(thm1_0,1,dt), thm1_1, paws(thm1_1,1,dt), thm1_2, paws(thm1_2,1,dt), thm1_3, paws(thm1_3,pt3,dt), thm1_4, paws(thm1_4,pt3,dt), thm1_5, paws(thm1_5,pt4,dt), thm1_6, paws(thm1_6,pt4,dt), thm1_7, paws(thm1_7,pt5,dt), thm1_8, paws(thm1_8,pt5,dt), thm1_9, paws(thm1_9,pt6,dt), thm1_10, paws(thm1_10,pt6,dt), thm1_11, paws(thm1_11,pt6,dt), thm1_12, paws(thm1_12,pt6,dt), thm1_13, paws(thm1_13,pt6,dt), thm1_14, paws(thm1_14,pt6,dt)},dt);
thm2 = stitch({thm2_0, paws(thm2_0,1,dt), thm2_1, paws(thm2_1,1,dt), thm2_2, paws(thm2_2,1,dt), thm2_3, paws(thm2_3,pt3,dt), thm2_4, paws(thm2_4,pt3,dt), thm2_5, paws(thm2_5,pt4,dt), thm2_6, paws(thm2_6,pt4,dt), thm2_7, paws(thm2_7,pt5,dt), thm2_8, paws(thm2_8,pt5,dt), thm2_9, paws(thm2_9,pt6,dt), thm2_10, paws(thm2_10,pt6,dt), thm2_11, paws(thm2_11,pt6,dt), thm2_12, paws(thm2_12,pt6,dt), thm2_13, paws(thm2_13,pt6,dt), thm2_14, paws(thm2_14,pt6,dt)},dt);
thm3 = stitch({thm3_0, paws(thm3_0,1,dt), thm3_1, paws(thm3_1,1,dt), thm3_2, paws(thm3_2,1,dt), thm3_3, paws(thm3_3,pt3,dt), thm3_4, paws(thm3_4,pt3,dt), thm3_5, paws(thm3_5,pt4,dt), thm3_6, paws(thm3_6,pt4,dt), thm3_7, paws(thm3_7,pt5,dt), thm3_8, paws(thm3_8,pt5,dt), thm3_9, paws(thm3_9,pt6,dt), thm3_10, paws(thm3_10,pt6,dt), thm3_11, paws(thm3_11,pt6,dt), thm3_12, paws(thm3_12,pt6,dt), thm3_13, paws(thm3_13,pt6,dt), thm3_14, paws(thm3_14,pt6,dt)},dt);
thm4 = stitch({thm4_0, paws(thm4_0,1,dt), thm4_1, paws(thm4_1,1,dt), thm4_2, paws(thm4_2,1,dt), thm4_3, paws(thm4_3,pt3,dt), thm4_4, paws(thm4_4,pt3,dt), thm4_5, paws(thm4_5,pt4,dt), thm4_6, paws(thm4_6,pt4,dt), thm4_7, paws(thm4_7,pt5,dt), thm4_8, paws(thm4_8,pt5,dt), thm4_9, paws(thm4_9,pt6,dt), thm4_10, paws(thm4_10,pt6,dt), thm4_11, paws(thm4_11,pt6,dt), thm4_12, paws(thm4_12,pt6,dt), thm4_13, paws(thm4_13,pt6,dt), thm4_14, paws(thm4_14,pt6,dt)},dt);

total_time = size(f1m1,1)*dt;

%% plot motor velocities

f1m1dt = calc_motor_velocity(f1m1);
f1m2dt = calc_motor_velocity(f1m2);
f1m3dt = calc_motor_velocity(f1m3);

f2m1dt = calc_motor_velocity(f2m1);
f2m2dt = calc_motor_velocity(f2m2);
f2m3dt = calc_motor_velocity(f2m3);

f3m1dt = calc_motor_velocity(f3m1);
f3m2dt = calc_motor_velocity(f3m2);
f3m3dt = calc_motor_velocity(f3m3);

f4m1dt = calc_motor_velocity(f4m1);
f4m2dt = calc_motor_velocity(f4m2);
f4m3dt = calc_motor_velocity(f4m3);

thm1dt = calc_motor_velocity(thm1);
thm2dt = calc_motor_velocity(thm2);
thm3dt = calc_motor_velocity(thm3);
thm4dt = calc_motor_velocity(thm4);

%%
figure(1);
hold on;

% Right y-axis for M1 (in rad/s)
yyaxis right
h1 = plot(f1m1dt(:,1), f1m1dt(:,2), 'r', 'LineWidth', 2); % Red for M1
ylabel('Angular Velocity (rad/s)'); % Label for right y-axis
ylim([-pi pi]);

% Left y-axis for M2, M3 (in mm/s)
yyaxis left
h2 = plot(f1m2dt(:,1), f1m2dt(:,2), 'g', 'LineWidth', 2); % Green for M2
hold on;
h3 = plot(f1m3dt(:,1), f1m3dt(:,2), 'b', 'LineWidth', 2); % Blue for M3
ylabel('Velocity (mm/s)'); % Label for left y-axis
ylim([-10, 10]);

% Common settings
xlabel('Time (s)'); % X-axis label
legend([h1, h2, h3], {'M1', 'M2', 'M3'}, 'Location', 'best'); % Explicitly assign handles
set(gca, 'Color', 'w'); % Set axes background to white
set(gcf, 'Color', 'w'); % Set figure background to white
xlim([0 48]);
title("Index Finger Motor Velocities")
hold off;

%%
figure(2);
hold on;

% Right y-axis for M1 (in rad/s)
yyaxis right
h1 = plot(f2m1dt(:,1), f2m1dt(:,2), 'r', 'LineWidth', 2); % Red for M1
ylabel('Angular Velocity (rad/s)'); % Label for right y-axis
ylim([-pi pi]);

% Left y-axis for M2, M3 (in mm/s)
yyaxis left
h2 = plot(f2m2dt(:,1), f2m2dt(:,2), 'g', 'LineWidth', 2); % Green for M2
hold on;
h3 = plot(f2m3dt(:,1), f2m3dt(:,2), 'b', 'LineWidth', 2); % Blue for M3
ylabel('Velocity (mm/s)'); % Label for left y-axis
ylim([-10, 10]);

% Common settings
xlabel('Time (s)'); % X-axis label
legend([h1, h2, h3], {'M1', 'M2', 'M3'}, 'Location', 'best'); % Explicitly assign handles
set(gca, 'Color', 'w'); % Set axes background to white
set(gcf, 'Color', 'w'); % Set figure background to white
xlim([0 48]);
title("Middle Finger Motor Velocities")
hold off;

%%
figure(3);
hold on;

% Right y-axis for M1 (in rad/s)
yyaxis right
h1 = plot(f3m1dt(:,1), f3m1dt(:,2), 'r', 'LineWidth', 2); % Red for M1
ylabel('Angular Velocity (rad/s)'); % Label for right y-axis
ylim([-pi pi]);

% Left y-axis for M2, M3 (in mm/s)
yyaxis left
h2 = plot(f3m2dt(:,1), f3m2dt(:,2), 'g', 'LineWidth', 2); % Green for M2
hold on;
h3 = plot(f3m3dt(:,1), f3m3dt(:,2), 'b', 'LineWidth', 2); % Blue for M3
ylabel('Velocity (mm/s)'); % Label for left y-axis
ylim([-10, 10]);

% Common settings
xlabel('Time (s)'); % X-axis label
legend([h1, h2, h3], {'M1', 'M2', 'M3'}, 'Location', 'best'); % Explicitly assign handles
set(gca, 'Color', 'w'); % Set axes background to white
set(gcf, 'Color', 'w'); % Set figure background to white
xlim([0 48]);
title("Ring Finger Motor Velocities")
hold off;

%%
figure(4);
hold on;

% Right y-axis for M1 (in rad/s)
yyaxis right
h1 = plot(f4m1dt(:,1), f4m1dt(:,2), 'r', 'LineWidth', 2); % Red for M1
ylabel('Angular Velocity (rad/s)'); % Label for right y-axis
ylim([-pi pi]);

% Left y-axis for M2, M3 (in mm/s)
yyaxis left
h2 = plot(f4m2dt(:,1), f4m2dt(:,2), 'g', 'LineWidth', 2); % Green for M2
hold on;
h3 = plot(f4m3dt(:,1), f4m3dt(:,2), 'b', 'LineWidth', 2); % Blue for M3
ylabel('Velocity (mm/s)'); % Label for left y-axis
ylim([-10, 10]);

% Common settings
xlabel('Time (s)'); % X-axis label
legend([h1, h2, h3], {'M1', 'M2', 'M3'}, 'Location', 'best'); % Explicitly assign handles
set(gca, 'Color', 'w'); % Set axes background to white
set(gcf, 'Color', 'w'); % Set figure background to white
xlim([0 48]);
title("Pinkie Finger Motor Velocities")
hold off;

%%
figure(5);
hold on;

% Right y-axis for M1 (in rad/s)
yyaxis right
h1 = plot(thm1dt(:,1), thm1dt(:,2), 'r', 'LineWidth', 2); % Red for M1
ylabel('Angular Velocity (rad/s)'); % Label for right y-axis
ylim([-pi pi])

% Left y-axis for M2, M3, and M4 (in mm/s)
yyaxis left
h2 = plot(thm2dt(:,1), thm2dt(:,2), 'g', 'LineWidth', 2); % Green for M2
hold on;
h3 = plot(thm3dt(:,1), thm3dt(:,2), 'b', 'LineWidth', 2); % Blue for M3
h4 = plot(thm4dt(:,1), thm4dt(:,2), 'k', 'LineWidth', 2); % Black for M4
ylabel('Velocity (mm/s)'); % Label for left y-axis
ylim([-15 15])

% Common settings
xlabel('Time (s)'); % X-axis label
legend([h1, h2, h3, h4], {'M1', 'M2', 'M3', 'M4'}, 'Location', 'best'); % Explicitly assign handles
set(gca, 'Color', 'w'); % Set axes background to white
set(gcf, 'Color', 'w'); % Set figure background to white
xlim([0 48]);
title("Thumb Motor Velocities")
hold off;
