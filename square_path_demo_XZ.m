clear

start_pos = [15,90,90,deg2rad(-60)];
end_pos_1 = [-15,90,90,deg2rad(-60)];
end_pos_2 = [-15,90,60,deg2rad(-110)];
end_pos_3 = [15,90,60,deg2rad(-110)];
dt = .01;
time = 2; % second
error_goal_lin = .25; % mm
error_goal_angle = .025; % rad

[shortm_1, m1_1, m2_1, m3_1, m4_1, actual_end_pos_1, ang_1, mv_1] = gen_motor(start_pos, end_pos_1, dt, time, error_goal_lin,error_goal_angle);
[shortm_2, m1_2, m2_2, m3_2, m4_2, actual_end_pos_2, ang_2, mv_2] = gen_motor(actual_end_pos_1, end_pos_2, dt, time, error_goal_lin,error_goal_angle);
[shortm_3, m1_3, m2_3, m3_3, m4_3, actual_end_pos_3, ang_3, mv_3] = gen_motor(actual_end_pos_2, end_pos_3, dt, time, error_goal_lin,error_goal_angle);
[shortm_4, m1_4, m2_4, m3_4, m4_4, actual_end_pos_4, ang_4, mv_4] = gen_motor(actual_end_pos_3, start_pos, dt, time, error_goal_lin,error_goal_angle);

%% Concat motors positions
m1 = [m1_1; m1_2; m1_3; m1_4];
m2 = [m2_1; m2_2; m2_3; m2_4];
m3 = [m3_1; m3_2; m3_3; m3_4];
m4 = [m4_1; m4_2; m4_3; m4_4];
total_time = length(m1)*dt;
time_index = 0:dt:total_time-dt;
m1(:,1) = time_index';
m2(:,1) = time_index';
m3(:,1) = time_index';
m4(:,1) = time_index';
%% get position analysis

X = out.simout.data;
Y = out.simout1.data;
Z = out.simout2.data;

m1_vel = [mv_1(:,1); mv_2(:,1); mv_3(:,1); mv_4(:,1)];
m1_vel = [m1(:,1), m1_vel];
m2_vel = [mv_1(:,2); mv_2(:,2); mv_3(:,2); mv_4(:,2)];
m2_vel = [m1(:,1), m2_vel];
m3_vel = [mv_1(:,3); mv_2(:,3); mv_3(:,3); mv_4(:,3)];
m3_vel = [m1(:,1), m3_vel];
m4_vel = [mv_1(:,4); mv_2(:,4); mv_3(:,4); mv_4(:,4)];
m4_vel = [m1(:,1), m4_vel];

figure(1)
title("Thumb Tip Position in the XZ Plane")
hold on
plot(Z, Y, 'LineWidth', 2);
axis([-35 35 40 110])
xlabel("Distance from MCP joint in the X axis (mm)")
ylabel("Distance from MCP joint in the Z axis (mm)")
set(gca, 'Color', 'w');     % Set axes background to white
set(gcf, 'Color', 'w');     % Set figure background to white

% Plot motor velocities
figure(2);
hold on;

% Right y-axis for M1 (in rad/s)
yyaxis right
h1 = plot(m1_vel(:,1), m1_vel(:,2), 'r', 'LineWidth', 2); % Red for M1
ylabel('Angular Velocity (rad/s)'); % Label for right y-axis
ylim([-.5 .5]);

% Left y-axis for M2, M3, and M4 (in mm/s)
yyaxis left
h2 = plot(m2_vel(:,1), m2_vel(:,2), 'g', 'LineWidth', 2); % Green for M2
hold on;
h3 = plot(m3_vel(:,1), m3_vel(:,2), 'b', 'LineWidth', 2); % Blue for M3
h4 = plot(m4_vel(:,1), m4_vel(:,2), 'k', 'LineWidth', 2); % Black for M4
ylabel('Velocity (mm/s)'); % Label for left y-axis
ylim([-10, 10]);

% Common settings
xlabel('Time (s)'); % X-axis label
legend([h1, h2, h3, h4], {'M1', 'M2', 'M3', 'M4'}, 'Location', 'best'); % Explicitly assign handles
set(gca, 'Color', 'w'); % Set axes background to white
set(gcf, 'Color', 'w'); % Set figure background to white
xlim([0, 8]);
title("Motor Velocities")
hold off;


