clear

start_pos = [0,70,70,deg2rad(-125)];
end_pos_1 = [0,80,70,deg2rad(-125)];
dt = .01;
time = 2; % second
error_goal_lin = .1; % mm
error_goal_angle = .01; % rad

[shortm, m1, m2, m3, m4, actual_end_pos, ang, mv] = gen_motor(start_pos, end_pos_1, dt, time, error_goal_lin,error_goal_angle);

total_time = length(m1)*dt;

%% get position analysis

X = out.simout.data;
Y = out.simout1.data;
Z = out.simout2.data;
alpha = out.simout3.data;

m1_vel = [m1(:,1), mv(:,1)];
m2_vel = [m1(:,1), mv(:,2)];
m3_vel = [m1(:,1), mv(:,3)];
m4_vel = [m1(:,1), mv(:,4)];
alpha = alpha(1:10:end-1);
alpha = -rad2deg(alpha);
figure(1)
title("Thumb Tip Position in the YZ Plane")
hold on
plot(X, Y, 'LineWidth', 2);
axis([60 90 60 80])
xlabel("Distance from MCP joint in the Y axis (mm)")
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
xlim([0, 2]);
title("Motor Velocities")
hold off;

figure(3)
title("Thumb Tip Angle (Alpha) in the YZ Plane")
hold on
plot(m1(:,1), alpha, 'LineWidth', 2);
axis([0 2 -130 -120])
xlabel("Time (s)")
ylabel("Thumb Tip Angle (degrees)")
set(gca, 'Color', 'w');     % Set axes background to white
set(gcf, 'Color', 'w');     % Set figure background to white
