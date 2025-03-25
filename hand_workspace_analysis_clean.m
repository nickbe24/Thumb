clear

[Positions_f, boundary_f, distances_f] = find_finger_workspace(-25,25,0,-90,0,-90, .5, 40,30, 25); %finger workspace
[Positions_t, boundary_t, distances_t] = find_thumb_workspace(-45,45,0,-90,0,-90,0,-90, .5, 50, 45, 30); %60 45 30

origin1 = [0; 0; 0; 1];

surfaces_f = [Positions_f; ones(1, size(Positions_f, 2))];
surfaces_t = [Positions_t;ones(1,size(Positions_t,2))];

boundary_f = [boundary_f; ones(1, size(boundary_f, 2))];
boundary_t = [boundary_t; ones(1,size(boundary_t,2))];

index_y_ang = 0;
Transform5 = [cosd(index_y_ang), 0, -sind(index_y_ang), 0; %ring finger
              0, 1, 0, 0; 
              sind(index_y_ang), 0, cosd(index_y_ang), 0; 
              0, 0, 0, 1];

ring_y_ang = -5; % -5
pinky_y_ang = -10; %-10
thumb_y_ang = 70; %70 
thumb_z_ang = 55; %50

Transform1 = [1, 0, 0, 32; %middle finger %32
              0, 1, 0, 0; 
              0, 0, 1, 0; 
              0, 0, 0, 1];

Transform3 = [cosd(ring_y_ang), 0, -sind(ring_y_ang), 59; %ring finger 59
              0, 1, 0, 0; 
              sind(ring_y_ang), 0, cosd(ring_y_ang), -5; % -5
              0, 0, 0, 1];

Transform4 = [cosd(pinky_y_ang), 0, -sind(pinky_y_ang), 86; %pinky finger 86
              0, 1, 0, 0; 
              sind(pinky_y_ang), 0, cosd(pinky_y_ang), -10; %-10
              0, 0, 0, 1];

R_y = [cosd(thumb_y_ang) 0 -sind(thumb_y_ang); 0 1 0; sind(thumb_y_ang) 0 cosd(thumb_y_ang)];
R_z = [cosd(thumb_z_ang) sind(thumb_z_ang) 0; -sind(thumb_z_ang) cosd(thumb_z_ang) 0; 0 0 1];
R_thumb = R_y*R_z;
p_thumb = [-20; 5.5; -120]; %-20 5.5 -120

Transform2 = [R_thumb p_thumb; 0 0 0 1];

origin2 = Transform1 * origin1;
surfaces2 = Transform1 * surfaces_f;
boundary2 = Transform1 * boundary_f;

origin3 = Transform2 * origin1;
surfaces3 = Transform2 * surfaces_t;
boundary3 = Transform2 * boundary_t;

origin4 = Transform3 * origin1;
surfaces4 = Transform3 * surfaces_f;
boundary4 = Transform3 * boundary_f;

origin5 = Transform4 * origin1;
surfaces5 = Transform4 * surfaces_f;
boundary5 = Transform4 * boundary_f;

origin6 = Transform5* origin1;
surfaces6 = Transform5* surfaces_f;
boundary6 = Transform5 * boundary_f;
close all

colormap('jet')

scatter3(surfaces6(1, :),surfaces6(2, :),surfaces6(3, :), 10, distances_f', 'filled')
cb = colorbar(); 
ylabel(cb,'Distance from MCP joint (mm)','FontSize',16,'Rotation',270)
axis("equal")
hold on
scatter3(origin6(1), origin6(2), origin6(3), 30, [0.3, 0.3, 0.3], 'filled')
scatter3(boundary6(1, :), boundary6(2, :), boundary6(3, :), 15, ...
zeros(size(boundary6,2), 3), 'filled')

set(gcf,'Color','white')
view(3);
view(-140,30);

%middle

scatter3(surfaces2(1, :),surfaces2(2, :),surfaces2(3, :), 10, distances_f', 'filled')
scatter3(boundary2(1, :), boundary2(2, :), boundary2(3, :), 15, ...
    zeros(size(boundary_f,2), 3), 'filled')
scatter3(origin2(1), origin2(2), origin2(3), 30, [0.3, 0.3, 0.3], 'filled')

% thumb

scatter3(surfaces3(1, :),surfaces3(2, :),surfaces3(3, :), 10, distances_t', 'filled')
scatter3(boundary3(1, :), boundary3(2, :), boundary3(3, :), 15, ...
    zeros(size(boundary_t,2), 3), 'filled')
scatter3(origin3(1), origin3(2), origin3(3), 30, [0.3, 0.3, 0.3], 'filled')

%ring
scatter3(surfaces4(1, :),surfaces4(2, :),surfaces4(3, :), 10, distances_f', 'filled')
scatter3(boundary4(1, :), boundary4(2, :), boundary4(3, :), 15, ...
    zeros(size(boundary_f,2), 3), 'filled')
scatter3(origin4(1), origin4(2), origin4(3), 30, [0.3, 0.3, 0.3], 'filled')

%pinky
scatter3(surfaces5(1, :),surfaces5(2, :),surfaces5(3, :), 10, distances_f', 'filled')
scatter3(boundary5(1, :), boundary5(2, :), boundary5(3, :), 15, ...
    zeros(size(boundary_f,2), 3), 'filled')
scatter3(origin5(1), origin5(2), origin5(3), 30, [0.3, 0.3, 0.3], 'filled')


