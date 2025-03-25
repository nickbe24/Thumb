clear

[Positions_t, boundary_t, distances_t] = find_thumb_workspace(-40,40,0,-80,0,-90,0,-90, 1, 60, 45,30);

origin1 = [0; 0; 0; 1];
surfaces_t = [Positions_t;ones(1,size(Positions_t,2))];
boundary_t = [boundary_t; ones(1,size(boundary_t,2))];

close all

colormap('jet')

cb = colorbar(); 
ylabel(cb,'Distance from MCP joint (mm)','FontSize',16,'Rotation',270)
axis("equal")
hold on
%title("Workspace of a Finger")
set(gcf,'Color','white')

% thumb

scatter3(surfaces_t(1, :),surfaces_t(2, :),surfaces_t(3, :), 10, distances_t', 'filled')
scatter3(boundary_t(1, :), boundary_t(2, :), boundary_t(3, :), 15, ...
    zeros(size(boundary_t,2), 3), 'filled')
scatter3(origin1(1), origin1(2), origin1(3), 30, [0.3, 0.3, 0.3], 'filled')


view(3)

