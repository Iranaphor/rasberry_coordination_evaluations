clc
opts = detectImportOptions('Documents/MATLAB/node_list.csv');
N = readtable('Documents/MATLAB/node_list.csv',opts);

opts = detectImportOptions('Documents/MATLAB/edge_list.csv');
E = readtable('Documents/MATLAB/edge_list.csv',opts);

% Plot Nodes and Edges of map
clf; cla; ax1=subplot(1,5,[1,2]);

plot3([E.start_x, E.end_x]', [E.start_y, E.end_y]', ([E.end_y, E.end_y]'.*0)+00, 'Color','#0076a8', 'LineWidth',0.01, 'LineStyle','-');
hold on

timesteps = 0:30:120;
for i = timesteps
    if i ~= 0
        %plot3([E.start_x, E.end_x]', [E.start_y, E.end_y]', ([E.end_y, E.end_y]'.*0)+i, 'Color', [0, .4, .68, .25], 'LineWidth',0.2, 'LineStyle','-');
        plot3([E.start_x, E.end_x]', [E.start_y, E.end_y]', ([E.end_y, E.end_y]'.*0)+i, 'Color', [0, .4, .68], 'LineWidth',0.1, 'LineStyle','-');
    end
end

% Plot Routes
% r1 = ['WayPoint141', 'WayPoint140', 'WayPoint74', 'WayPoint66', 'WayPoint56', 'WayPoint63', 'r10-ca', 'r10-cb'];
robot_01 = array2table([
 ["0",  "WayPoint74", "WayPoint74", "WayPoint74_WayPoint66", 0, 0]
 ["5",            "", "WayPoint74", "WayPoint74_WayPoint66", 0, 0]
 ["30",           "", "WayPoint66", "WayPoint74_WayPoint66", 0, 0]
 ["35", "WayPoint66", "WayPoint66", "WayPoint74_WayPoint66", 0, 0]
 ["35", "WayPoint66", "WayPoint66",                      "", 0, 0]
 ["40", "WayPoint66", "WayPoint66", "WayPoint66_WayPoint56", 0, 0]
 ["45",           "", "WayPoint66", "WayPoint66_WayPoint56", 0, 0]
 ["60",           "", "WayPoint56", "WayPoint66_WayPoint56", 0, 0]
 ["65", "WayPoint56", "WayPoint56", "WayPoint66_WayPoint56", 0, 0]
 ["70", "WayPoint56", "WayPoint56",                      "", 0, 0]
 ["72", "WayPoint56", "WayPoint56",      "WayPoint56_r9-ca", 0, 0]
 ["80",           "", "WayPoint56",      "WayPoint56_r9-ca", 0, 0]
 ["88",           "",      "r9-ca",      "WayPoint56_r9-ca", 0, 0]
 ["93",      "r9-ca",      "r9-ca",      "WayPoint56_r9-ca", 0, 0]
 ["95",      "r9-ca",      "r9-ca",                      "", 0, 0]
 ["103",      "r9-ca",      "r9-ca",           "r9-ca_r9-cb", 0, 0]
 ["107",           "",      "r9-ca",           "r9-ca_r9-cb", 0, 0]
 ["113",           "",      "r9-cb",           "r9-ca_r9-cb", 0, 0]
 ["119",      "r9-cb",      "r9-cb",           "r9-ca_r9-cb", 0, 0]
], "VariableNames",["time", "CurrentNode", "ClosestNode", "ClosestEdge", "x", "y"]);
plot_robot_path(robot_01, N, E, timesteps, [1,0,0], [.75,0,0])


% Plot Routes
% r2 = ['r1-cz', 'r1-cy', 'r1-c0', 'r1-cb', 'r1-ca', 'WayPoint67', 'r1.5-ca', 'WayPoint144'];
robot_02 = array2table([
 ["3",       "r1-cb",      "r1-cb",           "r1-cb_r1-ca", 0, 0]
 ["8",            "",      "r1-cb",           "r1-cb_r1-ca", 0, 0]
 ["16",           "",      "r1-ca",           "r1-cb_r1-ca", 0, 0]
 ["24",      "r1-ca",      "r1-ca",           "r1-cb_r1-ca", 0, 0]
 ["38",      "r1-ca",      "r1-ca",                      "", 0, 0]
 ["46",      "r1-ca",      "r1-ca",      "r1-ca_WayPoint67", 0, 0]
 ["48",           "",      "r1-ca",      "r1-ca_WayPoint67", 0, 0]
 ["50",           "", "WayPoint67",      "r1-ca_WayPoint67", 0, 0]
 ["55", "WayPoint67", "WayPoint67",      "r1-ca_WayPoint67", 0, 0]
 ["67", "WayPoint67", "WayPoint67",                      "", 0, 0]
 ["79", "WayPoint67", "WayPoint67", "WayPoint67_WayPoint73", 0, 0]
 ["83",           "", "WayPoint67", "WayPoint67_WayPoint73", 0, 0]
 ["87",           "", "WayPoint73", "WayPoint67_WayPoint73", 0, 0]
 ["96", "WayPoint73", "WayPoint73", "WayPoint67_WayPoint73", 0, 0]
], "VariableNames",["time", "CurrentNode", "ClosestNode", "ClosestEdge", "x", "y"]);
plot_robot_path(robot_02, N, E, timesteps, [0,1,0], [0,.75,0])


% Pretty up the axis
axis off
ax2 = copyobj(ax1,gcf);
ax3 = copyobj(ax1,gcf);

% Set the view angle
view(ax1, [-133 12]);
view(ax2, [-64 11]);
view(ax3, [0 90]);

ax1.Position = [0.0 0.0 0.4 1.0];
ax2.Position = [0.4 0.0 0.4 1.0];
ax3.Position = [0.8 0.0 0.2 1.0];

saveas(gcf, 'test.pdf')



function plot_robot_path(robot_log, N, E, timesteps, colourA, colourB)
    
    % Display plot of the robots path over time
    for i = 1:size(robot_log,1)
        d = robot_log(i,:);
        FurthestNode = split(d.ClosestEdge, '_');
        FurthestNode(FurthestNode == d.ClosestNode) = [];
        if d.CurrentNode == ""
            closest = N(string(N.node_name) == d.ClosestNode, :);
            furthest = N(string(N.node_name) == FurthestNode, :);
            robot_log(i, 'x') = {closest.x + ((furthest.x-closest.x)/3)};
            robot_log(i, 'y') = {closest.y + ((furthest.y-closest.y)/3)};
        else
            current = N(string(N.node_name) == d.CurrentNode, :);
            robot_log(i, 'x') = {current.x};
            robot_log(i, 'y') = {current.y};
        end
    end
    plot3(double(robot_log.x), double(robot_log.y), double(robot_log.time), 'LineWidth', 2, 'Color',colourA)
    %scatter3(double(robot_log.x), double(robot_log.y), double(robot_log.time), 'filled', 'MarkerEdgeColor',colour, 'MarkerFaceColor',colourA)
    
    % Highlight active edges/nodes
    for i = timesteps
        t = double(robot_log.time) >= i;
        t(end) = 1;
        d = robot_log(find(t, 1),:);
        if d.CurrentNode == ""
            e = E(string(E.edge_name) == d.ClosestEdge, :);
            plot3([e.start_x, e.end_x]', [e.start_y, e.end_y]', [i, i], 'Color', colourB, 'LineWidth', 2);
        else
            n = N(string(N.node_name) == d.CurrentNode, :);
            scatter3(double(n.x), double(n.y), i, 'filled', 'MarkerEdgeColor',colourB, 'MarkerFaceColor',colourB)
        end
    end

end





