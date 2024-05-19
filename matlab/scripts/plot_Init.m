%% Load data
TrajectoryRef_data = load('D:\Documents\DOAN\DO-AN\matlab\datamat\data_trajectoroy_ref.mat');
TrajectoryAc_data = load('D:\Documents\DOAN\DO-AN\matlab\datamat\data_trajectory_actual.mat');
MobilePosAc_data = load('D:\Documents\DOAN\DO-AN\matlab\datamat\data_mobile_position.mat');
ArmAngle_data = load('D:\Documents\DOAN\DO-AN\matlab\datamat\data_arm_angle.mat');

%% Config figure
f = figure;
f.WindowState = 'maximized';

%% Init condition
mobilePF = patch(0, 0, 0, 'b');
Robot_arm_2 = patch([0, 0], [0, 0], [0, 0], 'LineWidth',0.1);
Robot_arm_1=patch([0, 0], [0, 0], [0, 0], 'LineWidth',0.1);
arrow = quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 2);
p1 = plot3(0, 0, 0, 'bo', 'MarkerSize', 0.01);
p2 = plot3(0, 0, 0, 'bo', 'MarkerSize', 0.01);
p3 = plot3(0, 0, 0, 'bo', 'MarkerSize', 0.01);
hold on

hx(1) = 0;
hy(1) = 0;
hz(1) = 0;

% Plot trajectory ref
x = 1 : 1 : size(TrajectoryRef_data.ans, 2)
pl = plot3(TrajectoryRef_data.ans(2, x), TrajectoryRef_data.ans(3, x), TrajectoryRef_data.ans(4, x), 'r-');
pl.LineWidth = 2;