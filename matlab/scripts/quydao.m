dataXYTrajectory = load('D:\Documents\DOAN\DO-AN\matlab\datamat\quydao\quydao.mat');
dataVelocity = load('D:\Documents\DOAN\DO-AN\matlab\datamat\quydao\vantoc.mat');
dataError = load('D:\Documents\DOAN\DO-AN\matlab\datamat\quydao\saiso.mat');
dataWheel = load ('D:\Documents\DOAN\DO-AN\matlab\datamat\quydao\banhxe.mat');

% =========================Plot data trajectory figure =============================
figure;

% Plot data trajectory of mobile platform
Trajectory_X = dataXYTrajectory.ans(2,:);
Trajectory_Y = dataXYTrajectory.ans(3,:);
Trajectory_desire_X = dataXYTrajectory.ans(5,:);
Trajectory_desire_Y = dataXYTrajectory.ans(6,:);
Trajectory_t = dataXYTrajectory.ans(1,:);

% Plot x(t), y(t), and theta(t) on the same figure
subplot(2, 2, 1);
plot(Trajectory_t, Trajectory_X, 'r-.', 'LineWidth', 1.5);
hold on
plot(Trajectory_t, Trajectory_desire_X, 'g-.', 'LineWidth', 1.5);
title('X axis Trajectory Plot');
xlabel('Time(s)');
ylabel('X (m)');
legend('Actual X axis Trajectory', 'Desired X axis Trajectory');
grid on

subplot(2, 2, 2);
plot(Trajectory_t, Trajectory_Y, 'r-.', 'LineWidth', 1.5);
hold on
plot(Trajectory_t, Trajectory_desire_Y, 'g-.', 'LineWidth', 1.5);
title('Y axis Trajectory Plot');
xlabel('Time(s)');
ylabel('Y (m)');
legend('Actual Y axis Trajectory', 'Desired Y axis Trajectory');
grid on

subplot(2, 2, 3);
Trajectory_theta = dataXYTrajectory.ans(4,:);
Trajectory_theta_desired = dataXYTrajectory.ans(7,:);
plot(Trajectory_t, Trajectory_theta, 'r-.', 'LineWidth', 1.5);
hold on
plot(Trajectory_t, Trajectory_theta_desired, 'g-.', 'LineWidth', 1.5);
title('Theta axis Trajectory Plot');
xlabel('Time(s)');
ylabel('Theta (rad)');
legend('Actual Theta axis Trajectory', 'Desired Theta axis Trajectory');
grid on

% Plot Vx(t), Vy(t), and Vtheta(t) on the same figure
figure;

Velocity_X = dataVelocity.ans(2,:);
Velocity_Y = dataVelocity.ans(3,:);
Velocity_theta = dataVelocity.ans(4,:);
Velocity_t = dataVelocity.ans(1,:);

subplot(2, 2, 1);
plot(Velocity_t, Velocity_X, 'r-.', 'LineWidth', 1.5);
title('X axis Velocity Plot');
xlabel('Time(s)');
ylabel('X (m/s)');
legend('Actual X axis Velocity');
grid on

subplot(2, 2, 2);
plot(Velocity_t, Velocity_Y, 'r-.', 'LineWidth', 1.5);
title('Y axis Velocity Plot');
xlabel('Time(s)');
ylabel('Y (m/s)');
legend('Actual Y axis Velocity');
grid on

subplot(2, 2, 3);
plot(Velocity_t, Velocity_theta, 'r-.', 'LineWidth', 1.5);
title('Theta axis Velocity Plot');
xlabel('Time(s)');
ylabel('Theta (rad/s)');
legend('Actual Theta axis Velocity');
grid on

% Plot Ex(t), Ey(t), and Etheta(t) on the same figure
figure;

Error_X = dataError.ans(2,:);
Error_Y = dataError.ans(3,:);
Error_theta = dataError.ans(4,:);
Error_t = dataError.ans(1,:);

subplot(2, 2, 1);
plot(Error_t, Error_X, 'r-.', 'LineWidth', 1.5);
title('X axis Error Plot');
xlabel('Time(s)');
ylabel('Error X');
legend('Error in X axis');
grid on

subplot(2, 2, 2);
Wheel_W1 = dataWheel.ans(2,:);
Wheel_W2 = dataWheel.ans(3,:);
Wheel_W3 = dataWheel.ans(4,:);
Wheel_W4 = dataWheel.ans(5,:);
Wheel_t = dataWheel.ans(1,:);

plot(Wheel_t, Wheel_W1, 'r-.', 'LineWidth', 1.5);
hold on
plot(Wheel_t, Wheel_W2, 'g-.', 'LineWidth', 1.5);
hold on
plot(Wheel_t, Wheel_W3, 'b-.', 'LineWidth', 1.5);
hold on
plot(Wheel_t, Wheel_W4, 'm-.', 'LineWidth', 1.5);
title('Wheel Speed Plot');
xlabel('Time(s)');
ylabel('Wheel Speed');
legend('Wheel 1 Speed', 'Wheel 2 Speed', 'Wheel 3 Speed', 'Wheel 4 Speed');
grid on

% Adjust the subplot spacing
subplotSpacing = 0.03;
subplotPosition = get(gcf, 'Position');
subplotPosition(3) = subplotPosition(3) + subplotSpacing;
subplotPosition(4) = subplotPosition(4) + subplotSpacing;
set(gcf, 'Position', subplotPosition);
set(gcf, 'Resize', 'off');

% Save the figure
saveas(gcf, 'trajectory_plots.png');