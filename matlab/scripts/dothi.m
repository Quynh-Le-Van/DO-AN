% Load data from file
dataXYTrajectory = load('D:\Documents\DOAN\DO-AN\matlab\datamat\dothi\vitri.mat');
dataVelocity = load('D:\Documents\cdt\sim\DataMat\DataVelocityofMobilePlatform.mat');
dataError = load('D:\Documents\cdt\sim\DataMat\DataErrorofMobilePlatform.mat');
dataWheel = load ('D:\Documents\DOAN\DO-AN\matlab\datamat\dothi\banhxe.mat')
dataXZTrajectory = load('D:\Documents\DOAN\DO-AN\matlab\datamat\dothi\canhtay.mat');


xy = load('D:\Documents\DOAN\DO-AN\matlab\datamat\tmp\vitri.mat');

% =========================Plot data trajectory figure ============================= 
figure;

% Plot data tranjectory of mobile platform
Trajectory_X = dataXYTrajectory.ans(2,:);
Trajectory_Y = dataXYTrajectory.ans(3,:);
plot(Trajectory_X, Trajectory_Y,'b-.', 'LineWidth', 1.5);
hold on


% Config plot feature
title('Trajectory Plot');
xlabel('X (m)');
ylabel('Y (m)');
legend ('Desire Trajectory')
grid on

%================== Plot x(t) y(t) data =============
figure;

% Plot x(t)
Trajectory_t = dataXYTrajectory.ans(1,:);
Trajectory_X = dataXYTrajectory.ans(2,:);
plot(Trajectory_t, Trajectory_X,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('X axis Trajectory Plot');
xlabel('Time(s)');
ylabel('X (m)');
legend ('Actual X axis Trajectory','Desired X axis Trajectory')
grid on


% Plot y(t)
figure;

Trajectory_t = dataXYTrajectory.ans(1,:);
Trajectory_Y = dataXYTrajectory.ans(3,:);

plot(Trajectory_t, Trajectory_Y,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Y axis Trajectory Plot');
xlabel('Time(s)');
ylabel('Y (m)');
legend ('Actual Y axis Trajectory','Desired Y axis Trajectory')
grid on

%================== Plot Vx(t) Vy(t) data =============
% Plot Vx(t)
figure;

Velocity_t = dataVelocity.ans(1,:);
Velocity_X = dataVelocity.ans(2,:);
plot(Velocity_t, Velocity_X,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('X axis Velocity Plot');
xlabel('Time(s)');
ylabel('X (m/s)');
legend ('Actual X axis Velocity')
grid on

% Plot Vy(t)
figure;

Velocity_t = dataVelocity.ans(1,:);
Velocity_Y = dataVelocity.ans(3,:);
plot(Velocity_t, Velocity_Y,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Y axis Velocity Plot');
xlabel('Time(s)');
ylabel('Y (m/s)');
legend ('Actual Y axis Velocity')
grid on

%================== Plot Ex(t) Ey(t) data =============
% Plot Ex(t)
figure;

Error_t = dataError.ans(1,:);
Error_X = dataError.ans(2,:);
plot(Error_t, Error_X,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('X axis Error Plot');
xlabel('Time(s)');
ylabel('Ex');
legend ('Error in X axis')
grid on

% Plot Ey(t)
figure;

Error_t = dataError.ans(1,:);
Error_Y = dataError.ans(3,:);
plot(Error_t, Error_Y,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Y axis Error Plot');
xlabel('Time(s)');
ylabel('Ey');
legend ('Error in Y axis')
grid on

%================== Plot Angular velocity of wheel data =============
% Plot W1
figure;

AnVelovity_t = dataWheel.ans(1,:);
W1 = dataWheel.ans(2,:);
plot(AnVelovity_t, W1,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Angular velocity of Wheel 1 Plot');
xlabel('Time(s)');
ylabel('W1 (rad/s)');
legend ('Angular Velocity')
grid on

% Plot W2
figure;

AnVelovity_t = dataWheel.ans(1,:);
W2 = dataWheel.ans(3,:);
plot(AnVelovity_t, W2,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Angular velocity of Wheel 2 Plot');
xlabel('Time(s)');
ylabel('W2 (rad/s)');
legend ('Angular Velocity')
grid on

% Plot W3
figure;

AnVelovity_t = dataWheel.ans(1,:);
W3 = dataWheel.ans(4,:);
plot(AnVelovity_t, W3,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Angular velocity of Wheel 3 Plot');
xlabel('Time(s)');
ylabel('W3 (rad/s)');
legend ('Angular Velocity')
grid on

% Plot W4
figure;

AnVelovity_t = dataWheel.ans(1,:);
W4 = dataWheel.ans(5,:);
plot(AnVelovity_t, W4,'b-.', 'LineWidth', 1.5);
hold on

% Config plot feature
title('Angular velocity of Wheel 4 Plot');
xlabel('Time(s)');
ylabel('W4 (rad/s)');
legend ('Angular Velocity')
grid on


% Plot data figure 
figure;

% Plot data tranjectory of manipulator 
t1 = dataXZTrajectory.ans(1,:);

mni_X = dataXZTrajectory.ans(2,:);

plot(t1, mni_X,'b-.', 'LineWidth', 1.5);
hold on

xlabel('Time(s)');
ylabel('X (m)');
legend ('X position')
grid on

% Plot data figure 
figure;

% Plot data tranjectory of manipulator 
t1 = dataXZTrajectory.ans(1,:);

mni_Y = dataXZTrajectory.ans(3,:);

plot(t1, mni_Y,'b-.', 'LineWidth', 1.5);
hold on

xlabel('Time(s)');
ylabel('Y (m)');
legend ('Y position')
grid on

% Plot data figure 
figure;

% Plot data tranjectory of manipulator 
t1 = dataXZTrajectory.ans(1,:);

mni_Z = dataXZTrajectory.ans(4,:);

plot(t1, mni_Z,'b-.', 'LineWidth', 1.5);
hold on

xlabel('Time(s)');
ylabel('Z (m)');
legend ('Z position')
grid on