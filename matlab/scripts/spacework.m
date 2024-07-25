% Define parameters
l1 = 0.05; % Height of the first joint
l2 = 0.225; % Length of the second link
l3 = 0.225; % Length of the third link
l4 = 0.1;

% Define mobile robot dimensions
d = 0.45; % Length of the mobile robot
w = 0.36; % Width of the mobile robot
h = 0.20; % Height of the mobile robot

% Define resolution for the joint angles
theta1_res = 5; % Resolution for theta1
theta2_res = 5; % Resolution for theta2
theta3_res = 5; % Resolution for theta3

% Create arrays for joint angles
theta1 = 90:theta1_res:270;
theta2 = 0:theta2_res:180;
theta3 = 0:theta3_res:180;

% Initialize arrays to store end effector positions
x_positions = [];
y_positions = [];
z_positions = [];

% Loop through all possible joint angles
for t1 = theta1
    for t2 = theta2
        for t3 = theta3
            % Convert angles to radians
            t1_rad = deg2rad(t1);
            t2_rad = deg2rad(t2);
            t3_rad = deg2rad(t3);
            
            % Calculate the position of the end effector
            px = 0.06 + cos(t1_rad) * (l3 * cos(t2_rad + t3_rad) + l2 * cos(t2_rad)) ;
            py = sin(t1_rad) * (l3 * cos(t2_rad + t3_rad) + l2 * cos(t2_rad));
            pz = h + l3 * sin(t2_rad + t3_rad) + l2 * sin(t2_rad);
            
            % Check if the position is within the mobile robot's dimensions
             if  (px) <= w/2 && (py) <= d/2 && pz <= h
                % Store positions
             elseif pz >=0
                x_positions = [x_positions, px];
                y_positions = [y_positions, py];
                z_positions = [z_positions, pz];              
             end
        end
    end
end

% Plot the workspace on the XY plane
figure;
scatter(x_positions, z_positions, '.');
title('Workspace of 3-DOF Robotic Arm on Mobile Robot (XY Plane)');
xlabel('X');
ylabel('Z');
grid on;
axis equal;
hold on;

% Plot mobile robot dimensions on XY plane
fill([-w/2, w/2, w/2, -w/2], [h, h, 0, 0], 'r', 'FaceAlpha', 0.3);

% Add legend
legend('Workspace', 'Mobile Robot');
