cla;

% Độ dài các khớp
l1 = 0; % Chiều dài khớp 1
l2 = 0.25; % Chiều dài khớp 2
l3 = 0.25; % Chiều dài khớp 3

% Góc của các khớp
theta1 = 0; % Góc khớp 1
theta2 = pi/2; % Góc khớp 2
theta3 = -pi/2; % Góc khớp 3

% Tọa độ các điểm trong không gian
x0 = x; % Tọa độ x của điểm gốc
y0 = y; % Tọa độ y của điểm gốc
z0 = z; % Tọa độ z của điểm gốc

% Tính toán tọa độ các điểm trong không gian
x1 = 0;
y1 = 0;
z1 = z0;

x2 = x0 + cos(theta1)*l2*cos(theta2);
y2 = y0 + sin(theta1)*l2*cos(theta2);
z2 = z0 + l1 + l2*sin(theta2);

x3 = x2 + cos(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2));
y3 = y2 + sin(theta1)*(l3*cos(theta2 + theta3) + l2*cos(theta2));
z3 = z0 + l1 + l3*sin(theta2 + theta3) + l2*sin(theta2);

% Vẽ cánh tay robot
% figure;
% hold on;

% Vẽ đường nối giữa các điểm
plot3([x0, x2, x3], [y0, y2, y3], [z0, z2, z3], 'b-o', 'LineWidth', 2);

% Vẽ các điểm trên cánh tay robot
plot3(x1, y1, z1, 'bo', 'MarkerSize', 5);
plot3(x2, y2, z2, 'bo', 'MarkerSize', 5);
plot3(x3, y3, z3, 'bo', 'MarkerSize', 5);

% Cấu hình trục và khung nhìn
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

% Đặt giới hạn trục
axisSize = 2;
xlim([-axisSize, axisSize]);
ylim([-axisSize, axisSize]);
zlim([-axisSize, axisSize]);

% Hiển thị lưới
grid on;
