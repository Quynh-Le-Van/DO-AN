% cla;
delete(mobilePF);
delete(Robot_arm_1);
delete(Robot_arm_2);
delete(p1);
delete(p2);
delete(p3);
delete(arrow);

%% Khai báo 
% Kích thước của hình chữ nhật
length = 0.3;  % Chiều dài
width = 0.3;   % Chiều rộng
height = 0;  % Chiều cao

% Độ dài các khớp
l1 = 0; % Chiều dài khớp 1
l2 = 0.25; % Chiều dài khớp 2
l3 = 0.25; % Chiều dài khớp 3

% Góc của các khớp
% theta1 = 0; % Góc khớp 1
% theta2 = pi/6; % Góc khớp 2 
% theta3 = -pi/6; % Góc khớp 3

%% Vẽ mobile platform
xm = x;  % Tọa độ xm
ym = y;  % Tọa độ ym
zm = z;  % Tọa độ zm

% Tạo ma trận biểu diễn hình chữ nhật
rectangle = [-(length/2), -(width/2), 0; (length/2), -(width/2), 0; (length/2), (width/2), 0; -(length/2), (width/2), 0; ...
             -(length/2), -(width/2), height; (length/2), -(width/2), height; (length/2), (width/2), height; -(length/2), (width/2), height];

% Xoay hình chữ nhật quanh trục zm
R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
rotated_rectangle = (R * rectangle')';

% Dịch chuyển hình chữ nhật đến vị trí (xm, ym, zm)
translated_rectangle = rotated_rectangle + repmat([xm, ym, zm], size(rotated_rectangle, 1), 1);
mobilePF = patch(translated_rectangle(:, 1), translated_rectangle(:, 2), translated_rectangle(:, 3), 'b');

% Vẽ mũi tên chỉ hướng
% Tính toán tọa độ đầu mũi tên
arrowLength = 0.3; % Độ dài mũi tên
arrowX = x + arrowLength*cos(theta); % Tọa độ x của đầu mũi tên
arrowY = y + arrowLength*sin(theta); % Tọa độ y của đầu mũi tên
arrowZ = z; % Tọa độ z của đầu mũi tên (trong mặt phẳng Oxy)

arrow = quiver3(x, y, z, arrowX-x, arrowY-y, arrowZ-z, 'r', 'LineWidth', 1.5);

%% Vẽ cánh tay
% Tọa độ các điểm trong không gian
x0 = x; % Tọa độ x của điểm gốc
y0 = y; % Tọa độ y của điểm gốc
z0 = z; % Tọa độ z của điểm gốc

x1 = x0;
y1 = y0;
z1 = z0;

x2 = x0 + cos(theta1 + theta)*l2*cos(theta2);
y2 = y0 + sin(theta1 + theta)*l2*cos(theta2);
z2 = z0 + l1 + l2*sin(theta2);

x3 = x0 + l2*cos(theta2) * cos(theta + theta1) + l3 * cos(theta2 + theta3) * cos(theta + theta1); 
y3 = y0 + l2*cos(theta2) * sin(theta + theta1) + l3*cos(theta2 + theta3) * sin(theta + theta1);
z3 = z0 + l1 + l2 * sin(theta2) + l3*sin(theta2 + theta3);  

% % Vẽ đường nối giữa các điểm
% plot3([x0, x2, x3], [y0, y2, y3], [z0, z2, z3], 'r-o', 'LineWidth', 2);

% % Vẽ các điểm trên cánh tay robot
p1 = plot3(x1, y1, z1, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'red');
p2 = plot3(x2, y2, z2, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'red');
p3 = plot3(x3, y3, z3, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'red');

% Vẽ khớp 2
Robot_arm_1 = patch([x1, x2], [y1, y2], [z1, z2], 'g', 'LineWidth', 2);

% Vẽ khớp 3
Robot_arm_2 = patch([x2, x3], [y2, y3], [z2, z3], 'b', 'LineWidth', 2);

% Vẽ quỹ đạo thực tế của end-effects 
pl = plot3(x3, y3, z3, 'k.');
pl.LineWidth = 2;

%% Config đồ thị
% Tăng kích thước các trục
axisSize = 2;
xlim([-axisSize, axisSize]);
ylim([-axisSize, axisSize]);
zlim([0, axisSize-1]);

% Vẽ lưới
grid on;

% axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

hold on;