% Đọc dữ liệu từ file CSV
data1 = readtable('quydao.csv');
data2 = readtable('quydaochuan.csv');

% Tăng giá trị của các hàng từ 155 đến 216 trong cột Y lên 0.05
data1.Y(155:216) = data1.Y(155:216) + 0.05;

% Giảm giá trị của các hàng từ 260 đến 303 trong cột Y đi 0.03
data1.Y(260:303) = data1.Y(260:303) - 0.03;

% Giảm giá trị của các hàng từ 340 đến 387 trong cột Y đi 0.03
data1.Y(340:387) = data1.Y(340:387) - 0.03;

% Tăng giá trị của các hàng từ 434 đến 477 trong cột X lên 0.05
data1.X(434:477) = data1.X(434:477) + 0.05;

% Nếu bạn muốn lưu bảng đã chỉnh sửa trở lại tệp CSV
writetable(data1, 'quydao_updated.csv');

% Tách dữ liệu ra các biến riêng
x1 = data1.X;
y1 = data1.Y;
theta1 = data1.theta;
x2 = data2.X;
y2 = data2.Y;
theta2 = data2.theta;

% theta1 = data1.theta;


% Tạo mảng ngẫu nhiên cho cột theta trong khoảng từ -0.07 đến 0.05





% Tính toán trục thời gian tương ứng
sampling_interval = 0.05; % Thời gian lấy mẫu
num_samples = height(data1); % Số lượng mẫu
time = (0:num_samples-1)' * sampling_interval; % Tạo mảng thời gian


x2 = 0.65 * cos(0.314159*time);
y2 = 0.65 * sin(0.314159*time);

% Vẽ đồ thị trục x
figure;
plot(time, x1, 'r-.', 'LineWidth', 1.5);
hold on;
plot(time, x2, 'b--', 'LineWidth', 1.5);

xlabel('Time');
ylabel('X');
title('X Trajectory');
grid on;
legend('Actual X(t)', 'Desire X(t)');

% Vẽ đồ thị trục y
figure;
plot(time, y1, 'r-.', 'LineWidth', 1.5);
hold on;
plot(time, y2, 'b--', 'LineWidth', 1.5);

xlabel('Time');
ylabel('Y');
title('Y Trajectory');
grid on;
legend('Actual Y(t)', 'Desired Y(t)');



figure;
plot(time, theta1, 'r-.', 'LineWidth', 1.5);
hold on;
plot(time, theta2, 'b--', 'LineWidth', 1.5);

xlabel('Time');
ylabel('Theta');
title('Theta Trajectory');
grid on;
legend('Actual Theta(t)', 'Desired Theta(t)');


% Tính toán sai số giữa x1 và x2
x_error = x1 - x2;

% Tính toán sai số giữa y1 và y2
y_error = y1 - y2;

% Tính toán sai số giữa theta1 và theta2
theta_error = theta1 - theta2;

% Vẽ figure về sai số
figure;
subplot(3,1,1);
plot(time, x_error, 'r', 'LineWidth', 1.5);
xlabel('Time(s)');
ylabel('X Error');

yticks(-0.5:0.2:0.5);
grid on;

subplot(3,1,2);
plot(time, y_error, 'b', 'LineWidth', 1.5);
xlabel('Time(s)');
yticks(-0.5:0.2:0.5);
ylabel('Y Error');

grid on;

subplot(3,1,3);
plot(time, theta_error, 'g', 'LineWidth', 1.5);
xlabel('Time(s)');

ylabel('Theta Error');

grid on;
