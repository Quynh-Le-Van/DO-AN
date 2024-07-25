% Đọc dữ liệu từ file CSV
data1 = readtable('quydao.csv');

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
% Vẽ đồ thị
figure;

% Plot quỹ đạo từ file CSV đầu tiên
plot(x1, y1, 'r-.', 'LineWidth', 1.5);
hold on;

% Thiết lập nhãn và tiêu đề
xlabel('X');
ylabel('Y');
title('Comparison of Circular Trajectories');
grid on;

% Vẽ đường tròn bán kính 0.65m
theta = linspace(0, 2*pi, 100);
x_circle = 0.65 * cos(theta);
y_circle = 0.65 * sin(theta);
plot(x_circle, y_circle, 'b-.', 'LineWidth', 1.5);

% Thêm điểm bắt đầu (0, 0) và điểm kết thúc (0.09, 0.65)
plot(0, 0, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'g'); % Điểm bắt đầu màu xanh lá cây
text(0, 0, 'Start', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'g');
plot(0.09, 0.65, 'mo', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'm'); % Điểm kết thúc màu tím
text(0.09, 0.65, 'End', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'Color', 'm');

% Thêm legend
legend('Actual Trajectory', 'Desired Trajectory', 'Start Point', 'End Point');

% Giữ đồ thị
hold off;




















