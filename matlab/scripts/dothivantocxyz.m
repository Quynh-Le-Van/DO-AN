% Đọc dữ liệu từ file CSV
data = readtable('vantocxyz.csv');

% Tạo trục thời gian với thời gian lấy mẫu 0.05s
sampling_interval = 0.05; % Thời gian lấy mẫu
num_samples = min(height(data), 25 / sampling_interval); % Số lượng mẫu (tối đa 25 giây)
time = (0:num_samples-1)' * sampling_interval; % Tạo mảng thời gian

% Trích xuất các cột dữ liệu
velocity_x = data{1:num_samples, 'x'};
velocity_y = data{1:num_samples, 'y'};
angular_velocity = data{1:num_samples, 'theta'};

% Vẽ đồ thị cho vận tốc theo trục X
subplot(4, 1, 1);
plot(time, velocity_x, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('Vx(m/s)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks(-0.3:0.1:0.3);

% Vẽ đồ thị cho vận tốc theo trục Y
subplot(4, 1, 2);
plot(time, velocity_y, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('Vy(m/s)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks(-0.3:0.1:0.3);

% Vẽ đồ thị cho tốc độ góc
subplot(4, 1, 3);
plot(time, angular_velocity, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('Vtheta(rad/s)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks(-0.3:0.1:0.3);

% Vẽ đồ thị trống ở cột cuối cùng
subplot(4, 1, 4);
axis off;

% Định dạng hiển thị thời gian
time_format = 'HH:MM:SS'; % Định dạng giờ:phút:giây
time_ticks = xticks;
time_labels = datestr(time_ticks * sampling_interval, time_format);
set(gca, 'XTickLabel', time_labels);