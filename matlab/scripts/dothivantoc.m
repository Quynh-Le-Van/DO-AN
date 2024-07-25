% Đọc dữ liệu từ file CSV
data = readtable('vantocbanhxe.csv');

% Tạo trục thời gian với thời gian lấy mẫu 0,05s
sampling_interval = 0.05; % Thời gian lấy mẫu
num_samples = min(height(data), 25 / sampling_interval); % Số lượng mẫu (tối đa 25 giây)
time = (0:num_samples-1)' * sampling_interval; % Tạo mảng thời gian

% Trích xuất các cột dữ liệu
value1 = data{1:num_samples, 1};
value2 = data{1:num_samples, 2};
value3 = data{1:num_samples, 3};
value4 = data{1:num_samples, 4};

% Vẽ đồ thị cho vận tốc bánh xe 1
subplot(4, 1, 1);
plot(time, value1, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('W1(RPM)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks([-100, -50, 0, 50, 100]);

% Vẽ đồ thị cho vận tốc bánh xe 2
subplot(4, 1, 2);
plot(time, value2, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('W2(RPM)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks([-100, -50, 0, 50, 100]);

% Vẽ đồ thị cho vận tốc bánh xe 3
subplot(4, 1, 3);
plot(time, value3, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('W3(RPM)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks([-100, -50, 0, 50, 100]);

% Vẽ đồ thị cho vận tốc bánh xe 4
subplot(4, 1, 4);
plot(time, value4, 'linewidth', 1.3);
grid on;
xlabel('Time(s)');
ylabel('W4(RPM)');
xticks(0:5:time(end)); % Khoảng chia 5 giây
yticks([-100, -50, 0, 50, 100]);