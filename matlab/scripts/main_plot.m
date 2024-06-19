%% Init 
plot_Init

%% Simulate Loop
for t = 1 : 10 : size(TrajectoryRef_data.ans, 2)

  x = MobilePosAc_data.ans(2, t);
  y = MobilePosAc_data.ans(3, t);
  theta = MobilePosAc_data.ans(4, t);
  z = 0.2;

  theta1 = ArmAngle_data.ans(2, t);
  theta2 = ArmAngle_data.ans(3, t);
  theta3 = ArmAngle_data.ans(4, t);

  hx = TrajectoryAc_data.ans(2, t);
  hy = TrajectoryAc_data.ans(3, t);
  hz = TrajectoryAc_data.ans(4, t);

  plot_MobileManipulator

  pause(0.001);
  hold on

end
