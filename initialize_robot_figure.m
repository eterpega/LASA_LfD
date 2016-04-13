function fig = initialize_robot_figure(robot)

fig = figure();
robot.plot([0,0])
view([0 90])
hold on
% plot workspace limitation
rad = sum(robot.a);
x_data = rad*sin(0:0.01:2*pi);
y_data = rad*cos(0:0.01:2*pi);
plot(x_data, y_data, 'k--','linewidth',4);
axis([-0.5 1.5 -0.5 1.5])

end
