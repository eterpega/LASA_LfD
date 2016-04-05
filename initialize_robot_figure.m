function fig = initialize_robot_figure(robot)

fig = figure();
robot.plot([0,0])
view([0 90])
axis([-0.5 1.5 -0.5 1.5])

end
