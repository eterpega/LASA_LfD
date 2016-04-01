function fig = initialize_robot_figure(robot)

fig = figure();
robot.plot([0,0])
view([0 90])
axis([-0.5 2 -0.5 2])

end
