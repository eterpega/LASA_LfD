
function record_and_replay()
% set up a simple robot and a figure that plots it
robot = create_simple_robot();
fig = initialize_robot_figure(robot);
%fig = figure(1);clf;
%robot.plot([0,1])

% get a demonstration
data = get_demonstration(fig);
% fit a spline to the demonstration
nb_knots = 20;
nb_data = size(data,2);
skip = floor(nb_data/nb_knots);
knots = data(:,1:skip:end);
% function for querying our spline per dimension
spl = @(ind, t) spline(knots(3,:),knots(ind,:),t);
% lets plot that 
plot(knots(1,:), knots(2,:), 'b+');
plot(spl(1,data(3,:)), spl(2, data(3,:)),'k-');

qi = robot.ikine(transl(data(1,1), data(2,1),0.0),[0,0.2],[1,1,0,0,0,0]);
robot.animate(qi)

end


