
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
% get spline structures for our interpolation points
ppx1 = spline(knots(3,:),knots(1,:));
ppx2 = spline(knots(3,:),knots(2,:));
% get spline structures for first and second derivatives
ppx1_d = differentiate_spline(ppx1);
ppx2_d = differentiate_spline(ppx2);
ppx1_dd = differentiate_spline(ppx1_d);
ppx2_dd = differentiate_spline(ppx2_d);
% anonymous function for querying our spline per dimension
%spl = @(dim, t) spline(knots(3,:),knots(dim,:),t);
% convenvinece function for getting both coordinates togeather
reference_pos = @(t) [ppval(ppx1,t);ppval(ppx2,t)];
reference_vel = @(t) [ppval(ppx1_d,t);ppval(ppx2_d,t)];
reference_acc = @(t) [ppval(ppx1_dd,t);ppval(ppx2_dd,t)];
% lets plot that
plot(knots(1,:), knots(2,:), 'b+');
plot(ppval(ppx1,data(3,:)), ppval(ppx2, data(3,:)),'k-');
% find an initial joint configuration for the start point
qi = robot.ikine(transl(data(1,1), data(2,1),0.0),[0.2,0.2],[1,1,0,0,0,0]);
robot.animate(qi);

% lets simulate tracking of the trajectory in the absence of perturbations
% We will use a cartesian impedance controller to track the motion
% we need to define stiffness and damping values for our controller
K = eye(2)*500;
D = eye(2)*5;
% start simulation
dt = 0.005;
% simulation from same start point
simulation(qi);
% simulation from different starting point 
simulation(qi+0.5*randn(2,1)');
    function simulation(q)
        t = knots(3,1);
        qd = [0,0];
        while(1)
            % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:2,4);
            xd = robot.jacob0(q)*qd';
            xd = xd(1:2);
            % compute our time-dependent refernce trajectory
            x_ref = reference_pos(t);
            xd_ref = reference_vel(t);
            xdd_ref = reference_acc(t);
            % compute cartesian control
            u_cart = -K*(x-x_ref) - D*(xd-xd_ref);
            % compute joint space control
            u_joint = robot.jacob0(q)'*[u_cart;zeros(4,1)];
            % apply control to the robot
            qdd = robot.accel(q,qd,u_joint')';
            % integrate one time step
            qd = qd+dt*qdd;
            q = q+qd*dt+qdd/2*dt^2;
            t = t+dt;
            if (norm(x_ref - knots(1:2,end))<0.01)
                break
            end
            robot.delay = dt;
            robot.animate(q);
            %pause(dt);
        end
        %close(fig)
    end
% robot.accel()
% J = robot.jacob0(qi)
% robot.accel(positions, velocity, torque)
end

function pp_out = differentiate_spline(pp_in)
% extract details from piece-wise polynomial by breaking it apart
[breaks,coefs,l,k,d] = unmkpp(pp_in);
% make the polynomial that describes the derivative
pp_out = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
end

