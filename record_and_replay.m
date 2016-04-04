
function record_and_replay()
close all
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
ppx = spline(knots(3,:),knots(1:2,:));
% and get first and second derivative
ppxd = differentiate_spline(ppx);
ppxdd = differentiate_spline(ppxd);

% lets plot the reference trajectory
plot(knots(1,:), knots(2,:), 'b+');
ref_traj = ppval(ppx,data(3,:));
plot(ref_traj(1,:),ref_traj(2,:),'k-');

% find an initial joint configuration for the start point
qi = robot.ikine(transl(data(1,1), data(2,1),0.0),[0.2,0.2],[1,1,0,0,0,0]);
robot.animate(qi);

% simulate tracking of the trajectory in the absence of perturbations
% We will use a cartesian impedance controller to track the motion
% we need to define stiffness and damping values for our controller
K = eye(2)*400;
D = eye(2)*7;
% start simulation
dt = 0.005;
% simulation from same start point
simulation(qi);
% simulation from different starting point 
%simulation(qi,1);
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
            x_ref = ppval(ppx, t);%reference_pos(t);
            xd_ref = ppval(ppxd, t);%reference_vel(t);
            xdd_ref = ppval(ppxdd, t);%reference_acc(t);
            xdd_ref = xdd_ref-0.2*(xd-xd_ref);
            % compute cartesian control
            u_cart = -K*(x-x_ref) - D*(xd-xd_ref);
            % feedforward term
            u_cart = u_cart + cart_inertia(robot,q)*xdd_ref + robot.coriolis(q,qd)*qd';
            
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
        end
        %close(fig)
    end


function pp_out = differentiate_spline(pp_in)
% extract details from piece-wise polynomial by breaking it apart
[breaks,coefs,l,k,d] = unmkpp(pp_in);
% make the polynomial that describes the derivative
pp_out = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
end

function mx = cart_inertia(robot, q)
mq = robot.inertia(q);
J = robot.jacob0(q);
J = J(1:2,1:2);
Ji = inv(J);
mx = Ji'*mq*Ji;
end

end
