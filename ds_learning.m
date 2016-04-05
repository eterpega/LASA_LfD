
function ds_learning()
close all
% set up a simple robot and a figure that plots it
robot = create_simple_robot();
fig = initialize_robot_figure(robot);
%fig = figure(1);clf;
%robot.plot([0,1])
disp('In this exercise you will peroform very simple record and replay of a demonstrated trajectory.')
disp('You can see the robot in the figure. Give a demonstration of a trajectory in its workspace')
% get a demonstration
data = get_demonstration(fig);
disp('Its convenient to represent the collected data with interpolating polynomials, e.g. splines. This allows limit the complexity as well as guarantee smooth trajectories.')
nb_knots = 10;
disp(sprintf('you now see a spline with %d knots in the figure.', nb_knots));
% fit a spline to the demonstration
% nb_data = size(data,2);
% skip = floor(nb_data/nb_knots);
% knots = data(:,1:skip:end);
% ppx = spline(knots(3,:),knots(1:2,:));
% % and get first and second derivative
% ppxd = differentiate_spline(ppx);
% 
% % lets plot the demonstrated trajectory
% plot(knots(1,:), knots(2,:), 'b+');
% Data = [ppval(ppx,linspace(knots(3,1),knots(3,end),1000)); ppval(ppxd, linspace(knots(3,1),knots(3,end),1000))];

% A set of options that will be passed to the solver. Please type 
% 'doc preprocess_demos' in the MATLAB command window to get detailed
% information about other possible options.
options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                              % instabilities in Gaussian kernel [default: 10^-15]
                              
options.display = 1;          % An option to control whether the algorithm
                              % displays the output of each iterations [default: true]
                              
options.tol_stopping=10^-10;  % A small positive scalar defining the stoppping
                              % tolerance for the optimization solver [default: 10^-10]

options.max_iter = 500;       % Maximum number of iteration for the solver [default: i_max=1000]

options.objective = 'mse';    % 'likelihood': use likelihood as criterion to
                              % optimize parameters of GMM
                              % 'mse': use mean square error as criterion to
                              % optimize parameters of GMM
                              % 'direction': minimize the angle between the
                              % estimations and demonstrations (the velocity part)
                              % to optimize parameters of GMM                              
                              % [default: 'mse']
[x0 , xT, Data, index] = preprocess_demos({data(1:2,:)},{data(3,:)},1e-3); %preprocessing datas                              
[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,1); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solver

target = data(1:2,end);
ds = @(x) GMR(Priors,Mu,Sigma,x-repmat(target, 1, size(x,2)),1:2,3:4);
plot_ds_model(fig, ds);
% find an initial joint configuration for the start point
%qi = robot.ikine(transl(knots(1,1), knots(2,1),0.0),[0.2,0.2],[1,1,0,0,0,0]);
qi = simple_robot_ikin(data(1:2,1));
robot.animate(qi);
% simulate tracking of the trajectory in the absence of perturbations
% We will use a cartesian impedance controller to track the motion
% we need to define stiffness and damping values for our controller
K = eye(2)*00;
D = eye(2)*8;
% start simulation
dt = 0.005;
% simulation from same start point
disp('The robot will be able to perform the task when starting from the same starting location as the demonstration.')
disp('press enter to continure..')
pause
simulation(qi);
% simulation from different starting point
while 1
    disp('Now we imagine the robot starts the task from a different location. click on a departure point in the robot workspace.')
    try
        xs = get_point(fig);
        qs = simple_robot_ikin(xs);
        robot.animate(qs)
        disp('The simple time-dependent reference trajectory approach cannot deal with this situation. Press enter to see what happens..')
        pause
        simulation(qs);
    catch
        disp('could not find joint space configuration. Please choose another point in the workspace.')
    end
end
%simulation(qi,1);
    function simulation(q)
        t = data(3,1);
        qd = [0,0];
        x_ref = robot.fkine(q);
        x_ref = x_ref(1:2,4);
        h = plot(x_ref(1),x_ref(2),'go');
        ht = [];
        while(1)
            % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:2,4);
            xd = robot.jacob0(q)*qd';
            xd = xd(1:2);
            %eig(cart_inertia(robot, q))

            % compute our time-dependent refernce trajectory
            %x_ref = x_ref+dt*ds(x_ref);%reference_pos(t);
            xd_ref = ds(x_ref);%reference_vel(t);
            x_ref = x_ref+dt*xd_ref;
            %xdd_ref = ppval(ppxdd, t);%reference_acc(t);
            xdd_ref = -0.00002*(xd-xd_ref);
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
            if (norm(x - data(1:2,end))<0.1)
                break
            end
            robot.delay = dt;
            robot.animate(q);
            set(h,'XData',x_ref(1));
            set(h,'YData',x_ref(2));
            ht = [ht, plot(x_ref(1), x_ref(2), 'm.')];
        end
        delete(h);
        delete(ht);
    end

    function qs = get_other_start()
        cs = knots(1:2,1)+0.3*[0;1];
        %         lo_lim = 0; hi_lim = 1.5;
        %         cs(1) = max(cs(1),lo_lim);
        %         cs(2) = max(cs(2),lo_lim);
        %         cs(1) = min(cs(1), hi_lim);
        %         cs(2) = min(cs(2), hi_lim);
        qs = robot.ikine(transl(cs(1), cs(2),0.0),qi,[1,1,0,0,0,0])
    end

    function q = simple_robot_ikin(x)
        l1 = robot.a(1); l2 = robot.a(2);
        q2 = atan2(sqrt(1-(x'*x-l1.^2-l2.^2).^2/(2*l1*l2).^2), (x'*x-l1.^2-l2.^2)/(2*l1*l2))
        q1 = atan2(x(2), x(1)) - atan2(l2*sin(q2), l1+l2*cos(q2));
        q = [q1, q2];
    end

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