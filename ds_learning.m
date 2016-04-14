%hello
function ds_learning()
nb_gaussians = 2;
close all
robot = create_simple_robot();
fig = initialize_robot_figure(robot);
%fig = figure(1);clf;
%robot.plot([0,1])
disp('In this exercise you will peroform very simple record and replay of a demonstrated trajectory.')
disp('You can see the robot in the figure. Give a demonstration of a trajectory in its workspace')
% get a demonstration
nb_demo = 2;
nb_knots = 10;
nb_clean_data_per_demo = 50;
Data = [];
target = zeros(2,1);
for i=1:nb_demo
    data = get_demonstration(fig);
    nb_data = size(data,2);
    skip = floor(nb_data/nb_knots);
    knots = data(:,1:skip:end);
    ppx = spline(knots(3,:),knots(1:2,:));
    % % and get first derivative
    ppxd = differentiate_spline(ppx);
    tt = linspace(knots(3,1), knots(3,end), nb_clean_data_per_demo);
    pos = ppval(ppx,tt);
    target = target+pos(:,end);
    pos = pos - repmat(pos(:,end), 1, nb_clean_data_per_demo);
    vel = ppval(ppxd,tt);
    vel(:,end) = zeros(2,1);
    Data = [Data, [pos;vel]];
end
target = target/nb_demo;
plot(Data(1,:)+target(1), Data(2,:)+target(2), 'r.','markersize',20);

% learn SEDS model
options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                              % instabilities in Gaussian kernel [default: 10^-1                             
options.display = 1;          % An option to control whether the algorithm
                              % displays the output of each iterations [default: true]                            
options.tol_stopping=10^-6;  % A small positive scalar defining the stoppping
                              % tolerance for the optimization solver [default: 10^-10]
options.max_iter = 1000;       % Maximum number of iteration for the solver [default: i_max=1000]
options.objective = 'mse';    % 'likelihood': use likelihood as criterion to
                         
[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,nb_gaussians); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solver

ds = @(x) GMR(Priors,Mu,Sigma,x-repmat(target, 1, size(x,2)),1:2,3:4);
plot_ds_model(fig, ds);
% find an initial joint configuration for the start point
%qi = robot.ikine(transl(knots(1,1), knots(2,1),0.0),[0.2,0.2],[1,1,0,0,0,0]);
qi = simple_robot_ikin(robot,data(1:2,1));
robot.animate(qi);
% simulate tracking of the trajectory in the absence of perturbations
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
        qs = simple_robot_ikin(robot, xs);
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
      
            
        %h = plot(x_ref(1),x_ref(2),'go');
        ht = [];
        while(1)
            % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:2,4);
            xd = robot.jacob0(q)*qd';
            xd = xd(1:2);
          
            xd_ref = ds(x);%reference_vel(t);
            th = 1.0;
            if(norm(xd_ref)<th)
                xd_ref = xd_ref/norm(xd_ref)*th;
            end
            xdd_ref = -(xd - xd_ref)/dt*0.5;
            
            % compute cartesian control
            B = findDampingBasis(xd_ref);
            D = B*[4 0;0 8]*B';
            %D = 8*eye(2);
            u_cart = - D*(xd-xd_ref);
            % feedforward term
            u_cart = u_cart + simple_robot_cart_inertia(robot,q)*xdd_ref;
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
          
            ht = [ht, plot(x(1), x(2), 'm.')];
        end
      
        delete(ht);
    end


end

function B = findDampingBasis(xd)
 y1 = 1;
 y2 = -xd(1)/xd(2);
 y = [y1;y2];
 B = [xd./norm(xd), y./norm(y)];
end



