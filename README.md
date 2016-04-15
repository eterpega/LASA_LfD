# Introduction
This repository contains code and information for the computer exercises for the tutorial on Learning from Demonstration at ICRA 2016.

These exercises cover mainly representation and learning at the trajectory level, and should not be seen as a comprehensive set of exercises about the huge field of LfD.

These exercises use a simple two-link robot with dynamics from [Peter Corkes's robotics toolbox](http://petercorke.com/Robotics_Toolbox.html)[1]. 


# Exercise 1
The promise of Learning from Demonstration is to reduce the task of programming the robot to simply showing it what it should do. Perhaps the simplest possible way to achieve something like that is to simply record a trajectory and have the robot try to "play it back" exactly.

We encourage you to adjust any parameters you find, and rerun the function to understand the effect on the behavior of the robot. 

### Step 1
There are numerous ways in which we can give demonstrations to a robot. Since we will be working with planar task-spaces in these exercises, we can simply draw trajecories on the robot's task space. First, open the file `record_and_replay.m` and run it in matlab. A figure with the two-link robot as well as a dashed line delimiting its workspace is drawn. You are asked to provide a demonstration of a trajectory.

Keep in mind that you are providing a demonstration in *the workspace of the robot*, and hence if any part of the demonstration lies outside the boundary the robot will not be able to play it back. 

### Step 2
Your demonstration consists of a series of x,y coordinates unevenly distributed in time. In order to execute the trajecotory with the robot, we need access to at least a velocity profile and preferably also acceleration profile of the trajectory. The get a continous representaion that provides all these data, we can fit a spline representation to our demonstration.

With a spline representation, we don't need to use all of our demonstrated points. We can reduce the data set to a set of knots, which will be used to fit the spline. You can experiment with differnt number of knots by adjusting the nb_knots parameter in the matlab code. 

```matlab
nb_knots = 10;
disp(sprintf('you now see a spline with %d knots in the figure.', nb_knots));
% fit a spline to the demonstration
nb_data = size(data,2);
skip = floor(nb_data/nb_knots);
knots = data(:,1:skip:end);
```
Cubic splines is one possibility of interpolating the trajectory but there are also many others, feel free to test other methods built in to matlab by changing `'spline'` the following line of code:

```matlab
ppx = interp1(knots(3,:)',knots(1:2,:)','spline','pp');
```
Once we have generated the reduced data set we can interpolate it. The result is a series of 3rd degree polynomyials which can be easily differentiated to yield the velocity and acceleration profile. 

```matlab
% and get first and second derivative
ppxd = differentiate_spline(ppx);
ppxdd = differentiate_spline(ppxd);
```

### Step 3
If you have given a demonstration and see the figure with the spline representation of your demonstration, you can press enter to start a simulation. The simulation uses a rather simple cartesian controller with imperfect dynamic compensation, so it the trajecory may not be followd exactly if you gave a complex/fast demonstration.

### Step 4
Now suppose that the robot finds itself in another starting point, and we would like it to execute the task. The problem here is that we have not really demonstrated or encoded a task, but just a time-dependent trajectory. A naive approach of trying to use this trajecotory with other starting points will yield bad performance and is potentially unstable and very dangerous behavior. Try it out by clicking at a new location to start a simulation. The program will repeatedly ask you to click at new locations to simulate from that starting point.

At any time, you can abort the program by either rerunning the file or pressing `ctrl+c` in the matlab console.


# Excercise 2
In the previous exercise, we saw an example of a very basic record and replay approach which as we have seen is prone to error if the slightest thing in the environemnt changes between demonstration and execution.

A large body of work in LfD has been devoted to finding alternative means of representing demonstrated trajectories, so that the robot can *generalize* the demonstrations to new situations (e.g. new starting points). Many of these methods have been mentioned in the tutorial session. Here, we will focus on Dynamical Systems (DS) representations of a task. Specifically, we will use time-invariant DS representations. The methods used in this exercise is Stable Estimator of Dynamical Systems, SEDS [2].

### Step 1
Open the file `ds_learning.m` and run it. Again, you will see the two-link robot appearing in a figure with a workspace limit shown by a dashed line. Start by again giving a demonstration of a path in the robot workspace.

### Step 2
The source code starts with collection of a demonstration and processing of the demonstration to find velocity etc using splines, just as in Exercise 1. Scroll down until you find the following code:

```matlab
[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,nb_gaussians); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solver
```
In the first of line fo the code above creates a Gaussian Mixture Model(GMR) representing the joint probability density function of the demonstrated position and velocity. By conditioning this distribution, so that we get a distriubtion of velocity *given* position, we are getting close to something we can use for control. However, if we just used the first estimate, we would have an *unstable* model, with diverging trajecories.

The second line of code starts the SEDS optimizer, which modifies the GMM such that it yields a stable DS when conditioned. This is done in a constrained optimization procedure. Depending on the complexity of your demonstration, it is possible that no stable soluation can be found, or a solution that does not fit the data well. Rerun the program until the optimization finishes succesfully. 

### Step 3
Now that we have a DS model, inspect the behavior by launching simulations from differnt parts of the workspace by clikcing there. As you can see, the robot always follows smooth trajectories that end up at the demonstrated end point.

### Step 4
So far, we have only given one demonstration. In order to generalize, it is generally good to provide several demonstrations. Find the parameter `nb_demo` near the top of the main function and change a higher number, e.g. 2 or 3. You can now give several demonstrations. The demonstrations will be translated so that they all end at the same point. Try demonstrating differnt trajecotries in differnt parts of the workspace. 

### Step 5
So far, we have used a GMM with a single Gaussian to model the DS. To model more complex data, more Gaussians can be required. Find the parameter `nb_gaussians` and change it to a higher number, e.g. 2.

Note that it is generally a good idea to keep the number of Gaussians low, as the optimization is more likely to end up in a bad local minima with more Gaussians. It is not always possible to get a very accurate reproduction of the trajectory this way. SEDS should not be seen as a precision tool but rather a method that can capture the general motion pattern and generalize it.

### Step 6
Simulate a couple of trajcories by clicking at different starting points. Rerun the program and adjust the paramters to see how it influences the result.

# Exercise 3
In the last exercise we introduced time-invariant DS as a way of encoding motions, and we explored the SEDS algorithm for learning a DS on a particular form (GMR) from demonstrations.

SEDS is a global method, in the sense that it tries to generalize the demonstrations to any part of the state space. As you have seen in the previous exercise, the result is not always suitable and the cost of generalization is local precision. The demonstrated trajectories were not reproduced to the same level of accuracy as in the time-dependent record-and-replay exercise.

In this exeercise, we will explore *local* learning in DS represnetations. We will use SEDS as a global baseline model, and then we will *reshape* it locally in order to improve accuracy near the demonstrations. We will use the method described in [3]. 

# References
1. P.I. Corke, “Robotics, Vision & Control”, Springer 2011, ISBN 978-3-642-20143-1
2. Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models. IEEE Transaction on Robotics, vol. 27, num 5, p. 943-957 [link to pdf](http://lasa.epfl.ch/publications/uploadedFiles/Khansari_Billard_TRO2011.pdf)
3. Kronander, K., Khansari Zadeh, S. M. and Billard, A. (2015) Incremental Motion Learning with Locally Modulated Dynamical Systems. Robotics and Autonomous Systems, 2015, vol. 70, iss. 1, pp. 52-62. [link to pdf](http://lasa.epfl.ch/publications/uploadedFiles/LMDS_els.pdf)

