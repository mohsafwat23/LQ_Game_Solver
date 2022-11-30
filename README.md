# iLQGame solver for multi-agent robotic interactions

This repo is inspired by [ilqgames](https://arxiv.org/abs/1909.04694), efficient iterative linear-quadratic approxmiations for nonlinear multi-player general-sum differential games. 

It is still a work in progress. An example of a two 2D point-mass agents navigating at an intersection can be shown in the following GIF. 

![Alt Text](assets/two_agent.gif)

### Todo list:

- Add documentation and write report

- Receding horizon for online trajectory optimization

- Make a better GUI for the results
  - Graphical represenation of the robots pose and shape in plots
  - Add TurtleBot to MeshCat or on ROS to visualize results

-  ~~Put code in .jl file instead of .ipynb~~

- ~~Integrate the linear approximation of the nonlinear dynamics for multiple agents (linearize and discretize)~~
    - ~~Check that the point mass works using both dynamics~~

- Add state constraints

- ~~Use differential drive kinematics to use on TurtleBots~~

- Check augmented lagrangian dynamic programming for equality and inequality constraints

- Monte Carlo simulations to observe Nash Equilibria solutions
