%%% ME8352 Homework 1 %%%
clear; clc; close all;

%% Problem 1: x_dot = -x + x^2
%%Define initial parameters and arrays
xdot0_array = -1:0.1:1; 
xdot0_array = cat(2,xdot0_array,linspace(1.01,1.04,5));
t_sim = 3; %set simulation time
x_t = []; %create arrays to save simulated state output
x_dot = [];

%%Simulate
for i = 1:length(xdot0_array)
    %update initial condition
    x_dot0 = xdot0_array(i); 
    %call simulink model to evaluate nonlinear expression
    states = sim('Hw1_ME8352sim.slx'); 
    %extract states
    x_t(i,:) = states.x.Data(:,1);
    x_dot(i,:) = states.x_dot.Data(:,1);
end
t_out = states.tout; %extract simulation time stamps

%%Plot state behavior
figure
plot(t_out,x_t)
hold on
yline(0,'--','x = 0','LineWidth',1,'Color','k') %illustrate stable equilibrium at x = 0
yline(1,'--','x = 1','LineWidth',1,'Color','k') %illustrate unstable equilibrium at x = 1
hold off
xlabel('x position state')
ylabel('time')
title('Position v. Time')