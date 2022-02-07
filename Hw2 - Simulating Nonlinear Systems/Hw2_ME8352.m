%%% ME8352 Homework 2 %%%

%% Problem 1 - Explore Duffing Equation
clear; clc; close all;
%%Define initial parameters and variables
b_ = 0.3; %damping coefficient
a_ = -1; %linear stiffness coefficient
beta_ = 1; %nonlinear stiffness coefficient
omega_ = 1.2;
gamma_ = 0.37;
x_0 = 1; %initial state condition
xdot_0 = 0; %initial state condition
t_sim = 30; %set simulation time
