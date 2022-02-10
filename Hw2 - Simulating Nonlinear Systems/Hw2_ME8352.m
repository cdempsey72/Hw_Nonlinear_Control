%%% ME8352 Homework 2 %%%

%% Problem 1 - Explore Duffing Equation
clear; clc; close all;
%%Define initial parameters and variables
b_ = 0.03; %damping coefficient
a_ = -1; %linear stiffness coefficient
beta_ = 3; %nonlinear stiffness coefficient
omega_ = 1.2;

gamma_array = linspace(0.2,0.6,10);
x_0 = 1; %initial state condition
xdot_0 = 0; %initial state condition
x0_array = linspace(0,1,10);
t_sim = 30; %set simulation time

%%Simulate
gamma_ = gamma_array(end);
f1 = figure; %phase plot figure
f2 = figure; %time plot figure
hold on
for i = 1:length(x0_array)
    %update initial condition
    x_0 = x0_array(i);
    %call simulink model
    states = sim('Hw2_ME8352sim.slx'); 
    %extract states
    t_out = states.tout; %time stamps
    x_t = states.x_pos.Data(:,1);
    x_dot = states.x_dot.Data(:,1);
    %plot phase plane
    figure(f1);
    hold on
    plot(x_t,x_dot)
    %plot time-based evolution
    figure(f2);
    hold on
    plot(t_out,x_t)
end
hold off
%label figures
figure(f1);
title('Phase Plane'); xlabel('x_t'); ylabel('x_{dot}')
figure(f2);
title('x v. time'); xlabel('time (s)'); ylabel('x_t')

%% Problem 3
clear; clc; close all;
%%Define initial parameters and variables
a_ = 1;     %switching line coefficient
U_ = 1;     %control input

th_0 = 0;    %initial state condition
th_dot0 = 2; %initial state condition
t_sim = 10; %set simulation time

% th0_array = linspace(0,2,10);
th0_array = pi/6;

%%Simulate
f1 = figure; %phase plot figure
f2 = figure; %time plot figure
hold on
for i = 1:length(th0_array)
    %update initial condition
    th_0 = th0_array(i);
    %call simulink model
    states = sim('Hw2_ME8352sim.slx'); 
    %extract states
    t_out = states.tout; %time stamps
    th = states.th.Data(:,1);
    th_dot = states.th_dot.Data(:,1);
    %plot phase plane
    figure(f1);
    hold on
    plot(th,th_dot)
    %plot time-based evolution
    figure(f2);
    hold on
    plot(t_out,th)
end
hold off
%label figures
figure(f1);
title('Phase Plane'); xlabel('th'); ylabel('th_{dot}')
figure(f2);
title('theta v. time'); xlabel('time (s)'); ylabel('theta')

%% Problem 4
clear; clc; close all;
%%Define initial parameters and variables
alpha_ = pi/16;      %switching line coefficient
length_a = 9.81; %control input
g = 9.81;        %gravitational constant
th_0 = 0;        %initial state condition
th_dot0 = 0;     %initial state condition
t_sim = 10;      %set simulation time

th0_array = linspace(0,pi/4,10);

%%Simulate
f1 = figure; %phase plot figure
f2 = figure; %time plot figure
hold on
for i = 1:length(th0_array)
    %update initial condition
    th_0 = th0_array(i);
        %call simulink model
        states = sim('Hw2_ME8352sim.slx'); 
        %extract states
        t_out = states.tout; %time stamps
        th = states.th.Data(:,1);
        th_dot = states.th_dot.Data(:,1);
        %plot phase plane
        figure(f1);
        hold on
        plot(th,th_dot)
        %plot time-based evolution
        figure(f2);
        hold on
        plot(t_out,th)
end
hold off
%label figures
figure(f1);
title('Phase Plane'); xlabel('th'); ylabel('th_{dot}')
figure(f2);
title('theta v. time'); xlabel('time (s)'); ylabel('theta')
