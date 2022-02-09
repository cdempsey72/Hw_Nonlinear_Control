%%% ME8352 Homework 2 %%%

%% Problem 1 - Explore Duffing Equation
clear; clc; close all;
%%Define initial parameters and variables
b_ = 0.03; %damping coefficient
a_ = -1; %linear stiffness coefficient
beta_ = 3; %nonlinear stiffness coefficient
omega_ = 1.2;
% b_ = 0.3; %damping coefficient
% a_ = -1; %linear stiffness coefficient
% beta_ = 1; %nonlinear stiffness coefficient
% omega_ = 1.2;
gamma_array = linspace(0.2,0.6,10);

x_0 = 1; %initial state condition
xdot_0 = 0; %initial state condition

x0_array = linspace(0,1,10);

t_sim = 30; %set simulation time

% %%Simulate
% f1 = figure; %phase plot figure
% f2 = figure; %time plot figure
% hold on
% for i = 1:length(gamma_array)
%     %update initial condition
%     gamma_ = gamma_array(i);
%         %call simulink model
%         states = sim('Hw2_ME8352sim.slx'); 
%         %extract states
%         t_out = states.tout; %time stamps
%         x_t = states.x_pos.Data(:,1);
%         x_dot = states.x_dot.Data(:,1);
%         %plot phase plane
%         figure(f1);
%         hold on
%         plot(x_t,x_dot)
%         %plot time-based evolution
%         figure(f2);
%         hold on
%         plot(t_out,x_t)
% end
% hold off
% %label figures
% figure(f1);
% title('Phase Plane'); xlabel('x_t'); ylabel('x_{dot}')
% figure(f2);
% title('x v. time'); xlabel('time (s)'); ylabel('x_t')

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