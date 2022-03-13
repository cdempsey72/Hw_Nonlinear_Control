%%% ME8352 Homework 4 %%%
clear; clc; close all;
%%Define initial parameters and variables
k_ = 1; %stiffness coefficient
F0 = 1; %forcing amplitude
m = 1;  %unity mass

x0_array = linspace(1,2,5);   %initial state condition
% xdot0_array = linspace(0,1,5);%initial state condition
xdot0_array = 0.5;
t_sim = 20; %set simulation time

f1 = figure; %phase plot
f2 = figure; %time plot

for ii = 1:length(x0_array)
    x_0 = x0_array(ii);
    for iii = 1:length(xdot0_array)
        xdot_0 = xdot0_array(iii);
        %call simulink model
        states = sim('Hw4_ME8352sim.slx'); 
        %extract states
        t_out = states.tout; %time stamps
        x_t = states.x_pos.Data(:,1);
        x_dot = states.x_dot.Data(:,1);
        %plot phase plane
%         figure(f1);
%         hold on
%         plot(x_t,x_dot)
        %plot time behavior
        figure(f2);
        hold on 
        plot(t_out,x_t)
    end
end
hold off
%% label figures
figure(f1);
title('Phase plane'); xlabel('x'); ylabel('x_{dot}')
figure(f2);
title('x vs. time'); xlabel('x'); ylabel('time')
