%%% ME8352 Homework 4 %%%
clear; clc; close all;
%%initial parameters
k_ = 1; %stiffness coefficient
F0 = 1; %forcing amplitude
m = 1;  %unity mass
t_sim = 20; %simulation time

%%initial state conditions
%P2
% x0_array = linspace(1,5,25);
% xdot0_array = linspace(0.25,1,5);

%P3
x0_array = [-1,0,1];
xdot0_array = [0.25,0.5,1];

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
        %plot phase plane (P2)
        figure(f1);
        hold on
        plot(x_t,x_dot)
        %plot time behavior (P3)
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
