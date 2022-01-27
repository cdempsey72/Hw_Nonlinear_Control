%%% ME8352 Homework 1 %%%

%% Problem 1: x_dot = -x + x^2
clear; clc; close all;
%%Define initial parameters and arrays
xdot0_array = -1:0.1:1; 
xdot0_array = cat(2,xdot0_array,linspace(1.01,1.04,5)); 
%the above concatenation is performed bc even small values >1 quickly cause the soln to blow up to inf!
t_sim = 3; %set simulation time
x_t = [];
x_dot = [];

%%Simulate
for i = 1:length(xdot0_array)
    %update initial condition
    x_dot0 = xdot0_array(i); 
    %call simulink model
    states = sim('Hw1_ME8352sim.slx'); 
    %extract states and store in an array
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

%% Problem 2: m*x_ddot + 2*c*(x^2 - 1)*x_dot + k*x = 0
clear; close all; clc;
%%Define initial parameters and arrays
x0_array = -3:1:3; 
xdot0_array =0;
t_sim = 25; %set simulation time
m = 1;      %mass
k = 2;      %stiffness coefficient
c = 3;     %damping coefficient

%%Simulate
f1 = figure;
f2 = figure;
hold on
for i = 1:length(x0_array)
    %update initial condition
    x_0 = x0_array(i); 
    for ii = 1:length(xdot0_array)
        %update initial condition
        x_dot0 = xdot0_array(ii); 
        %call simulink model
        states = sim('Hw1_ME8352sim.slx'); 
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
end
hold off
%label figures
figure(f1);
title('Phase Plane'); xlabel('x_t'); ylabel('x_dot')
figure(f2);
title('x v. time'); xlabel('time (s)'); ylabel('x_t')

%% Problem 3: v_dot = u - |v|*v
clear; close all; clc;
%%Define initial parameters and arrays
v0 = 0; 
vdot0 = 1.5; %note: changing initial state condition has litte/no effect on step response steady state result; but check what happens when v0>1!
step_array = 1:0.25:2;
t_sim = 10; %set simulation time
v_t = [];
v_dot = [];

%%Simulate
for i = 1:length(step_array)
    %update step input amplitude
    step_val = step_array(i); 
    %call simulink model
    states = sim('Hw1_ME8352sim.slx'); 
    %extract states and store in an array
    v_t(i,:) = states.v_out.Data(:,1);
    v_dot(i,:) = states.v_dot.Data(:,1);
end
t_out = states.tout; %extract simulation time stamps

%%Plot
figure
plot(t_out,v_t)
axis([0 t_out(end) 0 2])