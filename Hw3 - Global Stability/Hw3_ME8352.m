%%% ME8352 Homework 3 %%%

%% Problem 3
clear; clc; close all;
%%Define initial parameters and variables
k1s = 3; k2s = 3; %closed-loop stable
k1u = 2; k2u = 1; %closed-loop unstable
tspan = [0 60];
x0_array = linspace(-5,5,5);
xdot0_array = linspace(-3,3,5);
x0 = [];

f_s = figure; f_u = figure;
options = odeset('OutputFcn',@odephas2); %tell ode45 to output phase plots

for i = 1:length(x0_array)
    x_0 = x0_array(i);
    for ii = 1:length(xdot0_array)
        xdot_0 = xdot0_array(ii);
        %update initial conditions
        x0 = [x_0 xdot_0];
        %solve and plot phase plane
        figure(f_s)
        hold on
        [t_s,x_s] = ode45(@(t,x) problem3ode(t,x,k1s,k2s), tspan, x0, options);
        figure(f_u)
        hold on
        [t_u,x_u] = ode45(@(t,x) problem3ode(t,x,k1u,k2u), tspan, x0, options);
    end
end

figure(f_s)
title('Stable Phase Plot - k_1 = 3, k_2 = 3')
xlabel('x'); ylabel('xdot')

figure(f_u)
title('Unstable Phase Plot - k_1 = 2, k_2 = 1')
xlabel('x'); ylabel('xdot')

%% Problem 4
clear; clc; close all;
%%Define initial parameters and variables
K_s = 6;    %K > 5 --> stable
K_m = 5;    %K = 5 --> marginally stable
K_u = 4;    %K < 5 --> unstable

K_array = [K_s,K_m,K_u];
% K_array = 6;

x_0 = 1;    %initial state condition
xdot_0 = 0; %initial state condition
x0_array = linspace(-5,5,25);
xdot0_array = linspace(-3,3,25);
t_sim = 60; %set simulation time

f1 = figure; %phase plot figure K>5
f2 = figure; %phase plot figure K=5
f3 = figure; %phase plot figure K<5
f_handles = [f1,f2,f3];

for i = 1:length(K_array)
    %update initial condition
    K_ = K_array(i);
    for ii = 1:length(x0_array)
        x_0 = x0_array(ii);
        for iii = 1:length(xdot0_array)
        xdot_0 = xdot0_array(iii);
        %call simulink model
        states = sim('Hw3_ME8352sim.slx'); 
        %extract states
        t_out = states.tout; %time stamps
        x_t = states.x_pos.Data(:,1);
        x_dot = states.x_dot.Data(:,1);
        %plot phase plane
        figure(f_handles(i));
        hold on
        plot(x_t,x_dot)
        end
    end
end
hold off
%label figures
figure(f1);
title('Phase Plane K>5'); xlabel('x_t'); ylabel('x_{dot}')
figure(f2);
title('Phase Plane K=5'); xlabel('x_t'); ylabel('x_{dot}')
figure(f3);
title('Phase Plane K<5'); xlabel('x_t'); ylabel('x_{dot}')