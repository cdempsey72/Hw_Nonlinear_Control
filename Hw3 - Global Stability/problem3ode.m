function dxdt = problem3ode(t,x,k1,k2)

dxdt1 = x(1) + 2*x(2);
u     = -1*k1*(x(1)+2*x(2)) - k2*(x(1)+2*x(2))^3; 
dxdt2 = 3*x(2) + 3*(x(1)+2*x(2))^3 + u;

dxdt = [dxdt1; dxdt2];