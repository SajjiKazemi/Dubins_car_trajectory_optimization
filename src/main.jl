import Pkg
Pkg.activate("..")

using SCPToolbox
using PyPlot, Colors, LinearAlgebra

using ECOS

## Initialize the trajectory optimization problem
pbm = TrajectoryProblem()

## Define the problem dimensions
n, m, d = 3, 2, 1
problem_set_dims!(pbm, n, m, d)

## Define the dynamics
t_f = 3
f(t, x, u, p) = begin
    x, y, theta = x
    v, w = u
    return [v*sin(theta); v*cos(theta); w]*t_f
end;

## Define the Jaconbians of the dynamics
A(t, x, u, p) = begin
    x, y, theta = x
    v, w = u
    return [0.0 0.0 v*cos(theta);
            0.0 0.0 -v*sin(theta);
            0.0 0.0 0.0]*t_f
end

B(t, x, u, p) = begin
    x, y, theta = x
    v, w = u
    return [sin(theta) 0.0;
            cos(theta) 0.0;
            0.0 1.0]*t_f
end

F(t, x, u, p) = begin
    return zeros(3, 1)
end

## Use the toolbox API function which is problem_set_dynamics! for defining the dynamics
wrap(func) = (t, k, x, u, p, pbm) -> func(t, x, u, p)
problem_set_dynamics!(pbm, wrap(f), wrap(A), wrap(B), wrap(F))

## Define the boundary conditions when car doesn't reverse
x_0 = zeros(3)
x_f = [0; 2; 0]

## Define the boundary conditions when car reverses
# x_0 = [0; 0.5; 0]
# x_f = [0; 1.5; 0]

g_ic(x, p) = x - x_0
g_tc(x, p) = x - x_f;

## Define Jacobians for g_ic and g_tc
H_0(x, p) = I(3)
H_f(x, p) = I(3);

## Use the toolbox API function which is problem_set_bc! for defining the boundary conditions
wrap(func) = (x, p, pbm) -> func(x, p)
problem_set_bc!(pbm, :ic, wrap(g_ic), wrap(H_0))
problem_set_bc!(pbm, :tc, wrap(g_tc), wrap(H_f))

## Define the nonconvex obstacle constraint
c_0 = [-0.1; 1]
r_0 = 0.35
car_width = 0.1
delta_r_theta = car_width/2
E_xy = [1 0 0; 0 1 0]
s(t, x, u, p) = [(r_0 + delta_r_theta)^2 - (E_xy*(x - c_0))'*(E_xy*(x - c_0))];

## Define the Jacobian of the nonconvex obstacle constraint
C(t, x, u, p) = reshape(2*E_xy'*(c_0 - E_xy*x), 1, 3);

## Use the toolbox API function which is problem_set_s! for defining the nonconvex obstacle constraint
# We should also determine the SCP algorithm type
#alg = :scvx
alg = :ptr
wrap(func) = (t, k, x, u, p, pbm) -> func(t, x, u, p)
problem_set_s!(pbm, alg, wrap(s), wrap(C))
