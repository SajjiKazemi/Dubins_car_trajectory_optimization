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



