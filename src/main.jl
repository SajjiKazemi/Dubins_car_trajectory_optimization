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

## Define the objective function
Q(x, u, p) = u'*u;

## Define the cost using the toolbox API function as follows:
## problem_set_terminal_cost!
## problem_set_running_cost!
wrap(func) = (t, k, x, u, p, pbm) -> func(x, u, p)
problem_set_running_cost!(pbm, alg, wrap(Q))

## Define the initial guess
state_guess(N) = straightline_interpolate(x_0, x_f, N)
input_guess(N) = straightline_interpolate(zeros(2), zeros(2), N);

## Use the toolbox API function which is problem_set_guess! for defining the initial guess
problem_set_guess!(pbm, (N, pbm) -> begin
    x = state_guess(N)
    u = input_guess(N)
    p = zeros(1)
end)

## Configure the solver
# PTR parameters
N, Nsub = 11, 10
iter_max = 30
disc_method = FOH
wvc, wtr = 1e3, 1e0
feas_tol = 5e-3
eps_abs, eps_rel = 1e-5, 1e-3
q_tr = Inf
q_exit = Inf
solver, solver_options = ECOS, Dict("verbose" => 0)
pars = PTR.Parameters(N, Nsub, iter_max, disc_method, wvc, wtr, eps_abs,
                      eps_rel, feas_tol, q_tr, q_exit, solver, solver_options);
if alg == :ptr
    ptr_pbm = PTR.create(pars, pbm)
    sol, history = PTR.solve(ptr_pbm)
end;

# SCvx parameters
# N, Nsub = 11, 10
# iter_max = 30
# disc_method = FOH
# lambda = 1000.0
# rho_0 = 0.0
# rho_1 = 0.1
# rho_2 = 0.7
# beta_sh = 2.0
# beta_gr = 2.0
# eta_init = 1.0
# eta_lb = 1e-3
# eta_ub = 10.0
# feas_tol = 5e-3
# eps_abs, eps_rel = 1e-5, 1e-3
# q_tr = Inf
# q_exit = Inf
# solver, solver_options = ECOS, Dict("verbose" => 0)
# pars = SCvx.Parameters(N, Nsub, iter_max, disc_method, lambda, rho_0, rho_1, rho_2,
#                        beta_sh, beta_gr, eta_init, eta_lb, eta_ub, eps_abs, eps_rel,
#                        feas_tol, q_tr, q_exit, solver, solver_options);
# if alg == :scvx
#     scvx_pbm = SCvx.create(pars, pbm)
#     sol, history = SCvx.solve(scvx_pbm)
# end;

