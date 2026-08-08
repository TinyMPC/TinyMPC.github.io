---
title: Model
description: How to obtain the dynamics model
---

## Linearization

TinyMPC in its vanilla implementation can only handle linear dynamics, which means systems must be linearized about an equilibrium before being used by the solver. Extensions to TinyMPC allow the user to better approximate a system's nonlinear dynamics by storing multiple linearizations, but we will start here with only one.

A discrete, linearized system takes the form $x_{k+1} = Ax_k + Bu_k$, where $x_k$ and $u_k$ are the state and control at the current time step, $A$ is the state-transition matrix, $B$ is the control or input matrix, and $x_{k+1}$ is the state at the next time step. Usually, this is derived from the continuous, nonlinear dynamics of the system, which takes the form $\dot{x} = f(x, u)$, where $f$ describes the instantaneous change in state at the current state and control input.

This page describes how to derive the discrete, linearized system dynamics from the continuous, nonlinear system dynamics.

<!-- For each of the examples given in [the previous page](./examples.md), the state-transition matrix $A$ and input matrix $B$ were computed from the system's continuous, nonlinear dynamics. (1)
{.annotate} -->

<!-- 1. The system still needs to be discretized even if it is already linear. This can be done with the matrix exponential or by the same methods shown for the nonlinear system below. -->

---

## Cart-pole example

The continuous time dynamics for the cart-pole [have](https://courses.ece.ucsb.edu/ECE594/594D_W10Byl/hw/cartpole_eom.pdf){:target="_blank"} [been](https://www.matthewpeterkelly.com/tutorials/cartPole/index.html){:target="_blank"} [derived](https://underactuated.mit.edu/acrobot.html){:target="_blank"} [many](https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html){:target="_blank"} [times](https://danielpiedrahita.wordpress.com/portfolio/cart-pole-control/){:target="_blank"}. For this example we'll use the convention from [this derivation](https://coneural.org/florian/papers/05_cart_pole.pdf){:target="_blank"}, where the pole is upright at $\theta=0$. If we ignore friction, the only equations we care about in that derivation are (23) and (24). (1)
{.annotate}

1. If you want to use a cart-pole model that has friction, use equations (21) and (22).

Let's write those down in a dynamics function

=== "Python"

    ```py
    import autograd.numpy as np
    
    mc = 0.2 # mass of the cart (kg)
    mp = 0.1 # mass of the pole (kg)
    ℓ = 0.5 # distance to the center of mass (meters)
    g = 9.81
    # (1)

    def cartpole_dynamics(x, u):
        r       = x[0] # cart position
        theta   = x[1] # pole angle
        rd      = x[2] # change in cart position
        theta_d = x[3] # change in pole angle
        F       = u[0] # force applied to cart
        
        theta_dd = (g*np.sin(theta) + np.cos(theta) * ((-F - mp*l*(theta_d**2) * \
                        np.sin(theta))/(mc + mp))) / (l*(4/3 - (mp*(np.cos(theta)**2))/(mc + mp)))
        rdd = (F + mp*l*((theta_d**2)*np.sin(theta) - theta_dd*np.cos(theta))) / (mc + mp)

        return np.array([rd, theta_d, rdd, theta_dd])
    ```

    1. Good practice would be to add an argument to $\text{cartpole_dynamics}$ that stores each of these parameters.

=== "Julia"

    ```julia
    mc = 0.2 # mass of the cart (kg)
    mp = 0.1 # mass of the pole (kg)
    ℓ = 0.5 # distance to the center of mass (meters)
    g = 9.81
    # (1)

    function cartpole_dynamics(x::Vector, u::Vector)
        r  = x[1] # cart position
        θ  = x[2] # pole angle
        rd = x[3] # change in cart position
        θd = x[4] # change in pole angle
        F  = u[1] # force applied to cart
        
        θdd = (g*sin(θ) + cos(θ) * ((-F - mp*ℓ*(θd^2) * sin(θ))/(mc + mp))) /
                 (ℓ*(4/3 - (mp*(cos(θ)^2))/(mc + mp)))
        rdd = (F + mp*ℓ*((θd^2)*sin(θ) - θdd*cos(θ))) / (mc + mp)
    
        return [rd; θd; rdd; θdd]
    end
    ```

    1. Good practice would be to add an argument to $\text{cartpole_dynamics}$ that stores each of these parameters.


This function describes the continuous (nonlinear) dynamics of our system, i.e. $\dot{x} = \text{cartpole_dynamics}(x, u)$. Before linearizing, we first discretize our continuous dynamics with an integrator. RK4 (Runge-Kutta 4th order) is a common explicit integrator, but you can write down whatever you like.

=== "Python"

    ```py
    def cartpole_rk4(x, u, dt):
        f1 = dt*cartpole_dynamics(x, u)
        f2 = dt*cartpole_dynamics(x + f1/2, u)
        f3 = dt*cartpole_dynamics(x + f2/2, u)
        f4 = dt*cartpole_dynamics(x + f3, u)
        return x + (1/6)*(f1 + 2*f2 + 2*f3 + f4)
    ```

=== "Julia"

    ```julia
    function cartpole_rk4(x::Vector, u::Vector, dt::Float64)
        f1 = dt*cartpole_dynamics(x, u)
        f2 = dt*cartpole_dynamics(x + f1/2, u)
        f3 = dt*cartpole_dynamics(x + f2/2, u)
        f4 = dt*cartpole_dynamics(x + f3, u)
        return x + (1/6)*(f1 + 2*f2 + 2*f3 + f4)
    end
    ```

Our integrator takes in the state and control at the current time step and integrates the state forward by $\Delta t$ seconds (the dt parameter in the RK4 function). We now have the discrete (nonlinear) dynamics of our system, defined by $x_{k+1} = \text{cartpole_rk4}(x_k, u_k, dt)$. Now, we linearize about an equilibrium position to obtain state-transition and input matrices $A$ and $B$ we described earlier. Differentiating $\text{cartpole_rk4}$ by hand would be a pain, but luckily we have access to automatic differentiation tools to do this for us.

=== "Python"

    ```py
    import autograd as AG

    xgoal = np.array([0.0, np.pi, 0.0, 0.0])
    ugoal = np.array([0.0])

    dt = 0.01

    A = AG.jacobian(lambda x_: cartpole_rk4(x_, ugoal, dt))(xgoal)
    B = AG.jacobian(lambda u_: cartpole_rk4(xgoal, u_, dt))(ugoal)
    ```

=== "Julia"

    ```julia
    import ForwardDiff as FD

    xgoal = [0; pi; 0; 0]
    ugoal = [0]

    dt = 0.01

    A = FD.jacobian(dx -> cartpole_rk4(params, dx, ugoal, dt), xgoal)
    B = FD.jacobian(du -> cartpole_rk4(params, xgoal, du, dt), ugoal)
    ```


Now all you have to do is save $A$ and $B$ and pass them to TinyMPC as shown in the problem setup section of the [examples page](examples.md).
