---
title: Inside TinyMPC
---

# Inside TinyMPC

Our 2024 ICRA submission video provides a concise overview of the solver:

[Watch the Video :fontawesome-brands-youtube:](https://www.youtube.com/watch?v=NKOrRyhcr6w){:target="_blank" .md-button}


## Problem formulation

TinyMPC solves convex quadratic model-predictive control programs of the form

$$
\begin{aligned}
\min_{x_{1:N}, u_{1:N-1}} \quad & J = \frac{1}{2} x_N^\intercal Q_f x_N + q_f^\intercal x_N + \sum_{k=1}^{N-1} \frac{1}{2} x_k^\intercal Q x_k + q_k^\intercal x_k + \frac{1}{2} u_k^\intercal R u_k + r_k^\intercal u_k \\
\text{subject to} \quad & x_{k+1} = A x_k + B u_k \quad \forall k \in [1, N), \\
& x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}
\end{aligned}
$$

where $x_k \in \mathbb{R}^n$, $u_k \in \mathbb{R}^m$ are the state and control input at time step $k$, $N$ is the number of time steps (also referred to as the horizon), $A \in \mathbb{R}^{n \times n}$ and $B \in \mathbb{R}^{n \times m}$ define the system dynamics, $Q \succeq 0$, $R \succ 0$, and $Q_f \succeq 0$ are symmetric cost weight matrices, and $q_k$, $r_k$, $q_f$ are linear cost vectors. The convex sets $\mathcal{X}$ and $\mathcal{U}$ represent state and input constraints respectively.

---

## Algorithm

TinyMPC employs the **Alternating Direction Method of Multipliers (ADMM)** to efficiently solve convex quadratic MPC problems. The algorithm separates dynamics constraints from other convex constraints, enabling the use of specialized techniques for each type.

### ADMM Framework

The algorithm reformulates the MPC problem by introducing slack variables and solving three iterative update steps:

$$
\begin{aligned}
\text{primal update: } \quad & (x^+, u^+) = \underset{x,u}{\arg \min} \, \mathcal{L}_A(x,u,z_x,z_u,\lambda,\mu) \\
\text{slack update: } \quad & (z_x^+, z_u^+) = \underset{z_x,z_u}{\arg \min} \, \mathcal{L}_A(x^+,u^+,z_x,z_u,\lambda,\mu) \\
\text{dual update: } \quad & \lambda^+ = \lambda + \rho(x^+ - z_x^+), \quad \mu^+ = \mu + \rho(u^+ - z_u^+)
\end{aligned}
$$

### Key Innovation: LQR-Based Primal Update

The **primal update** step is reformulated as a Linear Quadratic Regulator (LQR) problem, which has a closed-form solution through the **discrete Riccati recursion**. This is the computational bottleneck that TinyMPC optimizes.

The modified cost matrices for the LQR problem become:

$$
\begin{aligned}
\tilde{Q}_f &= Q_f + \rho I, \quad \tilde{q}_f = q_f + \lambda_N - \rho z_N \\
\tilde{Q} &= Q + \rho I, \quad \tilde{q}_k = q_k + \lambda_k - \rho z_k \\
\tilde{R} &= R + \rho I, \quad \tilde{r}_k = r_k + \mu_k - \rho w_k
\end{aligned}
$$

The optimal control policy is computed via backward Riccati recursion:

$$
\begin{aligned}
K_k &= (R + B^\intercal P_{k+1} B)^{-1}(B^\intercal P_{k+1} A) \\
d_k &= (R + B^\intercal P_{k+1} B)^{-1}(B^\intercal p_{k+1} + r_k) \\
u_k^* &= -K_k x_k - d_k
\end{aligned}
$$

### Constraint Handling via Projection

The **slack update** step handles convex constraints through simple projection operations:

$$
\begin{aligned}
z_k^+ &= \text{proj}_{\mathcal{X}}(x_k^+ + y_k) \\
w_k^+ &= \text{proj}_{\mathcal{U}}(u_k^+ + g_k)
\end{aligned}
$$

where $\mathcal{X}$ and $\mathcal{U}$ are the feasible sets for states and inputs respectively.

### Computational Optimization

For long horizons, TinyMPC pre-computes matrices that converge to steady-state values:

$$
\begin{aligned}
P_{\text{inf}} &= Q + A^\intercal P_{\text{inf}} A - A^\intercal P_{\text{inf}} B(R + B^\intercal P_{\text{inf}} B)^{-1} B^\intercal P_{\text{inf}} A \\
K_{\text{inf}} &= (R + B^\intercal P_{\text{inf}} B)^{-1} B^\intercal P_{\text{inf}} A
\end{aligned}
$$

This significantly reduces online computation by caching expensive matrix operations.

### Algorithm Pseudocode

```
Algorithm 1: TinyMPC
function TINY_SOLVE(input)
    while not converged do
        // Primal update
        p_{1:N-1}, d_{1:N-1} ← Backward pass via Riccati recursion
        x_{1:N}, u_{1:N-1} ← Forward pass via feedback law
        
        // Slack update  
        z_{1:N}, w_{1:N-1} ← Project to feasible set
        
        // Dual update
        y_{1:N}, g_{1:N-1} ← Gradient ascent update
        q_{1:N}, r_{1:N-1}, p_N ← Update linear cost terms
    end while
    return x_{1:N}, u_{1:N-1}
end function
```

---

## Implementations

The TinyMPC library offers a C++ implementation of the algorithm mentioned above, along with [interfaces to several high-level languages](../get-started/examples.md). This integration allows these languages to seamlessly solve optimal control problems using TinyMPC.

We have made available [Python](https://github.com/TinyMPC/tinympc-python), [MATLAB](https://github.com/TinyMPC/tinympc-matlab), and [Julia](https://github.com/TinyMPC/tinympc-julia) interfaces.

There are also several community-developed implementations of this algorithm: [Rust](https://github.com/peterkrull/tinympc-rs)

Numerical benchmarks against other solvers on microcontrollers are available at [this repository](https://github.com/RoboticExplorationLab/mcu-solver-benchmarks).

Crazyflie firmware with TinyMPC is available at [this repository](https://github.com/RoboticExplorationLab/tinympc-crazyflie-firmware).
