---
title: Background
---

## Overview

The underlying algorithm is the [alternating direction method of multipliers](https://stanford.edu/~boyd/admm.html){:target="_blank"}. TinyMPC reformulates the primal update step - the part that usually takes the longest - as an [LQR problem](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator){:target="_blank"}. These have been studied for decades, and we know how to write LQR problems in a closed form: specifically, using [Riccati recursion](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation){:target="_blank"}. We reorganize some of this recursive function to extract big matrices that only need to be computed once. In the vanilla implementation, this restricts TinyMPC to solving only a *linear* trajectory tracking problem (with any kinds of constraints, as long as they can be quickly re-linearized online). However, as seen in our demo videos, a single linearization can go a long way.

---

## Alternating direction method of multipliers (ADMM)

The alternating direction method of multipliers algorithm was developed in the 1970s and [used in 2011 by researchers at Stanford](https://stanford.edu/~boyd/papers/pdf/admm_distr_stats.pdf){:target="_blank"} to better solve the problem of distributed convex optimization. Some of these researchers later helped in developing [OSQP, the Operator Splitting Quadratic Program solver](https://osqp.org/){:target="_blank"}. TinyMPC takes much of its inspiration from these two sources.

We want to solve optimization problems in which our cost function $f$ and set of valid states $\mathcal{C}$ are both convex:

$$
\begin{alignat}{2}
\min_x & \quad f(x) \\
\text{subject to} & \quad x \in \mathcal{C}.
\end{alignat}
$$

We define an indicator function for the set $\mathcal{C}$:

$$
I_\mathcal{C}(x) =
\begin{cases}
0 & x \in \mathcal{C} \\
\infty & \text{otherwise}.
\end{cases}
$$

The indicator function says simply that there is infinite additional cost when $x$ violates the constraints (the state $x$ is outside the set of valid states $\mathcal{C}$) and zero additional cost for obeying the constraints ($x$ is inside the set $\mathcal{C}$). Thus, we need to be able to determine whether or not a state is in the set $\mathcal{C}$ in order to know if all the constraints on our problem are being met. For speed of computation, this often takes the form $Hx \geq h$ (or $Hx \leq h$, or a combination of $Hx \geq h$ and $Gx \leq g$ (each of these can be rewritten to be equivalent to the others)). This form can describe any kind of linear constraint in $x$. To do obstacle avoidance, for example, it is common to arrange $H$ and $h$ as half-space constraints where, in three dimensions, the entire space is split by a plane and only one half is inside the set $\mathcal{C}$. For arbitrary dimensionality, we say the space is divided by a hyperplane.

We modify the generic optimization problem to include the indicator function by adding it to the cost. We introduce a new state variable $z$, called the slack variable, to describe the constrained version of the original state variable $x$, which we will now call the primal variable.

Since both the state constraints ($\mathcal{X}$) and input constraints ($\mathcal{U}$) are convex, ADMM naturally decomposes the problem by projecting the primal variables ($x, u$) onto these convex constraint sets through the slack updates. This projection ensures constraint satisfaction and accelerates convergence by leveraging the separability of the constraint structure. The reduction via ADMM works by alternating between solving smaller subproblems for the primal and slack variables, significantly reducing the complexity of the original constrained optimization problem.



<!-- $$
\begin{alignat}{2}
\min_x & \quad f(x) + I_\mathcal{C}(z) \\
\text{subject to} & \quad x = z.
\end{alignat}
$$ -->

$$
\begin{alignat}{2}
\min_{x, u} & \quad f(x, u) + I_\mathcal{X}(z_x) + I_\mathcal{U}(z_u) \\
\text{subject to} & \quad x = z_x, \quad u = z_u.
\end{alignat}
$$


At minimum cost, the primal variable $x$ must be equal to the slack variable $z$, but during each solve they will not necessarily be equal. This is because the slack variable $z$ manifests in the algorithm as the version of the primal variable $x$ that has been projected onto the feasible set $\mathcal{C}$, and thus whenever the primal variable $x$ violates any constraint, the slack variable at that iteration will be projected back onto $\mathcal{C}$ and thus differ from $x$. To push the primal variable $x$ back to the feasible set $\mathcal{C}$, we introduce a third variable, $\lambda$, called the dual variable. This method is referred to as the [augmented Lagrangian](https://en.wikipedia.org/wiki/Augmented_Lagrangian_method){:target="_blank"} (originally named the method of multipliers), and introduces a scalar penalty parameter $\rho$ alongside the dual variable $\lambda$ (also known as a Lagrange multiplier). The penalty parameter $\rho$ is the augmentation to what would otherwise just be the Lagrangian of our constrained optimization problem above. $\lambda$ and $\rho$ work together to force $x$ closer to $z$ by increasing the cost of the augmented Lagrangian the more $x$ and $z$ differ.

$$
\mathcal{L}_A(x,z,\lambda) = f(x) + I_\mathcal{C}(z) + \lambda^\intercal(x-z) + \frac{\rho}{2}\|x-z\|^2_2.
$$

Our optimization problem has now been divided into two variables: the primal $x$ and slack $z$, and we can optimize over each one individually while holding all of the other variables constant. To get the ADMM algorithm, all we have to do is alternate between solving for the $x$ and then for the $z$ that minimizes our augmented Lagrangian. After each set of solves, we then update our dual variable $\lambda$ based on how much $x$ differs from $z$.

$$
\begin{alignat}{3}
\text{primal update: } & x^+ & ={} & \underset{x}{\arg \min} \hspace{2pt} \mathcal{L}_A(x,z,\lambda), \\
\text{slack update: } & z^+ & ={} & \underset{z}{\arg \min} \hspace{2pt} \mathcal{L}_A(x^+,z,\lambda), \\
\text{dual update: } & \lambda^+ & ={} & \lambda + \rho(x^+ - z^+),
\end{alignat}
$$

where $x^+$, $z^+$, and $\lambda^+$ refer to the primal, slack, and dual variables to be used in the next iteration.

Now all we have to do is solve a few unconstrained optimization problems!

<!-- ## TODO: primal and slack update and discrete algebraic riccati equation -->
---

## Primal and slack update

The primal update in TinyMPC takes advantage of the special structure of Model Predictive Control (MPC) problems. The optimization problem can be written as:

$$
\min_{x_{1:N}, u_{1:N-1}} J = \frac{1}{2}x_N^{\intercal}Q_fx_N + q_f^{\intercal}x_N + \sum_{k=1}^{N-1} \frac{1}{2}x_k^{\intercal}Qx_k + q_k^{\intercal}x_k + \frac{1}{2}u_k^{\intercal}Ru_k + r_k^{\intercal}u_k
$$

$$
\text{subject to: } x_{k+1} = Ax_k + Bu_k \quad \forall k \in [1,N)
$$

In addition to the dynamics constraints, the optimization problem also includes convex state and input constraints:

$$
x_k \in \mathcal{X}, u_k \in \mathcal{U} \quad \forall k \in [1,N)
$$

where $\mathcal{X}$ and $\mathcal{U}$ are convex sets representing the feasible state and input regions, respectively. These convex constraints ensure that the solution remains within feasible boundaries for both the state and the control inputs at every time step.

When we apply ADMM to this problem, the primal update becomes an equality-constrained quadratic program with modified cost matrices:

$$
\begin{aligned}
\tilde{Q}_f &= Q_f + \rho I, \quad \tilde{q}_f = q_f + \lambda_N - \rho z_N \\
\tilde{Q} &= Q + \rho I, \quad \tilde{q}_k = q_k + \lambda_k - \rho z_k \\
\tilde{R} &= R + \rho I, \quad \tilde{r}_k = r_k + \mu_k - \rho w_k
\end{aligned}
$$

This modified LQR problem has a closed-form solution through the discrete Riccati equation. The feedback law takes the form:

$$
u_k^* = -K_kx_k - d_k
$$

where $K_k$ is the feedback gain and $d_k$ is the feedforward term. These are computed through the backward Riccati recursion:

$$
\begin{aligned}
K_k &= (R + B^{\intercal}P_{k+1}B)^{-1}(B^{\intercal}P_{k+1}A) \\
d_k &= (R + B^{\intercal}P_{k+1}B)^{-1}(B^{\intercal}p_{k+1} + r_k) \\
P_k &= Q + K_k^{\intercal}RK_k + (A - BK_k)^{\intercal}P_{k+1}(A - BK_k) \\
p_k &= q_k + (A - BK_k)^{\intercal}(p_{k+1} - P_{k+1}Bd_k) + K_k^{\intercal}(Rd_k - r_k)
\end{aligned}
$$

The slack update is simpler, requiring only projection onto the constraint sets:

$$
\begin{aligned}
z_k^+ &= \text{proj}_{\mathcal{X}}(x_k^+ + y_k) \\
w_k^+ &= \text{proj}_{\mathcal{U}}(u_k^+ + g_k)
\end{aligned}
$$

where $\mathcal{X}$ and $\mathcal{U}$ are the feasible sets for states and inputs respectively, and $y_k, g_k$ are scaled dual variables.

A key optimization in TinyMPC is the pre-computation of certain matrices that remain constant throughout the iterations. Given a sufficiently long horizon, the Riccati recursion converges to the infinite-horizon solution, allowing us to cache:

$$
\begin{aligned}
C_1 &= (R + B^{\intercal}P_{\text{inf}}B)^{-1} \\
C_2 &= (A - BK_{\text{inf}})^{\intercal}
\end{aligned}
$$

This significantly reduces the online computational burden while maintaining the algorithm's effectiveness.

---

## Discrete Algebraic Riccati Equation (DARE)

For long time horizons, the Riccati recursion converges to a steady-state solution given by the discrete algebraic Riccati equation:

$$
P_{\text{inf}} = Q + A^{\intercal}P_{\text{inf}}A - A^{\intercal}P_{\text{inf}}B(R + B^{\intercal}P_{\text{inf}}B)^{-1}B^{\intercal}P_{\text{inf}}A
$$

This steady-state solution $P_{\text{inf}}$ yields a constant feedback gain:

$$
K_{\text{inf}} = (R + B^{\intercal}P_{\text{inf}}B)^{-1}B^{\intercal}P_{\text{inf}}A
$$

TinyMPC leverages this property by pre-computing these steady-state matrices offline, significantly reducing the online computational burden. The only online updates needed are for the time-varying linear terms in the cost function.

---

## Dual Updates and Convergence

The dual update step in ADMM pushes the solution toward constraint satisfaction:

$$
\begin{aligned}
y_k^+ &= y_k + x_k^+ - z_k^+ \\
g_k^+ &= g_k + u_k^+ - w_k^+
\end{aligned}
$$

where $y_k$ and $g_k$ are the scaled dual variables ($y_k = \lambda_k/\rho$ and $g_k = \mu_k/\rho$).

The algorithm terminates when both primal and dual residuals are sufficiently small:

$$
\begin{aligned}
\text{primal residual: } & \|x_k^+ - z_k^+\|_2 \leq \epsilon_{\text{pri}} \\
\text{dual residual: } & \rho\|z_k^+ - z_k\|_2 \leq \epsilon_{\text{dual}}
\end{aligned}
$$

where $\epsilon_{\text{pri}}$ and $\epsilon_{\text{dual}}$ are user-defined tolerance parameters.


<!--
this is an example of `code` in markdown
<!-- ``` py (or c or cpp) title="<custom title>" { .yaml .no-copy } --/>
``` julia
# This is a function
function function(x):
    return x**2 # (1)
```

1.  :man_raising_hand: I'm a code annotation! I can contain `code`, __formatted
    text__, images, ... basically anything that can be written in Markdown.

<!-- 
1.  :man_raising_hand: I'm $\beta$ $\int_5^{3x^2}$ a code annotation! I can contain `code`, __formatted
    text__, images, ... basically anything that can be written in Markdown.
-->

<!-- 
Hi this is something I am writing. (1)
{.annotate}

1. Hi this is an annotation with $\int_5^{3x^2}\sin(t) dt$, <span style="color:blue">some *colorful* text</span>, and emojis: :material-rocket:
-->