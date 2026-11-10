# Philae optimal landing on comet 67P

Minimum-propellant landing of ESA's Philae lander on comet 67P/Churyumov–Gerasimenko, formulated as an optimal control problem and solved with Pontryagin's maximum principle and Particle Swarm Optimisation in MATLAB.

Course project for *Optimal Trajectories for Aerospace Vehicles*, Sapienza University of Rome (June 2025). The full report, in Italian, is in `ACCOMETAGGIO.pdf`.

## Problem

Philae is released by Rosetta 22.5 km above the surface, moving towards the comet at 0.187 m/s. The goal is to bring it to the surface with zero velocity while using as little propellant as possible.

Simplifying assumptions:

- flat comet with constant gravity, g = 1e-4 m/s²
- comet stationary with respect to the lander (rotation neglected)
- purely vertical descent

| Parameter | Value |
|---|---|
| Initial mass | 98 kg |
| Maximum thrust | 1 N (assumed) |
| Specific impulse | 60 s |
| Exhaust velocity, c = g0·Isp | 588.6 m/s |
| Initial altitude and velocity | 22 500 m, −0.187 m/s |
| Final altitude and velocity | 0.46 m (centre of mass above the landing feet), 0 m/s |
| Final time | free |

With x1 the altitude, x2 the velocity, x3 the mass and u ∈ [0, 1] the throttle, the state equations are:

$$\dot x_1 = x_2, \qquad \dot x_2 = \frac{T_{max}\,u}{x_3} - g, \qquad \dot x_3 = -\frac{T_{max}\,u}{c}$$

## Method

1. **Pontryagin's maximum principle.** The objective is to maximise the final mass. The Hamiltonian is linear in the control, so the optimal thrust is bang-bang, driven by the switching function

   $$\Theta = \frac{p_2}{x_3} - \frac{p_3}{c}, \qquad u = 1 \text{ if } \Theta > 0, \quad u = 0 \text{ if } \Theta < 0$$

   Singular arcs are ruled out analytically. With free final time and final mass, the transversality conditions give p3(tf) = 1 and H(tf) = 0.

2. **Shooting problem.** The unknowns are the initial costates and the final time, [p1(0), p2(0), p3(0), tf]. They are found by minimising a weighted error on the final conditions:

   $$J = \beta_1 |x_1(t_f) - 0.46| + \beta_2 |x_2(t_f)| + \beta_3 |p_3(t_f) - 1| + \beta_4 |H(t_f)|$$

3. **Global search with PSO.** The problem is extremely sensitive: changing p1(0) by 10⁻⁵ produces a completely different trajectory, so a local optimiser alone does not converge. A Particle Swarm Optimiser (1000 particles, 100 iterations) explores a narrow range, estimated from a physical analysis of when the engine must switch on, and locates the region of the global minimum.

4. **Local refinement.** The PSO solution is the initial guess for `fminsearch`, with the state and costate equations integrated by `ode113`.

## Files

| File | Content |
|---|---|
| `main_PSO_philae.m` | Particle Swarm Optimisation over the initial costates and final time |
| `CostFunction.m` | Cost function evaluated by the PSO |
| `Ottimizzazione_Philae.m` | Main script: local refinement with `fminsearch`, final simulation and plots |
| `lander_Philae.m` | Cost function minimised by `fminsearch` |
| `lander.m` | State and costate equations integrated by `ode113` |
| `ACCOMETAGGIO.pdf` | Full report (in Italian) |

## How to run

Requires MATLAB only, no additional toolboxes.

1. Run `main_PSO_philae.m` to find the initial costates and the final time.
2. Copy the values into `P_0` in `Ottimizzazione_Philae.m` and run it to refine the solution and plot altitude, velocity, mass and thrust.

`Ottimizzazione_Philae.m` already contains the values found by the PSO, so it can be run directly to reproduce the results below.

## Results

| Quantity | Value |
|---|---|
| Free-fall phase | 19 337 s |
| Thrust phase | 129 s at full thrust |
| Final altitude | 0.46 m (error 3.4 × 10⁻⁵ m) |
| Touchdown velocity | 0.80 m/s, below Philae's 1.1 m/s anchoring limit |
| Propellant used | 0.221 kg |

The lander falls freely for about 5.4 hours, reaching 2.11 m/s at 188 m above the surface, then fires its engine at full thrust until touchdown. The Hamiltonian stays constant and close to zero, as expected for a free-final-time problem.

## Limitations

- The comet's irregular shape and its rotation (12.4 h period) are neglected.
- The maximum thrust of Philae's cold-gas thruster was not available and is assumed to be 1 N.
- The touchdown velocity is not exactly zero: `fminsearch` could not reduce the error further from the PSO solution.

## Author

Andrea Bianchini, MSc student in Space and Astronautical Engineering, Sapienza University of Rome.
