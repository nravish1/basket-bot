import math
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Import your existing simulation code from trajectory_simulation.py
# (Make sure trajectory_simulation.py is in the same folder or on your Python path!)
from trajectory_simulation import (
    solve_projectile_rk4,
    projectile_equations_drag
)

def projectile_state_at_time(func, v, theta_deg, t_target, dt=0.001):
    """
    Simulate the projectile (with drag or no drag) up to time t_target,
    and return (x, y, vx, vy) at that instant.
    
    :param func:       e.g., projectile_equations_drag or projectile_equations_no_drag
    :param v:          launch speed (m/s)
    :param theta_deg:  launch angle in degrees
    :param t_target:   time at which we want the state
    :param dt:         RK4 step size
    :return: (x(t_target), y(t_target), vx(t_target), vy(t_target))
    """

    # We can simply integrate up to t_target using solve_projectile_rk4,
    # then pick the final point if it didn't stop earlier.
    # But solve_projectile_rk4 might stop early if the projectile hits y<0.
    # We'll forcibly integrate up to t_target by adjusting its internal 't_max'.
    times, x_vals, y_vals = solve_projectile_rk4(
        func=func,
        v0=v,
        theta_deg=theta_deg,
        t_max=t_target,  # integrate exactly to t_target
        dt=dt
    )
    
    # Because we might break early in solve_projectile_rk4 if y<0,
    # let's see how many steps we got:
    last_idx = len(times) - 1
    # If times[-1] < t_target, the ball might have already “hit the ground.” 
    # We'll just treat the last index as the best we can do
    # and won't strictly enforce time if it’s incomplete or the ball is “dead.”

    # We also need vx, vy, but the solve function currently returns only x,y.
    # So let's do a direct single-step approach here if we want the velocity too.
    # Alternatively, you can modify solve_projectile_rk4 to also return velocity arrays.
    # For simplicity, let's do it ourselves in-line with RK4 steps:

    # -----------
    # A direct approach to get the full state at t_target:
    # -----------
    # We'll replicate the integration logic to get [x, y, vx, vy] at t_target:
    # (Implementation example below.)
    # -----------
    return _integrate_full_state(func, v, theta_deg, t_target, dt)


def _integrate_full_state(func, v, theta_deg, t_target, dt=0.001):
    """
    Internal helper that integrates [x, y, vx, vy] up to t_target
    regardless of the projectile dropping below y=0.
    """
    import math

    # Initial conditions
    theta = math.radians(theta_deg)
    x0 = 0.0
    y0 = 0.0
    vx0 = v * math.cos(theta)
    vy0 = v * math.sin(theta)
    u = [x0, y0, vx0, vy0]
    
    t = 0.0
    # We'll do small RK4 steps until we reach t_target
    # or exceed it. Then do a partial step if needed.
    
    while t < t_target:
        dt_step = min(dt, t_target - t)
        u, t = _rk4_step(func, t, u, dt_step)
        if abs(t - t_target) < 1e-9:
            break
    
    x, y, vx, vy = u
    return x, y, vx, vy


def _rk4_step(func, t, u, dt):
    """
    One RK4 step. The 'func' is something like projectile_equations_drag.
    u = [x, y, vx, vy]
    Returns (u_next, t_next).
    """
    k1 = func(t, u)
    k2 = func(t + 0.5*dt, [u[i] + 0.5*dt*k1[i] for i in range(len(u))])
    k3 = func(t + 0.5*dt, [u[i] + 0.5*dt*k2[i] for i in range(len(u))])
    k4 = func(t + dt,    [u[i] + dt*k3[i]      for i in range(len(u))])

    u_next = [
        u[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i])
        for i in range(len(u))
    ]
    return (u_next, t + dt)


##############################################################################
# Constrained optimization (SLSQP) to find v, theta, t that minimize v
# subject to:
#   1) x(t) = x_hoop
#   2) y(t) = y_hoop
#   3) vy(t) <= 0
#   4) 0 < theta < 90 deg
#   5) v>0, t>0
##############################################################################

def _objective(z):
    """
    z = [v, theta_deg, t].
    Minimize the launch speed v.
    """
    v = z[0]
    return v  # we want to minimize v

def ineq_approach_angle(z):
    """
    Force the ball to come in at or below a specified downward angle (steepness).
    angle_degrees = atan2(vy, vx) in degrees.
    We want angle_degrees <= -25 => angle_degrees + 25 <= 0.
    => define ineq = -(angle_degrees + 25), so ineq >= 0 => angle_degrees <= -25.
    """
    v, theta_deg, t = z
    x, y, vx, vy = projectile_state_at_time(projectile_equations_drag, v, theta_deg, t)
    angle_radians = math.atan2(vy, vx)
    angle_degrees = math.degrees(angle_radians)
    min_downward_angle = 50.0  # For example, want -25° approach or steeper.
    return -(angle_degrees + min_downward_angle)

def _constraints(func, x_hoop, y_hoop):
    """
    SLSQP constraints for the system:
      eq1:  x(t) - x_hoop = 0
      eq2:  y(t) - y_hoop = 0
      ineq1: vy(t) <= 0  ->  vy(t) = ???    so we want vy(t) <= 0 => -vy(t) >= 0
      ineq2: v >= 0
      ineq3: t >= 0
      ineq4: theta_deg >= 0
      ineq5: 90 - theta_deg >= 0  =>  theta_deg <= 90
    """
    g = 9.81

    def eq_x(z):
        v, theta_deg, t = z
        x, y, vx, vy = projectile_state_at_time(func, v, theta_deg, t)
        return x - x_hoop

    def eq_y(z):
        v, theta_deg, t = z
        x, y, vx, vy = projectile_state_at_time(func, v, theta_deg, t)
        return y - y_hoop

    def ineq_vy_down(z):
        v, theta_deg, t = z
        x, y, vx, vy = projectile_state_at_time(func, v, theta_deg, t)
        # we want vy <= 0 => -vy >= 0
        return -vy  # >= 0

    def ineq_v_positive(z):
        return z[0]  # v>=0 => v>0 in practice

    def ineq_t_positive(z):
        return z[2]  # t>=0 => t>0 in practice

    def ineq_theta_lower(z):
        return z[1]  # theta_deg>=0 => we prefer strictly positive, but okay

    def ineq_theta_upper(z):
        return 90.0 - z[1]  # => theta_deg <= 90
    
    return [
        {'type': 'eq', 'fun': eq_x},
        {'type': 'eq', 'fun': eq_y},
        {'type': 'ineq', 'fun': ineq_vy_down},
        {'type': 'ineq', 'fun': ineq_v_positive},
        {'type': 'ineq', 'fun': ineq_t_positive},
        {'type': 'ineq', 'fun': ineq_theta_lower},
        {'type': 'ineq', 'fun': ineq_theta_upper},
        {'type': 'ineq', 'fun': ineq_approach_angle},
    ]

def _debug_callback(z):
    """
    This function is called by the SLSQP solver at each iteration.
    z = [v, theta_deg, t].
    """
    v, theta_deg, t = z
    print(f"[DEBUG] Iteration: v={v:.4f}, theta={theta_deg:.3f} deg, t={t:.3f}")

def find_minimum_velocity_with_drag(x_hoop, y_hoop, v_init=8.0, theta_init=45.0, t_init=1.0):
    """
    Attempt to solve:
       minimize v
       s.t.  x(t)=x_hoop, y(t)=y_hoop, and vy(t)<=0
             v>0, t>0, 0<theta<90
    Using projectile_equations_drag from trajectory_simulation.py.

    Returns (v_opt, theta_opt, t_opt, full_result) if successful, else None.
    """
    # Build constraints
    cons = _constraints(projectile_equations_drag, x_hoop, y_hoop)
    
    # Initial guess
    z0 = np.array([v_init, theta_init, t_init])

    print(f"[DEBUG] Starting optimization with initial guess: v={v_init}, theta={theta_init}, t={t_init}")
    
    # Minimize
    res = minimize(
        fun=_objective,
        x0=z0,
        method='SLSQP',
        constraints=cons,
        callback=_debug_callback,   # Debug callback
        options={'ftol': 1e-7, 'maxiter': 400, 'disp': True}  # 'disp': True prints solver progress
    )
    
    if not res.success:
        print("[DEBUG] Optimization failed or no valid solution found.")
        return None
    
    v_opt, theta_opt_deg, t_opt = res.x
    print("[DEBUG] Optimization succeeded.")
    return (v_opt, theta_opt_deg, t_opt, res)

def main():
    # Example: Suppose the hoop is 7.24 m away and 3.05 m high (approx. 10 ft).
    hoop_x = float(input("Input horizontal distance: "))
    hoop_y = 3.05
    hoop_r  = 0.23

    result = find_minimum_velocity_with_drag(hoop_x, hoop_y, v_init=8.0, theta_init=45.0, t_init=1.0)
    if result is None:
        print("No valid solution found by the optimizer.")
    else:
        v_opt, theta_opt_deg, t_opt, sol = result
        print("Optimizer found a solution:")
        print(f"  v    = {v_opt:.4f} m/s")
        print(f"  theta= {theta_opt_deg:.3f} degrees")
        print(f"  t    = {t_opt:.4f} s")
        # If you want to see full SLSQP debug info:
        # print(sol)

        t_drag, x_drag, y_drag = solve_projectile_rk4(
            projectile_equations_drag, v_opt, theta_opt_deg
        )

        fig, ax = plt.subplots()

        ax.plot(x_drag, y_drag, label="With Drag")

        # Draw hoop (as an unfilled circle)
        hoop_circle = Circle((hoop_x, hoop_y), hoop_r, fill=False, label="Hoop")
        ax.add_patch(hoop_circle)

        ax.set_xlabel("Horizontal position (m)")
        ax.set_ylabel("Vertical position (m)")
        ax.set_title(f"Projectile Motion (v0={v_opt:.4f} m/s, angle={theta_opt_deg:.3f}°)")
        ax.set_aspect("equal", "box")
        ax.legend()
    
        plt.show()

if __name__ == "__main__":
    main()
