import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def projectile_equations_no_drag(t, u):
    """
    ODEs for projectile motion without drag:
    u = [x, y, vx, vy].
    """
    g = 9.81  # m/s^2
    x, y, vx, vy = u

    dxdt = vx
    dydt = vy
    dvxdt = 0.0
    dvydt = -g

    return [dxdt, dydt, dvxdt, dvydt]

def projectile_equations_drag(t, u):
    """
    ODEs for projectile motion with drag:
    u = [x, y, vx, vy].
    Drag formula: F_drag = -c * v * |v|
    """
    g = 9.81  # m/s^2

    # Suppose you derived this from: c = 0.5*(rho*Cd*A)/m
    # for a typical basketball:
    c = 0.022
    
    x, y, vx, vy = u
    speed = math.sqrt(vx*vx + vy*vy)

    dxdt = vx
    dydt = vy
    dvxdt = -c * vx * speed
    dvydt = -g - c * vy * speed
    
    return [dxdt, dydt, dvxdt, dvydt]

def rk4_step(func, t, u, dt):
    """
    Perform one RK4 step for u' = func(t, u).
    :param func: function f(t, u) -> list of derivatives
    :param t: current time
    :param u: current state vector/list
    :param dt: timestep
    :return: (u_next, t_next)
    """
    k1 = func(t, u)
    k2 = func(t + 0.5*dt, [u[i] + 0.5*dt*k1[i] for i in range(len(u))])
    k3 = func(t + 0.5*dt, [u[i] + 0.5*dt*k2[i] for i in range(len(u))])
    k4 = func(t + dt,    [u[i] + dt*k3[i]      for i in range(len(u))])

    u_next = [
        u[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i])
        for i in range(len(u))
    ]
    t_next = t + dt
    return u_next, t_next

def solve_projectile_rk4(func, v0, theta_deg, t_max=2.0, dt=0.01):
    """
    Solve the projectile motion ODEs (func) from t=0 to t=t_max using RK4.
    
    :param func: The derivative function (e.g., projectile_equations_no_drag or projectile_equations_drag)
    :param v0:   initial launch speed (m/s)
    :param theta_deg: initial launch angle in degrees
    :param t_max: final time of integration (s)
    :param dt:    timestep (s)
    :return: (times, x_vals, y_vals)
    """
    # Convert angle to radians
    theta = math.radians(theta_deg)
    
    # Initial conditions (assuming launch from (0,0)):
    x0 = 0.0
    y0 = 0.0
    vx0 = v0 * math.cos(theta)
    vy0 = v0 * math.sin(theta)
    
    # State: u = [x, y, vx, vy]
    u = [x0, y0, vx0, vy0]
    t = 0.0

    # Lists to store trajectory
    times = [t]
    x_vals = [x0]
    y_vals = [y0]

    n_steps = int(t_max / dt)
    for _ in range(n_steps):
        u, t = rk4_step(func, t, u, dt)
        times.append(t)
        x_vals.append(u[0])
        y_vals.append(u[1])
        
        # Optional stopping if ball hits the ground (y < 0) again:
        if u[1] < 0:
            break
    
    return times, x_vals, y_vals

def main():
    # Initial speed and angle
    v0 = 11.0        # m/s
    angle_deg = 60.0 # degrees
    t_max = 2.0      # seconds
    dt = 0.01        # time step

    # Solve for both scenarios
    t_no_drag, x_no_drag, y_no_drag = solve_projectile_rk4(
        projectile_equations_no_drag, v0, angle_deg, t_max, dt
    )
    t_drag, x_drag, y_drag = solve_projectile_rk4(
        projectile_equations_drag, v0, angle_deg, t_max, dt
    )

    # Hoop parameters
    x_hoop = 6.75      # Horizontal position of hoop (m); 3 point line is 6.75m
    y_hoop = 3.05     # Vertical position of hoop (m) ~ 10 ft
    r_hoop  = 0.23    # Approx basketball hoop radius (m)

    # Plot both trajectories
    fig, ax = plt.subplots()

    ax.plot(x_no_drag, y_no_drag, label="No Drag")
    ax.plot(x_drag, y_drag, label="With Drag")

    # Draw hoop (as an unfilled circle)
    hoop_circle = Circle((x_hoop, y_hoop), r_hoop, fill=False, label="Hoop")
    ax.add_patch(hoop_circle)

    ax.set_xlabel("Horizontal position (m)")
    ax.set_ylabel("Vertical position (m)")
    ax.set_title(f"Projectile Motion Comparison (v0={v0} m/s, angle={angle_deg}°)")
    ax.set_aspect("equal", "box")
    ax.legend()
    
    plt.show()

if __name__ == "__main__":
    main()
