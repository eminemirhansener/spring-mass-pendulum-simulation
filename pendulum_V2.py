import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class PendulumSystem:
    def __init__(self, m: float, L: float, M: float, k_coeff: float, b_coeff: float, gravity: float = 9.81):
        self.m = m
        self.L = L
        self.M = M
        self.k_coeff = k_coeff
        self.b_coeff = b_coeff
        self.gravity = gravity

        self.A, self.B = self.calculate_state_matrices()
    
    # State-Space matrices are calculated.
    def calculate_state_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        denominator = ((1/3) * self.m + self.M) * self.L**2

        if denominator == 0:
            raise ValueError("Denominator is zero. Physical parameters are invalid.")
        
        # A matrix
        a11 = 0
        a12 = 1
        a21 = -((self.gravity * self.L * ((self.m / 2) + self.M) + self.k_coeff * self.L**2) / denominator)
        a22 = -(self.b_coeff / denominator)

        A = np.array([[a11, a12],
                      [a21, a22]])
        
        # B matrix
        b1 = 0
        b2 = 1 / denominator

        B = np.array([[b1],
                      [b2]])

        return A, B
    
    # This function is used to create input torque which is sinusoidal in default.
    def input_function(self, t: float) -> np.ndarray:
        amplitude = 5.0
        frequency = 2.0
        U_scalar = amplitude * np.sin(2 * np.pi * frequency * t)
        
        return np.array([[U_scalar]])
        # return np.array([[5]]) # -> Step input
    
    # This function solves the state-space equation iteratively using euler forward euler integration.
    # This function returns the history of time data, state parameters (x, x') and input torque in
    # predetermined time with time step.
    def run_dynamics(self, t_final: float, dt: float, X0: np.ndarray, input_func=None):
        if input_func is None:
            input_func = self.input_function
        
        X = X0.copy()

        time_points = np.arange(0, t_final, dt)
        
        num_steps = len(time_points)
        pos_history = np.zeros(num_steps)
        vel_history = np.zeros(num_steps)
        u_history = np.zeros(num_steps)

        for i, t in enumerate(time_points):
            U = input_func(t)
            
            pos_history[i] = X[0, 0]
            vel_history[i] = X[1, 0]
            u_history[i] = U[0, 0]
            
            X_dot = self.A @ X + self.B @ U 
            
            X = X + X_dot * dt 

        return time_points, pos_history, vel_history, u_history

# -------------------------------------------------------------------------------------------
# -------------------------------------- VISUALIZATION --------------------------------------
# -------------------------------------------------------------------------------------------

# This function returns final system results as time responses. 
# position - time
# velocity - time
# input torque - time
def plot_system_results(time, position, velocity, Input):

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # ---------- POSITION ----------
    axes[0].plot(time, position, label='Angle ($\phi$)', color='blue')
    axes[0].set_ylabel('Angle (rad)', fontsize=12)
    axes[0].legend(loc='upper right')
    axes[0].grid(True)

    # ---------- VELOCITY ----------
    axes[1].plot(time, velocity, label='Angular Velocity ($\dot{\phi}$', color='orange')
    axes[1].set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    axes[1].legend(loc='upper right')
    axes[1].grid(True)

    # ---------- INPUT ----------
    axes[2].plot(time, Input, label='Input ($\phi$)', color='red')
    axes[2].set_ylabel('Input (N.m)', fontsize=12)
    axes[2].legend(loc='upper right')
    axes[2].grid(True)

    plt.tight_layout()

    plt.show()

# This function generates a spring horizontally with parameters; initial (fixed), final(moving)
# points, and n which is number of windings
def generate_spring(fixed, moving, n):
    spring_x_data = np.linspace(fixed[0], moving[0], n+14)
    height = spring_x_data[1] - spring_x_data[0]
    spring_y_data = np.array([fixed[1] - height, fixed[1], fixed[1] + height])

    points = []
    points.append([spring_x_data[0], fixed[1]]) # initial point
    points.append([spring_x_data[6], fixed[1]]) # initial point

    for i in range(7, n+7):
        y_index = (i + 1) % 2 * 2 # i-> 0,2,0,2 ...
        points.append([spring_x_data[i], spring_y_data[y_index]])

    points.append([spring_x_data[n + 7], moving[1]]) # final point
    points.append([spring_x_data[n + 13], moving[1]]) # final point

    spring_data = np.array(points)

    return spring_data

# This function rotates point cloud around a specific center point with an angle_rad.
def rotate_around_point(points, center, angle_rad):
    R = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])

    shifted = points - center
    rotated = shifted @ R.T
    return rotated + center

# This function scales point cloud around a specific center point with scale factor scale_x
# and scale_y.
def scale_around_point(points, center, scale_x = 1.0, scale_y = 1.0):
    S = np.array([
        [scale_x, 0],
        [0, scale_y]
    ])

    shifted = points - center
    scaled = shifted @ S.T
    return scaled + center

# This function simulates the model in realtime. Both animates and plot important graphs
# simultaneously.
def animate_model(time, time_step, position, velocity, input_torque, L: float):
    x_coords = L * np.sin(position) # x coordinates
    y_coords = -L * np.cos(position) # y coordinates
    o_point = np.array([1, -L])

    fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex='col')

    axes[0, 0].axis('off') 
    axes[2, 0].axis('off')

    # ---------------- X and Y LIMITS ----------------

    # Position Y-Limits
    Y_MIN_POS = np.min(position) - 0.1 * np.abs(np.min(position))
    Y_MAX_POS = np.max(position) + 0.1 * np.abs(np.max(position))
    
    # Velocity Y-Limits
    Y_MIN_VEL = np.min(velocity) - 0.1 * np.abs(np.min(velocity))
    Y_MAX_VEL = np.max(velocity) + 0.1 * np.abs(np.max(velocity))
    
    # Input Torque Y-Limits
    Y_MIN_INPUT = np.min(input_torque) - 0.1 * np.abs(np.min(input_torque))
    Y_MAX_INPUT = np.max(input_torque) + 0.1 * np.abs(np.max(input_torque))

    # Time X-Limits
    X_MIN = time[0]
    X_MAX = time[-1]

    # ---------------- ANIMATION ----------------
    axes[1, 0].set_xlim( -1.5 * L, 1.5 * L )
    axes[1, 0].set_ylim( -1.5 * L, 0.5 * L )
    axes[1, 0].set_aspect('equal', adjustable='box')
    axes[1, 0].set_title('Pendulum')
    axes[1, 0].grid(True)

    line, = axes[1, 0].plot([0, x_coords[0]], [0, y_coords[0]], 'ro-', lw=2) # Init line
    spring_data = generate_spring(o_point, [x_coords[0], y_coords[0]], 10) # generate spring
    spring, = axes[1, 0].plot(spring_data[:, 0], spring_data[:, 1], color='blue', lw=2) # Init spring

    mass_point, = axes[1, 0].plot(x_coords[0], y_coords[0], 'o', color='black', markersize=15, zorder=4) # Point Mass [M]

    # Pendulum Support
    axes[1, 0].hlines(0.01, -0.2, 0.2, colors='black', lw=1, zorder=3) # Horizontal Line
    for i in np.arange(-0.2, 0.2, 0.05): # Dashed Line
        axes[1, 0].plot([i, i + 0.05], [0, 0.05], color='black', lw=1, zorder=3)

    # Spring Support
    y_min_wall = o_point[1] - 0.25
    y_max_wall = o_point[1] + 0.25
    axes[1, 0].vlines(o_point[0], y_min_wall, y_max_wall, colors='black', lw=1, zorder=3) # Vertical Line
    for i in np.arange(y_min_wall, y_max_wall, 0.05): # Dashed Lines
        axes[1, 0].plot([o_point[0], o_point[0] + 0.05], [i, i + 0.05], color='black', lw=1, zorder=3)

    # ---------------- POSITION ----------------
    axes[0, 1].set_ylabel('Angle (rad)', fontsize=12)
    axes[0, 1].grid(True)
    axes[0, 1].set_title('Angle Change in Time')
    axes[0, 1].set_xlim(X_MIN, X_MAX)
    axes[0, 1].set_ylim(Y_MIN_POS, Y_MAX_POS)
    
    pos_line, = axes[0, 1].plot([], [], label='Angle ($\phi$)', color='blue')

    # ---------------- VELOCITY ----------------
    axes[1, 1].set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    axes[1, 1].grid(True)
    axes[1, 1].set_title('Angular Velocity Change in Time')
    axes[1, 1].set_xlim(X_MIN, X_MAX)
    axes[1, 1].set_ylim(Y_MIN_VEL, Y_MAX_VEL)

    vel_line, = axes[1, 1].plot([], [], label='Angular Velocity ($\dot{\phi}$', color='orange')

    # ---------------- INPUT ----------------
    axes[2, 1].set_ylabel('Input (N.m)', fontsize=12)
    axes[2, 1].grid(True)
    axes[2, 1].set_title('Input Torque Change in Time')
    axes[2, 1].set_xlim(X_MIN, X_MAX)
    axes[2, 1].set_ylim(Y_MIN_INPUT, Y_MAX_INPUT)

    input_line, = axes[2, 1].plot([], [], label='Input ($\tau$)', color='red')

    initial_spring_data = spring_data.copy() # copied original spring point cloud to avoid data corruption.

    # This function is iterated in FuncAnimation(...) and updates graphs in every frame.
    def animate(i):
        current_x = x_coords[i]
        current_y = y_coords[i]

        a = (current_y - o_point[1])
        b = (current_x - o_point[0])

        alpha = np.arctan(a / b)
        current_spring_length = np.sqrt(a*a + b*b)
        scale = current_spring_length / o_point[0]

        scaled_spring = scale_around_point(initial_spring_data, o_point, scale, 1.0)
        scaled_and_rotated = rotate_around_point(scaled_spring, o_point, alpha)

        line.set_data([0, current_x], [0, current_y]) # Update pendulum
        mass_point.set_data([current_x], [current_y]) # Update point mass
        spring.set_data(scaled_and_rotated[:, 0], scaled_and_rotated[:, 1]) # Update spring
        pos_line.set_data(time[:i+1], position[:i+1]) # Update position graph
        vel_line.set_data(time[:i+1], velocity[:i+1]) # Update velocity graph
        input_line.set_data(time[:i+1], input_torque[:i+1]) # Update input torque graph

        return line, mass_point, spring, pos_line, vel_line, input_line,

    ani = FuncAnimation(fig, animate, frames=len(time), interval=time_step*1000, blit=True)

    # Uncomment to save the video.
    '''
    # Save Video
    video_filename = 'step.mp4'
    
    print(f"Video is saving ({video_filename}).")
    print(f"Expected FPS: {1/time_step:.1f} FPS")
    
    ani.save(
        video_filename, 
        writer='ffmpeg', 
        fps=int(1/time_step),
        dpi=200 # Increase DPI to increase resolution
    )
    
    print(f"Saved: {video_filename}")
    '''

    plt.show()

if __name__ == "__main__":
    # --------------------- Parameters ---------------------
    m = 0.5
    L = 1.0
    M = 1.5
    k_coeff = 10.0
    b_coeff = 2.0
    gravity = 9.81

    # --------------------- CREATE PENDULUM OBJECT ---------------------
    system = PendulumSystem(m=m, L=L, M=M, k_coeff=k_coeff, b_coeff=b_coeff, gravity=gravity)

    # --------------------- State-Space - Inıtıal State ---------------------
    phi = 0
    phi_dot = 0
    X0 = np.array([[phi],
                [phi_dot]])

    # --------------------- SIMULATION SETTINGS ---------------------
    simulation_time = 10
    time_step = 0.01

    # --------------------- START SIMULATION---------------------
    time, position, velocity, input_torque = system.run_dynamics(simulation_time, time_step, X0)

    animate_model(time, time_step, position, velocity, input_torque, L=system.L)