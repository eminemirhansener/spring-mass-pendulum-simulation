import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from itertools import product
import pandas as pd 

# --- Pendulum System Action Space ---
# Action space definition: Torque (tau) values
tau_values = np.arange(-10, 10.2, 0.2) 
num_actions = len(tau_values)
U_MAX = 10.0 # Maximum torque

# === Pendulum System Model (Pendulum) Environment ===
class PendulumSystemEnv:
    def __init__(self, m=0.5, L=1.0, M=1.5, k_coeff=10.0, b_coeff=2.0, dt=0.02, 
                 angle_threshold=np.pi/3, max_episode_steps=500, ref=np.pi/12, gravity=9.81):
        
        self.m = m # Pendulum Rod Mass
        self.L = L # Pendulum Rod Length
        self.M = M # End Mass
        self.k_coeff = k_coeff # Spring Constant
        self.b_coeff = b_coeff # Damping Coefficient
        self.gravity = gravity
        self.dt = dt
        
        self.angle_threshold = angle_threshold # Threshold for balancing (e.g., ±60 degrees)
        self.max_episode_steps = max_episode_steps
        self.ref = ref # Reference angle (0.0 radians for balancing)
        
        self.state = None # [phi (angle), phi_dot (angular velocity)]
        self.step_count = 0
        self.last_tau = 0.0 

        # RL Reward Function Weights (Adapted from MCK)
        self.lam_tau = 0.005 # Control input (torque) weight
        self.lam_dtau = 0.25 # Control input derivative (torque change) weight
        
        # Calculate State Space Matrices (A and B)
        self.A, self.B = self._calculate_state_matrices()

    def _calculate_state_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        # Linearized model calculation 
        denominator = ((1/3) * self.m + self.M) * self.L**2
        if denominator == 0:
            raise ValueError("Denominator is zero. Invalid physical parameters.")
        
        a11, a12 = 0, 1
        a21 = -((self.gravity * self.L * ((self.m / 2) + self.M) + self.k_coeff * self.L**2) / denominator)
        a22 = -(self.b_coeff / denominator)

        A = np.array([[a11, a12], [a21, a22]])
        B = np.array([[0], [1 / denominator]])

        return A, B

    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
            
        # Initial state: Small random perturbations (e.g., ±5 degrees)
        initial_angle = np.random.uniform(low=-np.pi/36, high=np.pi/36) # ±5 degrees
        initial_velocity = np.random.uniform(low=-0.05, high=0.05)
        self.state = np.array([initial_angle, initial_velocity])
        
        self.step_count = 0
        self.last_tau = 0.0
        return self.state.copy()

    def step(self, action):
        if isinstance(action, np.ndarray):
            action = int(action)
        
        tau = tau_values[action] # Torque value (Control Input)
        tau = np.clip(tau, -U_MAX, U_MAX)

        # State Space Iteration (Forward Euler Method)
        # X = [phi, phi_dot]
        X = self.state.reshape(-1, 1) # Make state a column vector
        U = np.array([[tau]]) # Control Input Vector

        X_dot = self.A @ X + self.B @ U 
        X_next = X + X_dot * self.dt 
        
        self.state = X_next.flatten() # Flatten state back to a horizontal array
        self.step_count += 1
        
        phi, phi_dot = self.state

        # Boundary Check (Balancing: DONE if angle exceeds threshold)
        done = bool(
            abs(phi) > self.angle_threshold or self.step_count >= self.max_episode_steps
        )

        # Reward (Cost Function)
        error = phi - self.ref
        u = tau
        delta_u = u - self.last_tau
        
        # J = - (10 * |error| + lam_tau * torque^2 + lam_dtau * (torque_change)^2)
        # Goal: Minimize error, penalize torque usage and torque change
        reward = -20.0*abs(error) - self.lam_tau * (u**2) - self.lam_dtau * (delta_u**2)

        # Large penalty if balancing fails
        if done and abs(phi) > self.angle_threshold:
            reward -= 100 

        self.last_tau = u
        info = {"ref": self.ref, "u":u, "delta_u":delta_u, "phi_dot": phi_dot}
        return self.state.copy(), reward, done, info
    
# === Q-Learning Agent ===
class FastPendulumControl:
    def __init__(self, alpha, gamma, epsilon, epsilon_decay=0.999, min_epsilon=0.01):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay 
        self.min_epsilon = min_epsilon 

        # State space discretization intervals (Adjusted for Pendulum)
        # Angle Bins: [-pi/3, pi/3] ≈ [-1.047, 1.047] -> 50 bins
        self.phi_bins = np.linspace(-np.pi/3, np.pi/3, 50) 
        # Angular Velocity Bins: Approx. [-3.0, 3.0] for balancing -> 50 bins
        self.phi_dot_bins = np.linspace(-3.0, 3.0, 50) 

        # Initialize Q-table
        self.q_table = np.zeros((
            len(self.phi_bins)+1,
            len(self.phi_dot_bins)+1,
            num_actions
        ))

    def _get_state_idx(self, state):
        phi, phi_dot = state
        pb = int(np.digitize([phi], self.phi_bins)[0])
        vb = int(np.digitize([phi_dot], self.phi_dot_bins)[0])
        return (pb, vb)

    def learn(self, env, num_episodes):
        self.epsilon = float(self.epsilon)
        rewards_history = []
        episode_lengths = []
        
        for episode in range(num_episodes):
            state = env.reset()
            idx = self._get_state_idx(state)
            done = False
            episode_reward = 0.0
            steps = 0

            while not done:
                # Epsilon-Greedy Strategy
                if np.random.rand() < self.epsilon:
                    action = np.random.choice(num_actions) 
                else:
                    action = int(np.argmax(self.q_table[idx])) 
                    
                next_state, reward, done, _ = env.step(action)
                idx_next = self._get_state_idx(next_state)
                
                q = self.q_table[(*idx, action)]
                q_max_next = np.max(self.q_table[idx_next])
                
                # Q-Learning Update Rule
                self.q_table[(*idx, action)] += self.alpha * (reward + self.gamma * q_max_next - q)
                
                idx = idx_next 
                episode_reward += reward
                steps += 1
                
            rewards_history.append(episode_reward)
            episode_lengths.append(steps)
            
            self.epsilon = max(self.epsilon * self.epsilon_decay, self.min_epsilon)
            
            if (episode+1) % 500 == 0 or episode == 0:
                print(f"Episode {episode+1}/{num_episodes}   Reward: {episode_reward:.2f}   Epsilon: {self.epsilon:.4f}")

        return rewards_history, episode_lengths

    def test(self, env, num_episodes):
        rewards_list = []
        for episode in range(num_episodes):
            state = env.reset()
            idx = self._get_state_idx(state)
            done = False
            rewards = 0.0
            while not done:
                # Test mode: Always choose the best action
                action = int(np.argmax(self.q_table[idx]))
                next_state, reward, done, _ = env.step(action)
                idx = self._get_state_idx(next_state)
                rewards += reward
                
            rewards_list.append(rewards)
            
        mean_reward = np.mean(rewards_list)
        print(f"Test episodes: Mean Reward: {mean_reward:.2f}")
        return mean_reward

# === Helper Functions (Save/Load) ===
def save_agent(agent, filename = "best_qtable.npy"):
    """Saves the agent's Q-table."""
    try:
        np.save(filename, agent.q_table)
        print(f"Q-table successfully saved as '{filename}'.")
    except Exception as e:
        print(f"ERROR: Q-table save failed. {e}")

def load_agent(agent, filename="best_qtable.npy"):
    """Loads Q-table from file."""
    try:
        agent.q_table = np.load(filename)
        print(f"Q-table successfully loaded from '{filename}'.")
    except FileNotFoundError:
        print(f"ERROR: File '{filename}' not found.")
        raise
    except Exception as e:
        print(f"ERROR: Q-table load failed. {e}")
        raise

# === Control Analysis Metrics ===
def analyze_trial(phi_data, ref, dt, tolerance=0.05):
    """Analyzes trial results: Steady-State Error, Settling Time, Overshoot."""
    phi_data = np.array(phi_data)
    t_data = np.arange(len(phi_data)) * dt
    
    # 1. Steady-State Error - Average absolute error in the last 100 steps
    if len(phi_data) > 100:
        steady_state_error = np.mean(np.abs(phi_data[-100:] - ref))
    else:
        steady_state_error = np.mean(np.abs(phi_data - ref))
        
    # 2. Maximum Overshoot - Maximum deviation above the reference
    overshoot_data = phi_data[phi_data > ref]
    max_overshoot = np.max(overshoot_data) - ref if overshoot_data.size > 0 else 0.0

    # 3. Settling Time - Time when the system enters and stays within tolerance
    settled = np.abs(phi_data - ref) < tolerance 
    settling_time = float('inf')

    # Requires the last 10% of steps to be settled
    required_settle_steps = int(len(settled) * 0.1) 
    
    for i in range(len(settled) - required_settle_steps):
        if np.all(settled[i:]):
            settling_time = t_data[i]
            break

    return {
        'steady_state_error': steady_state_error,
        'settling_time': settling_time,
        'max_overshoot': max_overshoot
    }

# === Learning Curve Plotting ===
def plot_learning_metrics(rewards_history, episode_lengths, window_size=50):
    """
    Plots training metrics (Reward and Episode Length).
    """
    plt.figure(figsize=(14, 5))
    
    # --- 1. Episode Rewards ---
    plt.subplot(1, 2, 1)
    df_rewards = pd.Series(rewards_history)
    smooth_rewards = df_rewards.rolling(window=window_size, min_periods=1).mean()
    plt.plot(df_rewards, alpha=0.3, label='Raw Reward')
    plt.plot(smooth_rewards, color='blue', label=f'{window_size}-Episode Moving Avg.')
    plt.title('Episode Total Reward Over Training')
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.grid(True)
    plt.legend()

    # --- 2. Episode Lengths ---
    plt.subplot(1, 2, 2)
    df_lengths = pd.Series(episode_lengths)
    smooth_lengths = df_lengths.rolling(window=window_size, min_periods=1).mean()
    plt.plot(df_lengths, alpha=0.3, label='Raw Length')
    plt.plot(smooth_lengths, color='green', label=f'{window_size}-Episode Moving Avg.')
    plt.title('Episode Length Over Training')
    plt.xlabel('Episode')
    plt.ylabel('Step Count (Length)')
    plt.grid(True)
    plt.legend()
    
    plt.suptitle('Training Performance Metrics: Reward and Episode Length Curves', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# -------------------------------------------------------------------------------------------
# --- Animation Helper Functions ---
# -------------------------------------------------------------------------------------------

def generate_spring(fixed, moving, n):
    """Generates a spring horizontally."""
    spring_x_data = np.linspace(fixed[0], moving[0], n+14)
    height = spring_x_data[1] - spring_x_data[0]
    spring_y_data = np.array([fixed[1] - height, fixed[1], fixed[1] + height])

    points = []
    points.append([spring_x_data[0], fixed[1]]) 
    points.append([spring_x_data[6], fixed[1]]) 

    for i in range(7, n+7):
        y_index = (i + 1) % 2 * 2 
        points.append([spring_x_data[i], spring_y_data[y_index]])

    points.append([spring_x_data[n + 7], moving[1]]) 
    points.append([spring_x_data[n + 13], moving[1]]) 

    spring_data = np.array(points)
    return spring_data

def rotate_around_point(points, center, angle_rad):
    """Rotates the point cloud around a specified center."""
    R = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])
    shifted = points - center
    rotated = shifted @ R.T
    return rotated + center

def scale_around_point(points, center, scale_x = 1.0, scale_y = 1.0):
    """Scales the point cloud around a specified center."""
    S = np.array([
        [scale_x, 0],
        [0, scale_y]
    ])
    shifted = points - center
    scaled = shifted @ S.T
    return scaled + center

# --- Control Analysis and Animation Function ---
def animate_pendulum(agent, env, max_steps=500, max_trials=1, save_video=True, video_filename="pendulum_rl_control.mp4"):
    """
    Plots Control Response Analysis (Angle, Torque, Phase Plane) and Animation.
    MP4 saving capability is included.
    """
    for trial in range(max_trials):
        state = env.reset()
        states = [state.copy()]
        torques = []
        done = False
        steps = 0
        
        while not done and steps < max_steps:
            idx = agent._get_state_idx(state)
            action = int(np.argmax(agent.q_table[idx]))
            tau = tau_values[action]
            
            torques.append(tau)
            
            next_state, reward, done, info = env.step(action)
            state = next_state
            states.append(state.copy())
            steps += 1
            
        phi_data = [s[0] for s in states] # Angle (phi)
        phi_dot_data = [s[1] for s in states] # Angular Velocity (phi_dot)
        ref = info.get("ref", 0.0)
        dt = env.dt
        t_data = np.arange(len(phi_data)) * dt 

        metrics = analyze_trial(phi_data, ref, dt)
        
        print(f"\n--- Trial {trial+1} Results ---")
        print(f"Total Steps: {steps} (Final Phi = {state[0]:.4f} rad)")
        print(f"Steady-State Error (Absolute): {metrics['steady_state_error']:.5f} rad")
        print(f"Maximum Overshoot: {metrics['max_overshoot']:.5f} rad")
        print(f"Settling Time (<0.05 rad tolerance): {metrics['settling_time']:.2f} s")
        
        # --- Visualization: Angle, Torque, Phase Plane ---
        L = env.L
        x_coords = L * np.sin(np.array(phi_data)) # x coordinates
        y_coords = -L * np.cos(np.array(phi_data)) # y coordinates
        o_point = np.array([1, y_coords[0]]) # Spring attachment point

        fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharex='col')
        fig.subplots_adjust(hspace=0.33)

        axes[0, 0].axis('off') 
        axes[2, 0].axis('off')
        
        # --- Axis Limits ---
        Y_MIN_POS = np.min(phi_data) - 0.1 * np.abs(np.min(phi_data)) if np.min(phi_data) < 0 else -0.1
        Y_MAX_POS = np.max(phi_data) + 0.1 * np.abs(np.max(phi_data)) if np.max(phi_data) > 0 else 0.1
        Y_MIN_VEL = np.min(phi_dot_data) - 0.1 * np.abs(np.min(phi_dot_data))
        Y_MAX_VEL = np.max(phi_dot_data) + 0.1 * np.abs(np.max(phi_dot_data))
        Y_MIN_INPUT = np.min(torques) - 1.0
        Y_MAX_INPUT = np.max(torques) + 1.0
        X_MIN = t_data[0]
        X_MAX = t_data[-1]

        # ---------------- ANIMATION (Left Middle) ----------------
        axes[1, 0].set_xlim( -1.5 * L, 1.5 * L )
        axes[1, 0].set_ylim( -1.5 * L, 0.5 * L )
        axes[1, 0].set_aspect('equal', adjustable='box')
        axes[1, 0].set_title(f'Pendulum RL Control (Trial {trial+1})')
        axes[1, 0].grid(True)
        
        initial_a = y_coords[0] - o_point[1]
        initial_b = x_coords[0] - o_point[0]
        initial_alpha = np.arctan2(initial_a, initial_b)
        initial_length = np.sqrt(initial_a**2 + initial_b**2)
        line, = axes[1, 0].plot([0, x_coords[0]], [0, y_coords[0]], 'ro-', lw=2) 
        spring_data = generate_spring(o_point, [o_point[0] + initial_length, o_point[1]], 10)
        initial_spring_data = rotate_around_point(spring_data, o_point, initial_alpha)
        spring, = axes[1, 0].plot(initial_spring_data[:, 0], initial_spring_data[:, 1], color='blue', lw=2) 
        mass_point, = axes[1, 0].plot(x_coords[0], y_coords[0], 'o', color='black', markersize=15, zorder=4) 

        # Pendulum Support
        axes[1, 0].hlines(0.01, -0.2, 0.2, colors='black', lw=1, zorder=3)
        for i in np.arange(-0.2, 0.2, 0.05): axes[1, 0].plot([i, i + 0.05], [0, 0.05], color='black', lw=1, zorder=3)
        # Spring Support
        y_min_wall = o_point[1] - 0.25
        y_max_wall = o_point[1] + 0.25
        axes[1, 0].vlines(o_point[0], y_min_wall, y_max_wall, colors='black', lw=1, zorder=3)
        for i in np.arange(y_min_wall, y_max_wall, 0.05): axes[1, 0].plot([o_point[0], o_point[0] + 0.05], [i, i + 0.05], color='black', lw=1, zorder=3)

        # ---------------- ANGLE (Right Top) ----------------
        axes[0, 1].set_ylabel('Angle ($\phi$ rad)', fontsize=12)
        axes[0, 1].grid(True)
        axes[0, 1].set_title('Angle Change')
        axes[0, 1].set_xlim(X_MIN, X_MAX)
        axes[0, 1].set_ylim(Y_MIN_POS, Y_MAX_POS)
        axes[0, 1].axhline(ref, color='r', linestyle='--', label='Reference')
        pos_line, = axes[0, 1].plot([], [], label='Angle ($\phi$)', color='blue')

        # ---------------- ANGULAR VELOCITY (Right Middle) ----------------
        axes[1, 1].set_ylabel('Angular Velocity ($\dot{\phi}$ rad/s)', fontsize=12)
        axes[1, 1].grid(True)
        axes[1, 1].set_title('Angular Velocity Change')
        axes[1, 1].set_xlim(X_MIN, X_MAX)
        axes[1, 1].set_ylim(Y_MIN_VEL, Y_MAX_VEL)
        vel_line, = axes[1, 1].plot([], [], label='Angular Velocity ($\dot{\phi}$', color='orange')

        # ---------------- TORQUE (Right Bottom) ----------------
        axes[2, 1].set_ylabel('Torque (N.m)', fontsize=12)
        axes[2, 1].grid(True)
        axes[2, 1].set_title('Control Input (Torque)')
        axes[2, 1].set_xlim(X_MIN, X_MAX)
        axes[2, 1].set_ylim(Y_MIN_INPUT, Y_MAX_INPUT)
        axes[2, 1].set_xlabel('Time (s)', fontsize=12)
        input_line, = axes[2, 1].step(t_data[:-1], torques, where='post', label='Torque ($\tau$)', color='red')
        
        # ---------------- PHASE PLANE (Left Bottom) ----------------
        axes[2, 0].plot(phi_data, phi_dot_data, 'g-')
        axes[2, 0].plot(phi_data[0], phi_dot_data[0], 'ko', label='Start')
        axes[2, 0].plot(phi_data[-1], phi_dot_data[-1], 'ro', label='End')
        axes[2, 0].plot(ref, 0, 'rx', markersize=10, label='Target State (0, 0)')
        axes[2, 0].set_title('Phase Plane ($\dot{\phi}$ vs $\phi$)')
        axes[2, 0].set_xlabel('Angle ($\phi$ rad)', fontsize=12)
        axes[2, 0].set_ylabel('Angular Velocity ($\dot{\phi}$ rad/s)', fontsize=12)
        axes[2, 0].grid(True)
        axes[2, 0].legend()


        initial_spring_data = spring_data.copy() 
        dx_initial = x_coords[0] - o_point[0]
        dy_initial = y_coords[0] - o_point[1]
        initial_length = np.sqrt(dx_initial**2 + dy_initial**2)

        def animate(i):
            current_x = x_coords[i]
            current_y = y_coords[i]

            a = (current_y - o_point[1])
            b = (current_x - o_point[0])

            alpha = np.arctan2(a, b)
            current_spring_length = np.sqrt(a*a + b*b)
            scale = current_spring_length / initial_length

            # Spring scaling and rotation
            scaled_spring = scale_around_point(initial_spring_data, o_point, scale, 1.0)
            scaled_and_rotated = rotate_around_point(scaled_spring, o_point, alpha)

            line.set_data([0, current_x], [0, current_y]) 
            mass_point.set_data([current_x], [current_y]) 
            spring.set_data(scaled_and_rotated[:, 0], scaled_and_rotated[:, 1]) 
            
            # Update plots
            pos_line.set_data(t_data[:i+1], phi_data[:i+1]) 
            vel_line.set_data(t_data[:i+1], phi_dot_data[:i+1]) 
            input_line.set_data(t_data[:i+1], torques[:i+1]) 

            return line, mass_point, spring, pos_line, vel_line, input_line,

        # Create animation
        ani = animation.FuncAnimation(fig, animate, frames=len(t_data)-1, 
                                      interval=dt*1000, blit=True, repeat=False)
        
        # --- MP4 SAVING FUNCTIONALITY ---
        if save_video:
            full_filename = f"trial_{trial+1}_{video_filename}"
            print(f"\nSaving video ({full_filename})...")
            
            try:
                # Video saving settings
                ani.save(
                    full_filename, 
                    writer='ffmpeg', 
                    fps=int(1/dt),
                    dpi=150 
                )
                print(f"Successfully saved: {full_filename}")
            except Exception as e:
                print(f"ERROR: Video saving failed. FFmpeg might not be installed. Error: {e}")


        plt.show()

# === Main Execution Flow (Hyperparameter Search and Training) ===
def hyperparameter_search():
    # Pendulum System is used as the environment
    env = PendulumSystemEnv(max_episode_steps=500, ref=np.pi/12, angle_threshold=np.pi/3) 
    
    # Hyperparameter grid (Example: 4 combinations for quick run)
    alphas = [0.15, 0.20] 
    gammas = [0.95, 0.99]
    epsilons = [1.0]
    epsilon_decays = [0.995 ,0.999]
    min_epsilons = [0.01]
    
    num_episodes = 2000
    num_test_episodes = 3

    results = []
    param_grid = list(product(alphas, gammas, epsilons, epsilon_decays, min_epsilons))
    print(f"{len(param_grid)} combinations are being tested...")

    # Search
    for idx, (alpha, gamma, epsilon, epsilon_decay, min_epsilon) in enumerate(param_grid):
        agent = FastPendulumControl(alpha, gamma, epsilon, epsilon_decay, min_epsilon)
        
        print(f"\n[{idx+1}/{len(param_grid)}] alpha={alpha}, gamma={gamma}, epsilon={epsilon}, "
              f"epsilon_decay={epsilon_decay}, min_epsilon={min_epsilon}")
              
        agent.learn(env, num_episodes=num_episodes)
        mean_reward = agent.test(env, num_test_episodes)
        
        results.append({
            'params': (alpha, gamma, epsilon, epsilon_decay, min_epsilon),
            'mean_reward': mean_reward,
            'agent': agent
        })

    # Find the best agent
    best = max(results, key=lambda x: x['mean_reward'])
    
    print("\n=== BEST PARAMETERS ===")
    print(f"alpha={best['params'][0]}, gamma={best['params'][1]}, epsilon={best['params'][2]}, "
          f"epsilon_decay={best['params'][3]}, min_epsilon={best['params'][4]}")
    print(f"Mean Test Reward: {best['mean_reward']:.2f}")

    save_agent(best['agent'], "best_pendulum_qtable.npy")
    
    # --- EXTENDED TRAINING (10,000 EPISODES) AND ANALYSIS ---
    print("\n=== STARTING EXTENDED TRAINING (10,000 EPISODES) WITH THE BEST AGENT ===")
    best_params = best['params']
    
    # Initialize a new agent
    agent = FastPendulumControl(*best_params)
    
    total_episodes = 10000
    rewards_history, episode_lengths = agent.learn(env, num_episodes=total_episodes)
    
    save_agent(agent, "best_pendulum_qtable_10k.npy")
    
    # CRITERIA 1 & 2: Learning Curve Visualization (Training and Test Graphs)
    print("\n[CRITERIA 1 & 2] Plotting Learning Curves (Reward and Episode Length)...")
    plot_learning_metrics(rewards_history, episode_lengths, window_size=200)
    
    print("\nTest after 10,000 Episodes:")
    agent.test(env, num_episodes=5)
    
    # CRITERIA 3: Control Analysis (Response Graphs, Phase Plane, and Animation)
    print("\n[CRITERIA 3] Control Analysis (Response Graphs and Phase Plane) and Animation (Including MP4 Save):")
    animate_pendulum(agent, env, max_steps=500, max_trials=2, save_video=True) 

# Standard entry point
if __name__ == "__main__":
    hyperparameter_search()