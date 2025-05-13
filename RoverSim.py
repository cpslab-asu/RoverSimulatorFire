import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import matplotlib.animation as animation
import argparse


class SimulationModule:
    def __init__(self):
        self.maxX = 6
        self.t_s = 0.001
        self.CPV = 6
        self.k = 0  # Placeholder for k value
        self.pos = [20, 10]


    def animate_2d_noflip(self,states):
        fig, ax = plt.subplots()
        ax.set_xlim(-10, 25)
        ax.set_ylim(-25, 10)
        trail_length = 1000
        # Create multiple lines for the trail effect
        trail, = ax.plot([], [], 'b-', lw=1.5, alpha=0.7)  # Thin blue line for path
        # Marker for the moving point
        point, = ax.plot([], [], 'ro', markersize=6)

        x_data, y_data = [], []
        
        def init():
            #for trail in trails:
            trail.set_data([], [])
            point.set_data([], [])
            return trail, point

        def update(frame):
            # Select the range of frames to show as trails

            x = states[0,frame]
            y = states[1,frame]
            x_data.append(x)  # Assuming a single object, use x[:] if multiple
            y_data.append(y)
            trail.set_data(x_data, y_data)
            # Update moving point
            point.set_data(x, y)


            return trail, point

        ani = animation.FuncAnimation(fig, update, frames=len(states[0,:]), init_func=init, interval=5, blit=True)
        plt.show()

    def open_loop_reference(self,t, a, b, r, states, global_turn_right, auto_drive, check_position, omega, update_compass, update_gps, continue_moving, init_position_x, init_position_y, t_s, maxX, v, R, CPV):
        # Provide an open loop command to the wheels' rotation speeds
        velR = v
    
        # Use this to set v_x and omega_z based on the kinematic model
        M = np.array([[velR, velR], [b / (2 * (a ** 2 + b ** 2)), -b / (2 * (a ** 2 + b ** 2))]])
    
        # Call to compute new angle and velocity
        v, omega, global_turn_right, auto_drive, check_position, update_compass, update_gps, continue_moving, init_position_x, init_position_y, maxX = self.compute_new_angle_and_vel(t, states, global_turn_right, auto_drive, check_position, omega, update_compass, update_gps, continue_moving, init_position_x, init_position_y, t_s, maxX)

        # Inverse of matrix M multiplied by velocity and omega
        Vde = np.linalg.inv(M).dot(np.array([v, omega]))
    
        # Calculating Omega_r and Omega_l
        Omega_r = Vde[0] / r
        Omega_l = Vde[1] / r
    
        return Omega_r, Omega_l, global_turn_right, auto_drive, check_position, update_compass, update_gps, continue_moving, omega, init_position_x, init_position_y, maxX


    def compute_new_angle_and_vel(self,t, states, global_turn_right, auto_drive, check_position, omega, update_compass, update_gps, continue_moving, init_position_x, init_position_y, t_s, maxX):
        velR = 0.5
        v = velR
        cur_position_x = states[0]
        cur_position_y = states[1]
        thresh = 7.01
        prev_angle = omega
    
        if global_turn_right:
            turn_angle = 1
        else:
            turn_angle = -1
          
        if auto_drive and check_position:
            dist = np.sqrt((init_position_y - cur_position_y) ** 2 + (init_position_x - cur_position_x) ** 2)
         
            if dist < thresh:
                v = velR
                omega = 0
                return v, omega, global_turn_right, auto_drive, check_position, update_compass, update_gps, continue_moving, init_position_x, init_position_y, maxX
            
            check_position = False
        
            if global_turn_right:
                turn_angle = 1
            else:
                turn_angle = -1
        
            update_compass = 1
    
        elif auto_drive and not check_position and update_compass:
            #print(states.shape)
            cur_angle = states[2]
            v = velR
        
            if turn_angle == 1:
                proper_head = -np.pi / 2
            else:
                proper_head = np.pi / 2
        
            error = self.EMI(states)
            check_cur_angle = cur_angle - error
            print(check_cur_angle)
            if np.abs(proper_head - check_cur_angle) < 0.1 and np.abs(proper_head - check_cur_angle) > 0:
                maxX = states[0]
                omega = 0
                update_compass = 0
                update_gps = 1
            else:
                if proper_head == np.pi / 2:
                    omega = 0.1
                else:
                    omega = -0.1
    
        elif auto_drive and not check_position and not update_compass and update_gps:
            init_position_x = states[0]
            init_position_y = states[1]
            update_gps = 0
            continue_moving = 1
    
        elif auto_drive and not check_position and not update_compass and not update_gps and continue_moving:
            omega = prev_angle
            dist = np.sqrt((init_position_y - cur_position_y) ** 2 + (init_position_x - cur_position_x) ** 2)
        
            if dist < thresh:
                v = velR
                omega = 0
                return v, omega, global_turn_right, auto_drive, check_position, update_compass, update_gps, continue_moving, init_position_x, init_position_y, maxX
            else:
                v = 0
                omega = 0
    
        return v, omega, global_turn_right, auto_drive, check_position, update_compass, update_gps, continue_moving, init_position_x, init_position_y, maxX


    def EMI(self,state):
        pos = np.array(self.pos)
        range_ = 11.4375
        distance = max(np.linalg.norm(pos - state[2]), 0.005)
    
        if distance < range_:
            error = 30*66.14 * (1 + np.random.rand()) * (1 / (distance ** 2))
        else:
            error = 0
    
        return error





    def compute_relative_vel(self, u, c, vx, vy, wz, r):
        # Compute the velocity of the contact point of the wheel with respect to the floor
        Sw = np.array([[0, -wz], [wz, 0]])  # skew-symmetric matrix
    
        # Velocity due to the wheel rotation speed
        vel = np.array([-r * u, 0])
    
        # Add the velocity due to the robot's angular velocity
        vel = vel + Sw @ c
    
        # Add the velocity due to the robot's linear velocity
        vel = vel + np.array([vx, vy])
    
        return vel


    def friction_joined_model(self,CM, m, g, a, b, kf, u_all, epsilon):
        # Initialize the matrix A
        A = np.array([[1, 1, 1, 1],
                      [-b, -b, b, b],
                      [-a, a, a, -a]])

        # Add the modification to A based on CM, kf, u_all, and epsilon
        u_all = u_all.T
        for i in range(4):
            A[1, i] += CM[2] * kf * u_all[1, i] / (np.linalg.norm(u_all[:, i]) + epsilon)
            A[2, i] += CM[2] * kf * u_all[0, i] / (np.linalg.norm(u_all[:, i]) + epsilon)

        # Initialize the matrix B
        B = np.array([m * g, m * g * CM[1], -m * g * CM[0]])
        B = B.T
        #n = np.linalg.inv(A @ A.T) @ A        
        #print(n.shape)
        # Compute the normals
        n = A.T @ np.linalg.inv(A @ A.T) @ B

        # Check if one of the normals is negative
        min_n = np.min(n)
        id_min = np.argmin(n)
    
        if min_n < 0:
            A = np.delete(A, id_min, axis=1)  # Remove the column corresponding to the negative normal
            n3 = np.linalg.inv(A @ A.T) @ A @ B

            n_old = n.copy()
            n = np.ones(4)
            n[id_min] = 0

            for k in range(4):
                if k == id_min:
                    n[k] = 0
                else:
                    n[k] = n3[0]
                    n3 = np.delete(n3, 0)  # Remove the first element from n3

        # Compute forces
        F_1 = -kf * n[0] * u_all[:, 0] / (np.linalg.norm(u_all[:, 0]) + epsilon)
        F_2 = -kf * n[1] * u_all[:, 1] / (np.linalg.norm(u_all[:, 1]) + epsilon)
        F_3 = -kf * n[2] * u_all[:, 2] / (np.linalg.norm(u_all[:, 2]) + epsilon)
        F_4 = -kf * n[3] * u_all[:, 3] / (np.linalg.norm(u_all[:, 3]) + epsilon)

        return F_1, F_2, F_3, F_4, n

    def load_parameters(self, CPV, t_s):

        g = 9.81  # gravity acceleration

        # Select the robot type
        robot = 'EspeleoRobo'
        # robot = 'PioneerP3AT'

        # Robot parameters based on robot type
        if robot == 'EspeleoRobo':
            # Espeleorobo
            a = 0.215  # forward/backward distance of the wheels (from the robot's center)
            b = 0.18  # lateral distance of the wheels (from the robot's center)
            w = 2 * b
            r = 0.151  # radius of the wheels
            epsilon = 0.005  # "velocity of the maximum static friction"
            m = 27.4  # mass
            if CPV == 6 or CPV == 7:
                J = 300 * 0.76  # moment of inertia
            else:
                J = 0.76  # moment of inertia
            kf = 0.48  # coefficient of the kinetic friction (N/N)
            CM = np.array([0.0, 0.0, 0.12])  # Center of mass with respect to the body's center (x, y) and floor
            h = 1.5
            size_body = np.array([0.55, 0.35, 0.12])  # Size of the robot (x, y, z) - only for animation
        
        elif robot == 'PioneerP3AT':
            # Pioneer
            a = 0.135  # forward/backward distance of the wheels (from the robot's center)
            b = 0.2  # lateral distance of the wheels (from the robot's center)
            r = 0.098  # radius of the wheels
            epsilon = 0.005  # "velocity of the maximum static friction"
            J = 0.58  # moment of inertia
            m = 26.8  # mass
            kf = 0.42  # coefficient of the kinetic friction (N)
            CM = np.array([0.0, 0.0, 0.15])  # Center of mass with respect to the body's center (x, y) and floor
            size_body = np.array([0.4, 0.34, 0.18])  # Size of the robot (x, y, z) - only for animation
        
        else:
            raise ValueError('Select the robot')

        # Compute the vectors that go from the robot's center of mass to the wheels
        c1 = np.array([a, -b]) - CM[:2]
        c2 = np.array([-a, -b]) - CM[:2]
        c3 = np.array([-a, b]) - CM[:2]
        c4 = np.array([a, b]) - CM[:2]

        # Simulation times definitions
        T = 100
        dt = t_s
        t = np.arange(0, T + dt, dt)
        t2 = np.linspace(0, 10, 1000)

        # Initial states and wheel orientation
        states = np.zeros((6,len(t2)))  # initial states
        wheels = np.zeros((4,len(t2)))  # initial orientation of the wheels (only for plot)


        # Binary flag to plot simulated data
        DO_PLOTS = 0

        # Flag to run an animation
        # 0 - no animation
        # 1 - 3D animation
        # 2 - 2D animation
        RUN_ANIMATION = 1

        # Simulation speed factor
        SPEED = 10.0

        return states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, DO_PLOTS, RUN_ANIMATION, SPEED

    def run_simulator(self, states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, CPV, DO_PLOTS, RUN_ANIMATION, SPEED):
            # Initialize parameters and other constants
        # parameters
        Omega_1 = 0
        Omega_2 = 0
        Omega_3 = 0
        Omega_4 = 0

        F_last = np.array([0, 0])

        # Initialize logs
        vel_log = np.zeros((4, len(t)))
        F_tot_log = np.zeros((2, len(t)))
        F1_log = np.zeros((2, len(t)))
        F2_log = np.zeros((2, len(t)))
        F3_log = np.zeros((2, len(t)))
        F4_log = np.zeros((2, len(t)))
        TORQUE_1 = np.zeros(len(t))
        TORQUE_2 = np.zeros(len(t))
        TORQUE_3 = np.zeros(len(t))
        TORQUE_4 = np.zeros(len(t))
        POWER_1 = np.zeros(len(t))
        POWER_2 = np.zeros(len(t))
        POWER_3 = np.zeros(len(t))
        POWER_4 = np.zeros(len(t))
        vS = np.zeros(len(t))
        N_log = np.zeros((len(t),4))
 
        # Simulation loop
        t = np.linspace(0, 10, 1000)  # Assuming t is time array, you should define it as per your use case
        #states = np.zeros((6, len(t)))  # Initialize state vector (x, y, psi, vx, vy, wz)
        #wheels = np.zeros()
        # Initialize variables
        global_turn_right = 1
        auto_drive = 1
        check_position = 1
        update_compass = 0
        update_gps = 0
        continue_moving = 0
        prev_omega = 0
        init_position_x = 0
        init_position_y = 0
        omega_roll = 0
        roll_angle = 0
        v = 0.5
        R = 0.5
        omegaA = np.zeros(len(t)-1)
        maxX = self.maxX

        for k in range(len(t)-1):
            # Get current states
            x = states[0, k]
            y = states[1, k]
            psi = states[2, k]
            vx = states[3, k]
            vy = states[4, k]
            wz = states[5, k]

            # Roll dynamics
            Critical_vel = np.sqrt(g * (h - w/2) / (R * h / w))

            # FIXME: Make sure this is correct
            t_s = dt

            # Compute the wheels reference velocities using the open loop reference function
            Omega_r, Omega_l, global_turn_right, auto_drive, check_position, update_compass, update_gps, continue_moving, omega, init_position_x, init_position_y, maxX = self.open_loop_reference(t[k], a, b, r, states[:, k], global_turn_right, auto_drive, check_position, prev_omega, update_compass, update_gps, continue_moving, init_position_x, init_position_y, t_s, maxX, v, R, CPV)

            prev_omega = omega
            omegaA[k] = omega

            # Filter command to simulate a ramp time of the wheels
            ALPHA = dt / (0.5 / 2)
            Omega_1 = (1 - ALPHA) * Omega_1 + ALPHA * Omega_r
            Omega_2 = (1 - ALPHA) * Omega_2 + ALPHA * Omega_r
            Omega_3 = (1 - ALPHA) * Omega_3 + ALPHA * Omega_l
            Omega_4 = (1 - ALPHA) * Omega_4 + ALPHA * Omega_l

            # Compute relative velocities of the contact points of the wheels with respect to the floor
            u1 = self.compute_relative_vel(Omega_1, c1, vx, vy, wz, r)
            u2 = self.compute_relative_vel(Omega_2, c2, vx, vy, wz, r)
            u3 = self.compute_relative_vel(Omega_3, c3, vx, vy, wz, r)
            u4 = self.compute_relative_vel(Omega_4, c4, vx, vy, wz, r)

            # Compute friction forces (using the linearity of friction force)
            u_all = np.array([u1, u2, u3, u4])
            F_1, F_2, F_3, F_4, normals = self.friction_joined_model(CM, m, g, a, b, kf, u_all, epsilon)
            
            # Compute total force
            F_tot = F_1 + F_2 + F_3 + F_4
    
            # Compute the torque (around the center of mass and in the z-axis)
            Tau_1 = c1[0] * F_1[1] - c1[1] * F_1[0]
            Tau_2 = c2[0] * F_2[1] - c2[1] * F_2[0]
            Tau_3 = c3[0] * F_3[1] - c3[1] * F_3[0]
            Tau_4 = c4[0] * F_4[1] - c4[1] * F_4[0]

            # Compute total torque
            Tau_tot = Tau_1 + Tau_2 + Tau_3 + Tau_4

            # Compute the derivative of the states
            states_dot = np.array([
                np.cos(psi) * vx - np.sin(psi) * vy,
                np.sin(psi) * vx + np.cos(psi) * vy,
                wz,
                F_tot[0] / m + wz * vy,
                F_tot[1] / m - wz * vx,
                Tau_tot / J
            ])

            vehicle_speed = np.sqrt(states_dot[0]**2 + states_dot[1]**2)
            vS[k] = vehicle_speed
            if vehicle_speed > Critical_vel:
                print('Vehicle roll')
                break

            # Save current force
            F_last = F_tot

            # System integration
            states[:, k + 1] = states[:, k] + states_dot * dt
    
            # Compute the new position of the wheels (only for animation)
            #print(wheels.shape)
            wheels[:, k + 1] = wheels[:, k] + np.array([Omega_1, Omega_2, Omega_3, Omega_4]) * dt

            # Logs
            vel_log[:, k + 1] = np.array([np.linalg.norm(u1), np.linalg.norm(u2), np.linalg.norm(u3), np.linalg.norm(u4)])
            F_tot_log[:, k + 1] = F_tot
            F1_log[:, k + 1] = F_1
            F2_log[:, k + 1] = F_2
            F3_log[:, k + 1] = F_3
            F4_log[:, k + 1] = F_4
            N_log[k + 1, :] = normals.T

            TORQUE_1[k + 1] = r * np.dot(F_1, np.array([1, 0]))
            TORQUE_2[k + 1] = r * np.dot(F_2, np.array([1, 0]))
            TORQUE_3[k + 1] = r * np.dot(F_3, np.array([1, 0]))
            TORQUE_4[k + 1] = r * np.dot(F_4, np.array([1, 0]))

            POWER_1[k + 1] = TORQUE_1[k + 1] * Omega_1
            POWER_2[k + 1] = TORQUE_2[k + 1] * Omega_2
            POWER_3[k + 1] = TORQUE_3[k + 1] * Omega_3
            POWER_4[k + 1] = TORQUE_4[k + 1] * Omega_4

        # Plotting (same as MATLAB plotting but in Python)
        if DO_PLOTS == 1:
            plt.figure(1)
            plt.subplot(2, 2, 1)
            plt.plot(states[0, :], states[1, :], 'b')
            plt.plot(geo_center[0, :], geo_center[1, :], 'r')
            plt.axis('equal')
            plt.grid(True)
            plt.title('XY (center of mass)')
            plt.legend(['center mass', 'center body'])
    
            plt.subplot(2, 2, 2)
            plt.plot(t, states[2, :], 'b')
            plt.grid(True)
            plt.title('Yaw')

            plt.subplot(2, 2, 3)
            plt.plot(t, states[3, :], 'r')
            plt.plot(t, states[4, :], 'g')
            plt.plot(t, np.sqrt(states[3, :]**2 + states[4, :]**2), 'b')
            plt.grid(True)
            plt.title('Body velocities')
            plt.legend(['v_x', 'v_y', 'norm'])
    
            plt.subplot(2, 2, 4)
            plt.plot(t, states[5, :], 'b')
            plt.grid(True)
            plt.title('ω_z')

            plt.figure(3)
            plt.subplot(3, 1, 1)
            plt.plot(t, vel_log[0, :], 'r')
            plt.plot(t, vel_log[1, :], 'g')
            plt.plot(t, vel_log[2, :], 'b')
            plt.plot(t, vel_log[3, :], 'k')
            plt.legend(['w1', 'w2', 'w3', 'w4'])
            plt.title('Wheels velocities with respect to the floor')
            plt.grid(True)
    
            plt.subplot(3, 1, 2)
            plt.plot(t, TORQUE_1, 'r')
            plt.plot(t, TORQUE_2, 'g')
            plt.plot(t, TORQUE_3, 'b')
            plt.plot(t, TORQUE_4, 'k')
            plt.title('Torque on the wheels')
            plt.legend(['p1', 'p2', 'p3', 'p4'])
            plt.grid(True)

            plt.subplot(3, 1, 3)
            plt.plot(t, POWER_1, 'r')
            plt.plot(t, POWER_2, 'g')
            plt.plot(t, POWER_3, 'b')
            plt.plot(t, POWER_4, 'k')
            plt.title('Power on the wheels')
            plt.legend(['p1', 'p2', 'p3', 'p4'])
            plt.grid(True)

            plt.show()

        return states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, DO_PLOTS, RUN_ANIMATION, SPEED

    def run_animation(self, run_animation_mode):
        if run_animation_mode == 1:
            self.animation_3d_noflip()
        elif run_animation_mode == 2:
            self.animation_2d()

    def animation_3d_noflip(self):
        # 3D animation logic (Placeholder function)
        print("Running 3D animation (no flip)")

    def animation_2d(self):
        # 2D animation logic (Placeholder function)
        print("Running 2D animation")

if __name__ == "__main__":


    parser = argparse.ArgumentParser(
        description="Run SimulationModule; provide starting pos as --pos X Y"
    )
    parser.add_argument(
        "--pos",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        default=[20.0, 10.0],
        help="reference position for EMI (two floats: X Y)"
    )
    parser.add_argument(
        "--CPV",
        type=int,
        default=6,
        help="CPV value"
    )
    parser.add_argument(
        "--t_s",
        type=float,
        default=0.1,
        help="time step"
    )
    args = parser.parse_args()
    sim = SimulationModule()
    CPV = 6  # or any appropriate value
    t_s = 0.1  # time step, for example
    sim.pos = args.pos
    states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, DO_PLOTS, RUN_ANIMATION, SPEED = sim.load_parameters(CPV, t_s)
    
    states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, DO_PLOTS, RUN_ANIMATION, SPEED = sim.run_simulator(states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, CPV, DO_PLOTS, RUN_ANIMATION, SPEED)
    #RUN_ANIMATION = 1  # Set animation mode
    #print(states[0,:])
    sim.animate_2d_noflip(states)
