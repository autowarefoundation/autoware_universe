import numpy as np
import matplotlib.pyplot as plt
import argparse

track_radius_outer = 2 + .125
track_radius_inner = 2 - .125

# Draw the bounds for the track
theta = np.linspace(0,2*np.pi, 1000)
x_track_inner = track_radius_inner*np.cos(theta)
y_track_inner = track_radius_inner*np.sin(theta)
x_track_outer = track_radius_outer*np.cos(theta)
y_track_outer = track_radius_outer*np.sin(theta)




def plot_trajectories(axis, axis_name, trajectory_array):
    axis.set_title(axis_name)
    time_horizon, trajectory_length, state_dim = trajectory_array.shape
    i = 0
    speed_vec = np.zeros((time_horizon))
    for i in range(time_horizon):
        if (trajectory_array[i,0,:] == 0).all():  # Means that the system stopped saving trajectories because of task failure
            break
        axis.plot(trajectory_array[i,:,0], trajectory_array[i,:,1], 'b', linewidth=0.5, alpha=0.7)

        # Compute the speed
        speed_vec[i] = np.linalg.norm(trajectory_array[i,0,2:])
        # Check for tube failure

        axis.plot(trajectory_array[:i,0,0], trajectory_array[:i,0,1], 'y', linewidth=2, alpha=1.0)
    average_speed = np.mean(speed_vec)
    print(axis_name)
    print(average_speed)
    axis.plot(x_track_inner, y_track_inner, 'r', linewidth=2)
    axis.plot(x_track_outer, y_track_outer, 'r', linewidth=2)


def plot_trajectories2(axis, axis_name, trajectory_array):
    axis.set_title(axis_name)
    time_horizon, state_dim = trajectory_array.shape
    i = 0
    speed_vec = np.zeros((time_horizon))
    for i in range(time_horizon):


        # Compute the speed
        speed_vec[i] = np.linalg.norm(trajectory_array[i,2:])
        # Check for tube failure

        axis.plot(trajectory_array[:i,0], trajectory_array[:i,1], 'y', linewidth=2, alpha=1.0)
    average_speed = np.mean(speed_vec)
    print(axis_name)
    print(average_speed)
    axis.plot(x_track_inner, y_track_inner, 'r', linewidth=2)
    axis.plot(x_track_outer, y_track_outer, 'r', linewidth=2)



def main(args):
    build_dir = args['build_dir']
    data_dir = build_dir + 'tests/controllers/'
    CCM_true = args['CCM']
    # Create the figures for each subplot
    fig, axs = plt.subplots(2,2)

    for ax in axs.flat:
        ax.set(xlabel='X Position (m)', ylabel='Y Position (m)')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for ax in axs.flat:
        ax.label_outer()

    # # Plot the vanilla trajectory
    # vanilla_nominal = np.load(data_dir + 'vanilla_nominal.npy')
    # plot_trajectories(axs[0,0], 'Vanilla MPPI Nominal Disturbance', vanilla_nominal)
    #
    # # Vanilla MPPI with large noise
    # vanilla_large = np.load(data_dir + 'vanilla_large_track_feedback.npy')
    # plot_trajectories(axs[0,1], 'Vanilla MPPI Tracking Large Disturbance', vanilla_large)
    #
    # # Vanilla MPPI with large noise + a tracking controller
    # # vanilla_large_track = np.load(data_dir + 'vanilla_large_track.npy')
    # # plot_trajectories(axs[1,0], 'Vanilla MPPI Large Disturbance + Tracking', vanilla_large_track)
    #
    # # Tube MPPI with large noise
    # tube_large = np.load(data_dir + 'tube_large_actual.npy')
    # plot_trajectories(axs[1,0], 'Tube MPPI Large Disturbance', tube_large)
    #
    # # RMPPI with large noise
    # # Create the figures for each subplot
    # # fig2, axs2 = plt.subplots(2,2)
    # #
    # # for ax in axs2.flat:
    # #     ax.set(xlabel='X Position (m)', ylabel='Y Position (m)')
    # robust_large = np.load(data_dir + 'robust_large_actual.npy')
    # plot_trajectories(axs[1,1], 'Robust LQR MPPI Actual', robust_large)

    if (CCM_true):
        fig, axs = plt.subplots(2,2)

        for ax in axs.flat:
            ax.set(xlabel='X Position (m)', ylabel='Y Position (m)')
        # robust_ccm = np.load(data_dir + 'robust_large_actual_CCM_t_4950.npy')
        # plot_trajectories(axs[0,0], 'Robust CCM MPPI Actual', robust_ccm)
        #
        # robust_ccm = np.load(data_dir + 'robust_large_nominal_CCM_t_4950.npy')
        # plot_trajectories(axs[0,1], 'Robust CCM MPPI Nominal', robust_ccm)

        robust_ccm = np.load(data_dir + 'robust_large_actual_traj_CCM_t_2999.npy')
        plot_trajectories2(axs[0,0], 'Robust CCM MPPI Actual', robust_ccm)

        robust_ccm = np.load(data_dir + 'robust_large_nominal_traj_CCM_t_2999.npy')
        plot_trajectories(axs[0,1], 'Robust CCM MPPI Nominal', robust_ccm)

        robust_large = np.load(data_dir + 'robust_large_actual.npy')
        plot_trajectories(axs[1,0], 'Robust LQR MPPI Actual', robust_large)

        robust_large = np.load(data_dir + 'robust_large_nominal.npy')
        plot_trajectories(axs[1,1], 'Robust LQR MPPI Nominal', robust_large)

    # robust_large = np.load(data_dir + 'robust_ancillary.npy')
    # plot_trajectories(axs2[1,0], 'Robust MPPI Ancillary', robust_large)


# # Tube MPPI ancillary trajectories
    # tube_ancillary = np.load(data_dir + 'tube_ancillary.npy')
    # plot_trajectories(axs[1,1], 'Tube MPPI Ancillary Solution', tube_ancillary)

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'Say hello')
    parser.add_argument('--build_dir', help='Location of MPPI-Generic build folder', required=True)
    parser.add_argument('--CCM', help="Plot the CCM stuff?", required=False, default=0)
    args = vars(parser.parse_args())

    main(args)
