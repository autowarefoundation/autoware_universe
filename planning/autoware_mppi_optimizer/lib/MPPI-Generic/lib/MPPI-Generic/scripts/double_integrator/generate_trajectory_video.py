import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import numpy as np
from matplotlib import rc
import argparse

controller_dict = {'v': 'vanilla_large_',
                   'vr': 'vanilla_large_robust_',
                   't': 'tube_',
                   'tr': 'tube_robust_',
                   'rs': 'robust_sc_',
                   'rr': 'robust_rc_',
                   'cc':  'robust_cc_',
                   'vr': 'vanilla_large_robust_',
                   'tr': 'tube_robust_'}

title_dict = {'v': 'MPPI Standard Cost',
              't': 'Tube-MPPI Standard Cost',
              'vr': 'MPPI Robust Cost',
              'tr': 'Tube-MPPI Robust Cost',
              'rs': 'RMPPI LQR Standard Cost',
              'rr': 'RMPPI LQR Robust Cost',
              'cc': 'RMPPI CCM Robust Cost'}

rc('font', **{'size': 30})


track_radius_outer = 2 + .125
track_radius_inner = 2 - .125

# Draw the bounds for the track
theta = np.linspace(0,2*np.pi, 1000)
x_track_inner = track_radius_inner*np.cos(theta)
y_track_inner = track_radius_inner*np.sin(theta)
x_track_outer = track_radius_outer*np.cos(theta)
y_track_outer = track_radius_outer*np.sin(theta)

fig, ax = plt.subplots()
fig.set_dpi(100)
fig.set_size_inches(10, 10)
xdata, ydata = [], []
xndata, yndata = [], []

ax.plot(x_track_outer, y_track_outer, 'k', linewidth=2)
ax.plot(x_track_inner, y_track_inner, 'k', linewidth=2)
ax.axis('equal')
ln3, = ax.plot([],[], 'g', alpha=0.7, linewidth=2, label='Nominal Trajectory')
ln2, = ax.plot([], [], 'ro', alpha=0.5, label='Nominal State')
ln1, = ax.plot([], [], 'bo', alpha=0.5, label='Actual State')
ax.set_ylabel('Y Pos (m)')
ax.set_xlabel('X Pos (m)')
title = ax.text(0.5,0.5, "", bbox={'facecolor':'w', 'alpha':0.5, 'pad':20},
                transform=ax.transAxes, ha="center", fontsize=30)


def init():
    ax.set_xlim(-2.25, 2.25)
    ax.set_ylim(-2.25, 2.25)
    return ln1, ln2, ln3


def update(frame):
    xndata.append(nominal_state[frame, 0])
    yndata.append(nominal_state[frame, 1])
    xdata.append(actual_state[frame, 0])
    ydata.append(actual_state[frame, 1])
    xnt_data = nominal_trajectory[frame,:,0]
    ynt_data = nominal_trajectory[frame,:,1]
    if len(xdata) > 20:
        xdata.pop(0)
        ydata.pop(0)
        xndata.pop(0)
        yndata.pop(0)
    ln3.set_data(xnt_data, ynt_data)
    ln2.set_data(xndata, yndata)
    ln1.set_data(xdata, ydata)
    # fig.legend()
    title.set_text("Time: {val:05.2f} (sec)".format(val=frame*0.02+0.02))
    return ln1, ln2, ln3, title


def main(args):
    build_dir = args['build_dir']
    data_dir = build_dir + 'examples/'
    controller_name = controller_dict[args['controller']]
    fps = int(args['fps'])


    # Let us load the data first
    global actual_state, nominal_trajectory, nominal_state, real_fe, nominal_fe
    actual_state = np.load(data_dir + controller_name + 'state_trajectory.npy')
    nominal_trajectory = np.load(data_dir + controller_name + 'nominal_trajectory.npy')
    nominal_state = nominal_trajectory[:,0,:]
    ax.set_title(title_dict[args['controller']])

    # Set up formatting for the movie files
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=fps, bitrate=-1)
    ani = FuncAnimation(fig, update, frames=np.arange(0,actual_state.shape[0],1),
                    init_func=init, blit=True, interval=1000/fps, repeat=False)
    if args['save_mp4']:
        ani.save(title_dict[args['controller']] + '_trajectory' + '.mp4', writer=writer)
    else:
        print('Not saving mp4 file, pass --save_mp4 if you want to save it')
    # plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'Say hello')
    parser.add_argument('--build_dir', help='Location of MPPI-Generic build folder', required=True)
    parser.add_argument('--controller', help="Which controller we are plotting", required=True)
    parser.add_argument('--fps', required=False, default=50)
    parser.add_argument('--save_mp4', required=False, default=False)
    args = vars(parser.parse_args())

    main(args)