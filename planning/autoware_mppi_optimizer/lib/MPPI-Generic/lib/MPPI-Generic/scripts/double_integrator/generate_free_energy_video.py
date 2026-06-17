import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import numpy as np
from matplotlib import rc
import argparse

controller_dict = {'v': 'vanilla_large_',
                   't': 'tube_',
                   'rs': 'robust_sc_',
                   'rr': 'robust_rc_',
                   'rhv': 'test_mppi_',
                   'rht': 'test_tmppi_',
                   'rhr': 'test_rmppi_',
                   'cc':  'robust_cc_',
                   'vr': 'vanilla_large_robust_',
                   'tr': 'tube_robust_'}

title_dict = {'v': 'MPPI Standard Cost',
                   't': 'Tube-MPPI Standard Cost',
                   'rs': 'RMPPI LQR Standard Cost',
                   'rr': 'RMPPI LQR Robust Cost',
                   'rhv': 'MPPI Autorally',
                   'rht': 'Tube-MPPI Autorally',
                   'rhr': 'RMPPI Autorally',
                   'cc': 'RMPPI CCM Robust Cost',
                   'vr': 'MPPI Robust Cost',
                   'tr': 'Tube-MPPI Robust Cost'}

rc('font', **{'size': 30})


fig, ax = plt.subplots()
fig.set_dpi(100)
fig.set_size_inches(10, 10)
xdata, ydata, yndata = [], [], []

ax.set_ylabel('Log(Free Energy)',labelpad=-30, position=(0.5,.6))
ax.set_xlabel('Time (sec)')
ax.set_yscale('log')

#ax.xaxis.set_ticklabels([])
title = None
ln3, = ax.plot([], [], 'r', alpha=0.7, linewidth=3, label='Nominal')
ln2, = ax.plot([], [], 'b', alpha=0.7, linewidth=3, label='Real')
ln1, = ax.plot(np.linspace(0,1000,100), 1*np.ones(100), 'k', alpha=0.7, linewidth=3, label='Crash')


def init():
    ax.set_xlim(0, time_window)
    ax.set_ylim(ymin, ymax)
    return ln1, ln2, ln3


def update(frame):
    xdata.append(time[frame])
    ydata.append(real_fe[frame])
    yndata.append(nominal_fe[frame])
    ln2.set_data(xdata, ydata)
    ln3.set_data(xdata, yndata)
    ax.legend(loc=2)
    if (xdata[-1] - xdata[0]) > time_window:
        xdata.pop(0)
        ydata.pop(0)
        yndata.pop(0)
        ln1.set_data(xdata, 1)
        ax.set_xlim(xdata[0], xdata[-1])
    else:
        ax.set_xlim(xdata[0], xdata[0]+time_window)

    return ln1, ln2, ln3


def index_data(time, nominal_fe, real_fe, begin_time, end_time):
    ind = tuple([begin_time < time])
    time = time[ind]
    real_fe = real_fe[ind]
    nominal_fe = nominal_fe[ind]
    ind = tuple([time < end_time])
    time = time[ind]
    real_fe = real_fe[ind]
    nominal_fe = nominal_fe[ind]
    return time, nominal_fe, real_fe


def main(args):
    build_dir = args['build_dir']
    data_dir = build_dir + 'examples/'
    controller_name = controller_dict[args['controller']]
    fps = int(args['fps'])

    # Let us load the data first
    global real_fe, nominal_fe, time_window, time, ymin, ymax

    time_window = int(args['time_window'])

    # Load the free energy data
    if args['controller'] == 'v':
        time = np.linspace(0.02, 0.02*5000, 5000)
        real_fe = np.load(data_dir + controller_name + 'real_free_energy.npy')/1000
        nominal_fe = -1*np.ones_like(real_fe)

    elif args['controller'] == 'rhv':
        data = np.load(data_dir + controller_name + 'real_free_energy.npz')
        time = data['arr_0']
        real_fe = data['arr_1']/10000
        nominal_fe = -1*np.ones_like(real_fe)
        data.close()
        time, nominal_fe, real_fe = index_data(time, nominal_fe, real_fe, 150, 190)

    elif args['controller'] == 'rht':
        data = np.load(data_dir + controller_name + 'real_free_energy.npz')
        time = data['arr_0']
        real_fe = data['arr_1']/10000
        data.close()
        data = np.load(data_dir + controller_name + 'nominal_free_energy.npz')
        nominal_fe = data['arr_1']/10000
        data.close()
        time, nominal_fe, real_fe = index_data(time, nominal_fe, real_fe, 275, 315)

    elif args['controller'] == 'rhr':
        data = np.load(data_dir + controller_name + 'real_free_energy.npz')
        time = data['arr_0']
        real_fe = data['arr_1']/125000
        data.close()
        data = np.load(data_dir + controller_name + 'nominal_free_energy.npz')
        nominal_fe = data['arr_1']/125000
        data.close()
        time, nominal_fe, real_fe = index_data(time, nominal_fe, real_fe, 180, 220)
    elif args['controller'] == 'cc':
        time = np.linspace(0.02, 0.02*3000, 3000)
        real_fe = np.load(data_dir + controller_name + 'real_free_energy.npy')/100
        nominal_fe = np.load(data_dir + controller_name + 'nominal_free_energy.npy')/100
    elif args['controller'] == 'vr':
        time = np.linspace(0.02, 0.02*5000, 5000)
        real_fe = np.load(data_dir + controller_name + 'real_free_energy.npy')/100
        nominal_fe = -1*np.ones_like(real_fe)
    elif args['controller'] == 'tr':
        time = np.linspace(0.02, 0.02*5000, 5000)
        real_fe = np.load(data_dir + controller_name + 'real_free_energy.npy')/100
        nominal_fe = np.load(data_dir + controller_name + 'nominal_free_energy.npy')/100
    elif not args['controller'] == 'rr':
        time = np.linspace(0.02, 0.02*5000, 5000)
        real_fe = np.load(data_dir + controller_name + 'real_free_energy.npy')/1000
        nominal_fe = np.load(data_dir + controller_name + 'nominal_free_energy.npy')/1000
    else:
        time = np.linspace(0.02, 0.02*5000, 5000)
        real_fe = np.load(data_dir + controller_name + 'real_free_energy.npy')/100
        nominal_fe = np.load(data_dir + controller_name + 'nominal_free_energy.npy')/100
    ax.set_title(title_dict[args['controller']])

    if np.mean(nominal_fe) == -1:
        ymin = np.amin(real_fe)
    else:
        ymin = min(np.amin(real_fe), np.amin(nominal_fe))
    ymax = max(1.1,max(np.amax(real_fe), np.amax(nominal_fe)))

    # Set up formatting for the movie files
    Writer = animation.writers['ffmpeg']
    writer = Writer(fps=fps, bitrate=-1)

    ani = FuncAnimation(fig, update, frames=np.arange(0,time.shape[0],1),
                    init_func=init, blit=False, interval=1000/fps, repeat=False)
    if args['save_mp4']:
        ani.save(title_dict[args['controller']] + '_free_energy' + '.mp4', writer=writer)
    else:
        print('Not saving mp4 file, pass --save_mp4=True if you want to save it')
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'Say hello')
    parser.add_argument('--build_dir', help='Location of MPPI-Generic build folder', required=True)
    parser.add_argument('--controller', help="Which controller we are plotting", required=True)
    parser.add_argument('--time_window', help='Time window size (s)', required=False, default=5)
    parser.add_argument('--fps', required=False, default=50)
    parser.add_argument('--save_mp4', required=False, default=False)
    args = vars(parser.parse_args())

    main(args)
