import numpy as np
from matplotlib import rc
rc('font', **{'family': 'serif', 'serif': ['Computer Modern'], 'size': 16})
rc('text', usetex=True)
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

def plot_fe_vs_time(fe_array, crash_cost=1.0, title="", savefig=0):
	fig = plt.figure()
	axis = plt.gca()
	# axis.set_title(title)
	time = np.linspace(0,fe_array.shape[0]*0.02, fe_array.shape[0])
	plt.plot(time, np.ones_like(time), 'r', alpha=0.7, label='Crash Threshold')
	plt.plot(time, fe_array/crash_cost, 'b', alpha=0.7, label='Free Energy')
	plt.legend()
	if savefig:
		fig.savefig(title+'.pdf', bbox_inches='tight')

def plot_fe_bounded(fe_array, fe_bound, title="", savefig=0):
	fig = plt.figure()
	axis = plt.gca()
	# axis.set_title(title)
	# plt.yscale('log')
	time = np.linspace(0,fe_array.shape[0]*0.02, fe_array.shape[0])
	plt.plot(time, fe_array, 'b', alpha=0.7, label='FE Increase')
	plt.plot(time, fe_bound, 'r', alpha=0.7, label='Bound')
	plt.legend(bbox_to_anchor=(1.05, 1),loc='upper left')
	if savefig:
		fig.savefig(title+'.pdf', bbox_inches='tight')

def plot_fe_vs_space(fe_array, state_array):
	pass

def plot_trajectory(trajectory, title="", savefig=0):
	fig = plt.figure()
	axis = plt.gca()
	# axis.set_title(title)
	axis.set_aspect('equal', 'box')
	axis.set(xlim=(-2.5, 2.5), ylim=(-2.5, 2.5))
	axis.plot(trajectory[:,0], trajectory[:,1], 'limegreen', alpha=0.95, linewidth=1.5, label='Actual Trajectory')
	axis.plot(x_track_inner, y_track_inner, 'k', linewidth=2)
	axis.plot(x_track_outer, y_track_outer, 'k', linewidth=2)
	if savefig:
		fig.savefig(title+'_state_trajectory.pdf', bbox_inches='tight')


def plot_nominal_trajectory(trajectory, nominal_trajectory, title="", savefig=0):
	fig = plt.figure()
	axis = plt.gca()
	# axis.set_title(title)
	axis.set_aspect('equal', 'box')
	axis.set(xlim=(-2.3, 2.3), ylim=(-2.3, 2.3))

	time_horizon, trajectory_length, state_dim = nominal_trajectory.shape

	for i in range(time_horizon-1):
		axis.plot(nominal_trajectory[i,:,0], nominal_trajectory[i,:,1], 'b', linewidth=1.0, alpha=0.7)

	axis.plot(nominal_trajectory[time_horizon-1,:,0], nominal_trajectory[time_horizon-1,:,1], 'b', linewidth=1.0, alpha=0.7, label='Nom')
	axis.plot(trajectory[:,0], trajectory[:,1], 'limegreen', alpha=0.95, linewidth=1.5, label='Act')
	axis.plot(x_track_inner, y_track_inner, 'k', linewidth=2)
	axis.plot(x_track_outer, y_track_outer, 'k', linewidth=2)
	# plt.legend(bbox_to_anchor=(1.05, 1),loc='upper left')
	if savefig:
		fig.savefig(title+'_nominal_trajectory.pdf', bbox_inches='tight')




def plot_nominal_state_used_vs_time(ns_array, title=""):
	plt.figure()
	axis = plt.gca()
	axis.set_title(title)
	time = np.linspace(0,ns_array.shape[0]*0.02, ns_array.shape[0])
	plt.scatter(time, ns_array)


def main(args):
	build_dir = args['build_dir']
	data_dir = build_dir + 'examples/'
	show_plot = args['show_fig']
	save_fig = args['save_fig']

	vanilla_fe = np.load(data_dir + "vanilla_free_energy.npy")
	vanilla_state_trajectory = np.load(data_dir + "vanilla_state_trajectory.npy")
	vanilla_robust_state_trajectory = np.load(data_dir + "vanilla_large_robust_state_trajectory.npy")
	vanilla_robust_fe = np.load(data_dir + "vanilla_large_robust_free_energy.npy")

	# vanilla_large_fe = np.load(data_dir + "vanilla_large_free_energy.npy")
	vanilla_large_state_trajectory = np.load(data_dir + "vanilla_large_state_trajectory.npy")
	vanilla_sc_nominal_trajectory = np.load(data_dir + "vanilla_nominal_trajectory.npy")


	# tube_large_rfe = np.load(data_dir + "tube_real_free_energy.npy")
	# tube_large_nfe = np.load(data_dir + "tube_nominal_free_energy.npy")
	tube_large_state_trajectory = np.load(data_dir + "tube_state_trajectory.npy")
	tube_large_nominal_state_used = np.load(data_dir + "tube_nominal_state_used.npy")
	tube_sc_nominal_trajectory = np.load(data_dir + "tube_nominal_trajectory.npy")

	tube_robust_state_trajectory = np.load(data_dir + "tube_robust_state_trajectory.npy")
	tube_robust_nominal_trajectory = np.load(data_dir + "tube_robust_nominal_trajectory.npy")
	tube_robust_rfe = np.load(data_dir + "tube_robust_real_free_energy.npy")


	robust_sc_rfe = np.load(data_dir + "robust_sc_real_free_energy.npy")
	# robust_sc_nfe = np.load(data_dir + "robust_sc_nominal_free_energy.npy")
	robust_sc_state_trajectory = np.load(data_dir + "robust_sc_state_trajectory.npy")
	robust_sc_nominal_state_used = np.load(data_dir + "robust_sc_nominal_state_used.npy")
	robust_sc_nominal_trajectory = np.load(data_dir + "robust_sc_nominal_trajectory.npy")


	robust_rc_rfe = np.load(data_dir + "robust_rc_real_free_energy.npy")
	# robust_rc_nfe = np.load(data_dir + "robust_rc_nominal_free_energy.npy")
	robust_rc_state_trajectory = np.load(data_dir + "robust_rc_state_trajectory.npy")
	robust_rc_nominal_state_used = np.load(data_dir + "robust_rc_nominal_state_used.npy")
	robust_rc_rfe_bound = np.load(data_dir + "robust_rc_real_free_energy_bound.npy")
	robust_rc_nfe_bound = np.load(data_dir + "robust_rc_nominal_free_energy_bound.npy")
	robust_rc_rfe_growth_bound = np.load(data_dir + "robust_rc_real_free_energy_growth_bound.npy")[1:]
	# robust_rc_rfc_growth = np.load(data_dir + "robust_rc_real_free_energy_growth.npy")
	robust_rc_rfc_growth = robust_rc_rfe[1:] - robust_rc_rfe[:-1]
	robust_rc_nfc_growth = np.load(data_dir + "robust_rc_nominal_free_energy_growth.npy")
	robust_rc_nominal_trajectory = np.load(data_dir + "robust_rc_nominal_trajectory.npy")


	robust_ccm_rfe = np.load(data_dir + "robust_large_actual_free_energy_CCM_t_2999.npy")
	robust_ccm_nfe = np.load(data_dir + "robust_large_nominal_free_energy_CCM_t_2999.npy")
	robust_ccm_state_trajectory = np.load(data_dir + "robust_large_actual_traj_CCM_t_2999.npy")
	robust_ccm_nominal_state_used = np.load(data_dir + "robust_large_nominal_state_used_CCM_t_2999.npy")
	robust_ccm_rfe_bound = np.load(data_dir + "robust_large_actual_free_energy_bound_CCM_t_2999.npy")
	robust_ccm_nfe_bound = np.load(data_dir + "robust_large_nominal_free_energy_bound_CCM_t_2999.npy")
	robust_ccm_rfe_growth_bound = np.load(data_dir + "robust_large_actual_free_energy_growth_bound_CCM_t_2999.npy")
	# robust_ccm_rfc_growth = np.load(data_dir + "robust_large_actual_free_energy_growth_CCM_t_2999.npy")
	robust_ccm_rfc_growth = robust_ccm_rfe[1:] - robust_ccm_rfe[:-1]
	robust_ccm_nfc_growth = np.load(data_dir + "robust_large_nominal_free_energy_growth_CCM_t_2999.npy")
	robust_ccm_nominal_trajectory = np.load(data_dir + "robust_large_nominal_traj_CCM_t_2999.npy")



	# plot_fe_vs_time(vanilla_fe, 1000,"Vanilla Real Free Energy")
	# plot_trajectory(vanilla_state_trajectory, "Vanilla State Trajectory", save_fig)

	# plot_fe_vs_time(vanilla_large_fe, 1000,"Vanilla Large Real Free Energy")
	# plot_trajectory(vanilla_large_state_trajectory, "Vanilla Large State Trajectory", save_fig)

	# plot_fe_vs_time(vanilla_robust_fe, 100, "Vanilla Robust Real Free Energy", save_fig)
	# plot_trajectory(vanilla_robust_state_trajectory, "Vanilla Robust State Trajectory", save_fig)
	#
	# plot_fe_vs_time(tube_large_rfe, 1000,"Tube Real Free Energy")
	# plot_fe_vs_time(tube_large_nfe, 1000,"Tube Nominal Free Energy")
	# plot_fe_vs_time(tube_robust_rfe, 100, "Tube Robust Real Free Energy", save_fig)
	# plot_trajectory(tube_large_state_trajectory, "Tube State Trajectory", save_fig)
	# plot_nominal_trajectory(tube_large_state_trajectory, tube_sc_nominal_trajectory, "Tube Standard State Trajectory", save_fig)
	# plot_nominal_trajectory(tube_robust_state_trajectory, tube_robust_nominal_trajectory, "Tube Robust State Trajectory", save_fig)

# # plot_nominal_state_used_vs_time(tube_large_nominal_state_used)
	#
	# plot_fe_vs_time(robust_sc_rfe, 1000, "Robust Standard Real Free Energy", save_fig)
	# plot_fe_vs_time(robust_sc_nfe, 1000, "Robust Standard Nominal Free Energy")
	# plot_trajectory(robust_sc_state_trajectory, "Robust Standard State Trajectory")
	# # plot_nominal_state_used_vs_time(robust_sc_nominal_state_used)
	# plot_nominal_trajectory(robust_sc_state_trajectory, robust_sc_nominal_trajectory, "Robust Standard State Trajectory", save_fig)

	# plot_fe_vs_time(robust_rc_rfe, 100, "Robust Robust Real Free Energy", save_fig)
	plot_fe_bounded(robust_rc_rfc_growth, robust_rc_rfe_growth_bound, "Robust Robust Real Free Energy Growth")
	# plot_fe_bounded(robust_rc_nfe, robust_rc_nfe_bound, "Robust Robust Nominal Free Energy")
	# plot_fe_vs_time(robust_rc_nfc_growth, 0,"Robust Robust Nominal Free Energy Growth")
	# plot_trajectory(robust_rc_state_trajectory, "Robust Robust State Trajectory")
	# plot_nominal_trajectory(robust_rc_state_trajectory, robust_rc_nominal_trajectory, "Robust Robust State Trajectory", save_fig)

	# plot_nominal_state_used_vs_time(robust_rc_nominal_state_used)

	# Correct bound
	incorrect_bound = (robust_ccm_rfe_bound - robust_ccm_nfe)
	correct_bound = 8*incorrect_bound
	# plot_fe_vs_time(robust_ccm_rfe, 100, "Robust CCM Real Free Energy", save_fig)
	plot_fe_bounded(robust_ccm_rfc_growth, (robust_ccm_rfe_growth_bound-incorrect_bound+correct_bound)[1:], "Robust CCM Real Free Energy Growth")
	# plot_fe_bounded(robust_ccm_nfe, robust_ccm_nfe_bound, "Robust CCM Nominal Free Energy")
	# plot_fe_vs_time(robust_ccm_nfc_growth, 0,"Robust CCM Nominal Free Energy Growth")
	# plot_trajectory(robust_ccm_state_trajectory, "Robust CCM State Trajectory")
	# plot_nominal_state_used_vs_time(robust_ccm_nominal_state_used)
	# plot_nominal_trajectory(robust_ccm_state_trajectory, robust_ccm_nominal_trajectory, "Robust CCM State Trajectory", save_fig)

	if show_plot:
		plt.show()


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description = 'Say hello')
	parser.add_argument('--build_dir', help='Location of MPPI-Generic build folder', required=True)
	parser.add_argument('--show_fig', default=False, help='Show the figure upon completion of plotting.', required=False)
	parser.add_argument('--save_fig', default=False, help='Save the figures after they are generated.', required=False)

	args = vars(parser.parse_args())

	main(args)
