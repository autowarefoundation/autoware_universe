# reads in an LSTM npz and saves it out so that we can read it with cnpy
import torch
import numpy as np
import pickle
from torch import Tensor
from torch.nn.parameter import Parameter

#filename = "/home/jason/Documents/research/MPPI-Generic-Model-Learning/Results/train/autorally/PF_1000_integrated_fixed_1/LSTM_1X15-1000X10_0.02-0_I/model_store/LSTM_1X15-1000X10_0.02-0_I_37_1.npz"
filename = "/home/jason/Documents/research/MPPI-Generic-Model-Learning/Results/train/autorally/PF_200_fixed/LSTM_1X15-200X10_0.02-0_I/LSTM_1X15-200X10_0.02-0_I.npz"


network = np.load(open(filename, 'rb'), allow_pickle=True)
hidden_init = network["hidden_init"].item()
cell_init = network["cell_init"].item()
output = network["output"].item()
lstm = network["lstm"].item()

np.savez("PF_200_hidden_init.npz", **hidden_init)
np.savez("PF_200_cell_init.npz", **cell_init)
np.savez("PF_200_output.npz", **output)
np.savez("PF_200_lstm.npz", **lstm)

test = np.load(open("/home/jason/catkin_ws/ar/src/MPPI-ROS/"
                    "submodules/MPPI-Generic/resources/hidden_init.npz", 'rb'), allow_pickle=True)
print(test)
print(test.keys())
print(test["dynamics_W1"])

test = np.load(open("PF_1000_hidden_init.npz", 'rb'), allow_pickle=True)
print(test)
print(test.keys())
print(test["dynamics_W1"])

#np.savez(filename, **pa)
