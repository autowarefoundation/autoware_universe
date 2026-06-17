#!/usr/bin/env python3

import numpy as np
from PIL import Image
import pickle
import argparse

def genLoadNetworkDataTest(args):
    param_dict = {}
    layer_counter = 0

    net_structure = [6, 32, 32, 4]
    params = 0
    for i in range(1, len(net_structure)):
        inc = net_structure[i-1] * net_structure[i]
        print("param  = ", params, " inc = ", inc)
        param_dict["dynamics_W" + str(i)] = np.arange(params, params + inc, dtype=np.float64)
        params += inc
        inc = net_structure[i]
        print("param  = ", params, " inc = ", inc)
        param_dict["dynamics_b" + str(i)] = np.arange(params, params + inc, dtype=np.float64)
        params += inc
    print(param_dict)

    np.savez(args.output+"/neuralNetLoadTest.npz", **param_dict)

def genComputationNetworkTest(args):
    param_dict = {}

    net_structure = [4, 3, 4]
    for i in range(1, len(net_structure)):
        param_dict["dynamics_W" + str(i)] = 1
        param_dict["dynamics_b" + str(i)] = 1
    print(param_dict)

    np.savez(args.output+"/neuralNetComputeTest.npz", **param_dict)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates a npz file for testing the NN")
    # parser.add_argument("-i", "--input", type = str, help = "Costmap in old .txt format")
    # parser.add_argument("-d", "--display", type = str,
    #                     help = "Name of image to save costmap as",
    #                     default="display_image.jpg")
    parser.add_argument("-o", "--output", type = str,
                        help = "File to save map to",
                        default = "../../../resource/autorally/test/")
    args = parser.parse_args()
    genLoadNetworkDataTest(args)
    genComputationNetworkTest(args)
