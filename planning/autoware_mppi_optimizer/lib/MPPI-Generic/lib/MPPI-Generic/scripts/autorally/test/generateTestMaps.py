#!/usr/bin/env python3

import numpy as np
from PIL import Image
import math
import argparse

def genLoadTrackDataTestMap(args):

    # used to check if the map is being loaded correctly
    width = 10
    height = 20
    pixelsPerMeter = 2

    channel0 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel1 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel2 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel3 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)

    counter = 0
    for i in range(0, width*pixelsPerMeter):
        for j in range(0, height*pixelsPerMeter):
            # TODO this is actually flipped, but loaded into CUDA correctly
            #print("putting width and height, ", pixelsPerMeter * height * i + j, " ", counter)
            counter+= 1
            channel0[i, j] = counter
            channel1[i, j] = counter * 10
            channel2[i, j] = counter * 100
            channel3[i, j] = counter * 1000
    #print(channel0.flatten())

    #Save data to numpy array, each channel is saved individually as an array in row major order.
    track_dict = {"xBounds":np.array([-width/2, width/2], dtype = np.float32),
                  "yBounds":np.array([-height/2, height/2], dtype = np.float32),
                  "pixelsPerMeter":np.array([pixelsPerMeter], dtype=np.float32),
                  "channel0":channel0.flatten(),
                  "channel1":channel1.flatten(),
                  "channel2":channel2.flatten(),
                  "channel3":channel3.flatten()}


    np.savez(args.output+"track_map.npz", **track_dict)

    # used to check for valid outputs of the AutoRally standard cost function
    # dims match the standard ones used to marietta track
    width = 30
    height = 30
    pixelsPerMeter = 20

    channel0 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel1 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel2 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel3 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)

    for i in range(0, width*pixelsPerMeter):
        for j in range(0, height*pixelsPerMeter):
            x = (j*1.0)/pixelsPerMeter
            y = (i*1.0)/pixelsPerMeter
            channel0[i, j] = abs(height/2.0 - y) + ((x)/(width))
            #if ((x == 12.5 or x == 13.5) and y == 0):
            #print(x,y, " = ", channel0[i,j])
            #if channel0[i, j] < 1.0:
            #    print("putting width and height ",i , " ", j, " = ", channel0[i,j])
    #print(channel0.flatten())

    track_dict = {"xBounds":np.array([-13, 17], dtype = np.float32),
                  "yBounds":np.array([-10, 20], dtype = np.float32),
                  "pixelsPerMeter":np.array([pixelsPerMeter], dtype=np.float32),
                  "channel0":channel0.flatten(),
                  "channel1":channel1.flatten(),
                  "channel2":channel2.flatten(),
                  "channel3":channel3.flatten()}

    np.savez(args.output+"track_map_standard.npz", **track_dict)

    # used to check for valid outputs of the AutoRally robust cost function
    # dims match the standard ones used to CCRF track
    width = 70
    height = 55
    pixelsPerMeter = 20

    channel0 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel1 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel2 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)
    channel3 = np.zeros((width*pixelsPerMeter, height*pixelsPerMeter), np.float32)

    for i in range(0, width*pixelsPerMeter):
        for j in range(0, height*pixelsPerMeter):
            x = (j*1.0)/pixelsPerMeter
            y = (i*1.0)/pixelsPerMeter
            channel1[i, j] = abs(height/2.0 - y) + ((x)/(width))
            if x > 50 or x < 15:
                channel0[i,j] = 1.0
            elif x > 40 or x < 25:
                channel0[i,j] = 0.6
            channel2[i,j] = x
            channel3[i,j] = math.atan2(y,x)
            #print("putting x,y = ",x , " ", y, " = ", channel0[i,j], channel1[i,j], channel2[i,j], channel3[i,j])
    #print(channel3.flatten())

    track_dict = {"xBounds":np.array([-25, 45], dtype = np.float32),
                  "yBounds":np.array([-50, 5], dtype = np.float32),
                  "pixelsPerMeter":np.array([pixelsPerMeter], dtype=np.float32),
                  "channel0":channel0.flatten(),
                  "channel1":channel1.flatten(),
                  "channel2":channel2.flatten(),
                  "channel3":channel3.flatten()}

    np.savez(args.output+"track_map_robust.npz", **track_dict)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates a npz map file for testing purposes")
    # parser.add_argument("-i", "--input", type = str, help = "Costmap in old .txt format")
    # parser.add_argument("-d", "--display", type = str,
    #                     help = "Name of image to save costmap as",
    #                     default="display_image.jpg")
    parser.add_argument("-o", "--output", type = str,
                        help = "File to save map to",
                        default = "../../../resource/autorally/test/")
    args = parser.parse_args()
    genLoadTrackDataTestMap(args)
