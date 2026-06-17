# Overview
The overall structure for the Feedback Controller is a bit confusing but I will try to lay out the basic idea here.
There are two halves of the Feedback Controller, the GPU portion and the CPU portion.
These are separated into different classes as we need to do different things on the different devices.
The GPU portion just needs to provide feedback control while the CPU portion needs to do that and have the ability to update the feedback controller itself.


# Steps to creating a new FeedbackController
## Create a new FeedbackParams struct
This should be where the necessary data for the CPU portion of the controller resides.
This does not get copied over to the GPU so the struct can have Eigen matrices in it.
For example, this struct has the Q, Q_f, and R matrices for DDP and could have the desired poles for a PID controller.

## Create  a new FeedbackState struct
This is the data structure that will contain everything necessary to actually compute feedback on the GPU and CPU.
This struct does get copied to the GPU so try to use float arrays of known sizes to prevent a need to rewrite the copy to GPU code.
It should also inherit from the GPUState struct in `feedback.cuh` as that has a necessary variable SHARED_MEM_SIZE that is used to allocate shared memory on the GPU for the feedback controller to use.
This space would be needed for keeping track of PID integrated errors for each sample of MPPI as an example.
For DDP, this is where the trajectory of feedback gains is stored, and for a PID controller, this would be where you put your P, I, and D gains.

## Create a GPUFeedbackController Class
This should inherit from the GPUFeedbackController template in `feedback.cuh` as that provides a lot of methods already.
You at minimum need to write the `void k(x_act, x_goal, t, theta, control_output)` method to return the feedback control (`control output`) you would expect given the current state (`x_act`), the goal state (`x_goal`), the timestep you are on (`t`), and some potentially scratch space (`theta`).
For DDP, this is ends up being essentially `feedback_gain_traj[t] * (x_act - x_goal)`  (no use for `theta`).

This class also has methods you can overwrite for copyToDevice() and copyFromDevice() if you have some fancy GPUFeedback or want to get diagnostic information back from the GPU.

## Create a FeedbackController Class
This should inherit from the FeedbackController template in `feedback.cuh` which requires the GPUFeedback Class and the FeedbackParams struct you made earlier.
There are 3 methods at minimum you need to overwrite for this class.

1. `void initTrackingController()`  is where you can do some setup that might not be done in the  constructor (DDP uses this to setup the ddp optimizer using the current params)
2. `void computeFeedback(init_state, goal_state_traj, control_traj)` is where you can write how to update your feedback controller.
For DDP, this is where the optimization occurs and the new feedback gains trajectory is put into the DDPFeedbackState struct.
3. `control_array k_(x_act, x_goal, t, fb_state)` is the CPU version of calculating the feedback control.
It has the FeedbackState passed in as there are times that the feedback control needs to be calculated from a different place than the internal FeedbackState (the internal state might be in the middle of updating for example).
The resulting control needs to be the same as the GPU Controller output given the same FeedbackState.

# Conclusion
After creating all of the components, you can then use this controller class as the feedback controller template argument in MPPI, Tube, RMPPI, and it "should" plug in without any problems. The DDP implementation is the only example at the moment so use that as a guiding resource if you get lost.
