===============================================================================
Contact Lula Robotics for more information
===============================================================================

Baxter interface maintainer: Nathan Ratliff
email: nathan.ratliff@lularobotics.com

Other maintainers
- Jan Issac: jan.issac@lularobotics.com
- Daniel Kappler: daniel.kappler@lularobotics.com

===============================================================================
Setting up and building
===============================================================================

System requirements: Ubuntu 14.04 and ROS Indigo

To setup the system, firsts create a catkin workspace (or use an existing 
workspace, and clone the uw_init package into it. A canonical setup would 
be the following

  mkdir -p ~/lula_ws/src
  cd ~/lula_ws/src
  catkin_init_workspace
  cd ..  # It's common to place uw_init in the top level of the workspace
  git clone https://github.com/lularobotics/uw_init.git

The system is installed and updated from a script 

  uw_init/init.sh

which should be run from the top level of a catkin workspace (the script will
tell you if it's not being run from the right directory). Using the above
example workspace:

  cd ~/lula_ws
  uw_init/init.sh

It will step you through the installation processes, pulling the latest Docker
image from hub.lularobotics.com and cloning public Lula packages to a 'lula'
subdirectory of the workspace's 'src' directory. 

In order to visualize the baxter model there needs to be local installations in
that workspace of two rethink robotics packages

  baxter_description
  rethink_ee_description

Those two packages come with the baxter_common package. The above mentioned
script will search for them in the workspace, and if it does not find them 
it will clone baxter_common into 'src/lula/'. 

To manually install that package, you can clone it using

  git clone https://github.com/RethinkRobotics/baxter_common.git

Once the script is run, you can run all of the below mentioned demos. Remember
to source 

  <workspace>/devel/setup.bash

before running anything.


Here's a breakdown of what you should do the first time through:
1. Set the credientials to the ones we sent you by email.
2. Say yes when it asks to update the binary. If you've already run this step,
   the update will be very fast because the image will already be cached on
   your local machine. It's a good idea to always run this step.
3. Say yes to setting up workspace.
4. If there is a problem with the build, you can always rebuild manually by
   going into the workspace and running

     catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
   
   This workspace is like any other catkin workspace. The script just handles
   pulling the latest docker image and updating the lula packges.

To update, call the script again from the top-level of the catkin workspace.
It will lead you through the process. The only difference is that you don't
have to re-set your docker credentials after that first time.

===============================================================================
Running the demos in simulation
===============================================================================

The following assumes that each terminal has sourced the setup.bash file:

  source <workspace>/devel/setup.bash

Playing with continuous optimization

1. From a new terminal, start either the real robot or the simulator. To start
   the simulator, run the following:

   roslaunch lula_baxter baxter_simulator.launch

   To start the real robot, follow the instructions from Rethink. The simulator
   uses the same interface as the robot, so the rest of the continuous
   optimization system is agnostic to which is running.

2. From a separate terminal, start the continuous optimization node:
   
   roslaunch lula_baxter baxter_continuous_optimization.launch side:=<right|left>

3. From a third terminal, run one of the demo commands:

   rosrun lula_baxter <demo_script> <right|left>

4. Shut everything down cleanly using

   rosrun lula_baxter shutdown.sh


There are a number of demo scripts. For a first pass, try the following:

   rosrun lula_baxter move_to_default <right|left>
   rosrun lula_baxter stream_pose_demo1 <right|left>
   rosrun lula_baxter stream_pose_demo2 <right|left>

The visualization will show the target pose as a frame in Rviz; the robot
tracks the target and convergences to its exact pose once it stops moving.
These behaviors are useful for tracking moving grasp targets. For instance, 
the target pose could be attached to an object that's moving over time. The
system will prompt Baxter to reach out to it, and continually re-position
its end-effector as the object moves around. 

Existing demos:
1. stream_pose_demo1: Track a pose as it moves.
2. stream_pose_demo2: Track another pose as it moves.
3. stream_config_demo: Track a full configuration as it changes over time.
4. move_to_default: Moves the robot to a default configuration. Useful for
   resetting the pose of the robot to a neutral state. 
5. move_to_pose1: Move to the first pose of demo 1. It is not necessary to run
   this demo before running the stream_pose_demo1 demo.
6. move_to_pose2: Move to the first pose of demo 2. It is not necessary to run
   this demo before running the stream_pose_demo2 demo.

===============================================================================
Running the demos on the real robot
===============================================================================

The only difference between running demos in simulation and running them on the
real robot is:
1. You shouldn't start the simulator. The simulator streams joint state
   messages on the topic /robot/joint_states to mimic the output of the robot.
   If it's running while the robot is running and communicating with the same
   master, it'll confuse the optimizers.
2. In each terminal you need to run baxter.sh to initiate communications with
   the robot.

The steps are as follows. In each case, the example runs the script to connect
to Baxter provided in the lula_baxter library, but any sourcing of the connection
script baxter.sh will work as well. Each step assumes that you start from 
the root of the workspace.

1. Activate the robot:

  ./src/lula/lula_baxter/scripts/connect_to_baxter.sh baxter
  rosrun baxter_tools enable_baxter.py -e

2. In a new terminal, start the continuous optimization:

  ./src/lula/lula_baxter/scripts/connect_to_baxter.sh baxter
  roslaunch lula_baxter baxter_continuous_optimization.launch \
      side:=<right|left> \
      visualize:=true

  (Note that since the simulator isn't running you need to tell the 
  baxter_continuous_optimization.launch file to additional launch 
  a visualization node. This just starts rviz; it doesn't start the 
  simulator.)

3. Run the demos:
    
  ./src/lula/lula_baxter/scripts/connect_to_baxter.sh baxter
  rosrun lula_baxter <demo_script> <right|left>

  where the <demo_scripts> are the same as the simulation scripts.

4. Shutdown everything -- disable the robot, then shutdown the nodes:

  rosrun baxter_tools enable_baxter.py -d
  rosrun lula_baxter shutdown.sh
  

===============================================================================
How it works
===============================================================================

Once the continuous optimization node is started (step 2 of the above startup
sequence), the system starts continuously optimizing the latest problem.
Initially that problem specifies nothing more than that the robot should
minimize its velocities and accelerations. The net result is that it, for the
most part, just stays where it is. You may see it drift slightly away from its
joint limits depending on its configuration. 

It listens to desired pose and desired configuration messages. When it receives
either, it sets the latest optimization problem to move the robot to the
desired pose or configuration, starts optimizing it, and starts executing the
resulting plan. If the desired pose/configuration moves over time, the latest
optimization problem keeps up-to-date with the latest desired
pose/configuration so that the optimizer tracks the latest information.

For instance, stream_pose_demo1 and stream_pose_demo2 simply stream
PoseStamped messages to the continuous optimization node. Those messages
are interpreted by the optimizer and transformed into motion optimization
problems that the system promptly optimizes. 

In both of those demos, the target pose moves over time and the robot tracks it
until the frame stops and the robot converges to the target. In all case, even
though the robot has stopped moving, it's just in a convergent state. The
optimizer continues to run as before, but the problem has already reached the
goal so the optimal move is to remain stationary. When the second demo is run,
the problem suddenly changes, so the optimizer optimizes to the new target and
starts tracking the new movement. There is no underlying state transition, the
robot just monitors the current problem, optimizes it continuously, and
executes beginning of the most recent plan until it receives a new updated plan
from the optimizer. In the baxter_controller window, you can see it receives
new plans approximately every .1 seconds.

===============================================================================
The basic API
===============================================================================

The above demos use the public API described here to communicate with the 
optimizer.

1. Send (or stream) a pose to the robot by sending geometry_msgs/PoseStamped
   messages to the following topic:
   
   /RieMO/Baxter/[Right/Left]/DesiredPose

2. Send (or stream) a configuration to the robot by sending
   lula_baxter/Configuration messages to the following topic:
   
   /RieMO/Baxter/[Right/Left]DesiredConfiguration

   The Configuration message requires just a header and a specification 
   of the active joint angles:

   Header header 
   float64[] q

   The ordering of the joints is specified by the list of "generalized
   coordinates" given in the RieMO descriptor file:

   lula_baxter/config/riemo_robot_baxter_[right/left].json

   At installation, that specification is set as to control the right arm
   with a joint ordering ranging from most proximal to most distal:

     "generalizedCoordinates": [
       "[right/left]_s0",
       "[right/left]_s1",
       "[right/left]_e0",
       "[right/left]_e1",
       "[right/left]_w0",
       "[right/left]_w1",
       "[right/left]_w2"
     ],

See lula_baxter/lula/baxter/stream_target_main.cpp for examples of how to use
the API to stream both desired poses and desired configurations. This binary is
called by the demo scripts (e.g. stream_pose_demo1) mentioned above.
