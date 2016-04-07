===============================================================================
Setting up and building
===============================================================================

System requirements: Ubuntu 14.04 and ROS Indigo

Run the following:

  mkdir lula; cd lula
  git clone https://github.com/lularobotics/uw_init.git
  ./uw_init/init.sh

Follow the instructions:
1. Set the credientials to the ones we sent you by email.
2. Say yes when it asks to update the binary. If you've already run this step,
   the update will be very fast because the image will already be cached on
   your local machine. It's a good idea to always run this step.
3. Say yes to setting up the lularobotics_ws workspace.
4. If there is a problem with the build, you can always rebuild manually by
   going into the lularobotics_ws workspace and running

     catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
   
   In general, when changing any of the client code (see the note below about
   playing with speed modulation in the execution), you can rebuild manually
   this way.


===============================================================================
How to run the demo(s)
===============================================================================


===============================================================================
The basic API
===============================================================================


