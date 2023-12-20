************
Installation
************

To note, ``rosgenmsh`` (ROS 1) and ``ros2genmsg`` (ROS 2) have one
rather annoying limitation, they expect a directory that contains
directories of ROS packages. So if you clone ``crtk_msgs`` in
``my_ws/src`` you will need to use ``rosgenmsg`` (or ``ros2genmsg``)
on all packages in ``my_ws/src``.  To avoid this, you need to create a
sub-directory in ``my_ws/src`` and then clone ``crtk_msgs`` in said
sub-directory (for example ``~/my_ws/src/crtk/crtk_msgs``).

Matlab uses pretty similar names for the ROS 1 and ROS 2 commands so
the following instructions are valid for both ROS 1 and ROS 2.
Instructions below are for ROS 2.  For ROS 1, replace ``ros2_ws`` by
``catkin_ws``, ``ros2genmsg`` by ``rosgenmsg`` and ``colcon build`` by
``catkin_build``.

.. note::

   If you've installed a package relying on CRTK using a tool such as
   ``vcs``, it is quite possible all the CRTK dependencies have
   already been pulled from GitHub.  Before pulling another copy,
   check the content of your ROS workspace ``src`` directory.

If you're cloning ``crtk_msgs`` by hand, you can use the following instructions.

ROS 1
=====

.. code-block:: bash

   cd ~/catkin_ws/src
   # clone in sub-directory crtk and rename to crtk_msgs
   git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
   git clone https://github.com/collaborative-robotics/crtk_matlab_client crtk/crtk_matlab_client
   cd ~/catkin_ws
   catkin build


ROS 2
=====

.. code-block:: bash

   cd ~/ros2_ws/src
   # clone in sub-directory crtk and rename to crtk_msgs
   git clone https://github.com/collaborative-robotics/ros2_crtk_msgs crtk/crtk_msgs
   git clone https://github.com/collaborative-robotics/ros2_crtk_matlab_client crtk/crtk_matlab_client
   cd ~/ros2_ws
   colcon build


*************************
Custom message generation
*************************

At that point, you can finally generate the code **in Matlab**:

ROS 1:

.. code-block:: matlab

   rosgenmsg('~/catkin_ws/src/crtk')

ROS 2:

.. code-block:: matlab

   ros2genmsg('~/ros2_ws/src/crtk')

You could follow the instructions displayed at the end of the message
generation to use ``savepath`` but this would apply to all users on
the workstation.  So you should probably skip that step and read the
following section so the paths would be defined per user.

Instead of setting the path for all users, you can use your startup
script.  To edit the user's ``startup.m`` **in Matlab**, use:

.. code-block:: matlab

   edit(fullfile(userpath,'startup.m'))

If the file doesn't exist yet, click *yes* to create it.  In your
``startup.m``, you can add the ``addpath`` commands that you want
executed everytime your start Matlab:

.. note::

   In the following, the path to find the generated code varies based
   on the ROS and Matlab versions, make sure you use the path provided
   by Matlab at the end of the ``rosgenmsg`` call.

ROS 1:

.. code-block:: matlab

   % to locate crtk_msgs
   % some Matlab versions use the following
   addpath('~/catkin_ws_ws/src/crtk/matlab_msg_gen_XXXXXXXXXXX')
   % to locate crtk client
   addpath('~/catkin_ws/src/crtk/crtk_matlab_client')
   % to locate dvrk code - only for dVRK users
   addpath('~/catkin_ws/src/dvrk-ros/dvrk_matlab')

ROS 2:

.. code-block:: matlab

   % to locate crtk_msgs
   % some Matlab versions use the following
   addpath('~/ros2_ws/src/crtk/matlab_msg_gen')
   % to locate crtk client
   addpath('~/ros2_ws/src/crtk/crtk_matlab_client')
   % to locate dvrk code - only for dVRK users
   addpath('~/ros2_ws/src/dvrk/dvrk_matlab')

Then quit Matlab, restart it and test using:

.. code-block:: matlab

   clear classes
   rehash toolboxcache
   which startup
   % create a message (ROS 1)
   m = rosmessage('crtk_msgs/OperatingState')
   % create a message (ROS 2)
   m = ros2message('crtk_msgs/OperatingState')

*******************
Setting up a client
*******************

The first step is to create a Matlab class with dynamic properties.
For example, let's assume we want to create a simple force sensor
client:

.. code-block:: matlab

   classdef force_sensor < dynamicprops

The class should own an instance of ``crtk_utils``:

.. code-block:: matlab

   properties (Access = protected)
     crtk_utils;
   end

Then in the constructor, create an instance of ``crtk_utils`` and add
the CRTK features you need.  For example, if the device supports
``measured_cf``, use the method ``add_measured_cf()``.

.. code-block:: matlab

   methods
     function self = force_sensor(ros_namespace)
       self.crtk_utils = crtk.utils(self, ros_namespace);
       self.crtk_utils.add_measured_cf();
     end
   end

The method ``add_measured_cf`` will create the necessary ROS subscriber
and add a function handle (``measured_cf``) to the force sensor class.
Once this is done, you can create an instance of the force sensor and
call the method ``measured_cf``:

.. code-block:: matlab

   >> fs = force_sensor('optoforce/');
   >> cf = fs.measured_cf()
   cf =
      -0.0025   -0.0125    0.0775         0         0         0

If there are no messages on the CRTK topic subscribed to, you will get
a warning similar to:

.. code-block:: matlab

   >> cf = fs.measured_cf()
   Warning: measured_cf has not received messages yet (topic /optoforce/measured_cf)

This can be used to make sure you're using the right ROS topic name
and namespace.


**************
Using a client
**************

This example is based on the dVRK Matlab package.

.. code-block:: matlab

    ral = crtk.ral('test_arm_move');
    r = dvrk.arm(arm_name, ral);
    disp('---- Enabling (waiting up to 30s)');
    if ~r.enable(30.0)
        error('Unable to enable arm');
    end
    disp('---- Homing (waiting up to 30s)');
    if ~r.home(30.0)
        error('Unable to home arm');
    end

    % general settings
    rate = 200; % aiming for 200 Hz
    ros_rate = ral.rate(rate);

    % move_jp
    disp('---- Joint move');
    % move to 0 position
    joints_home = r.setpoint_js();
    joints_home(:) = 0.0;
    if (strcmp(arm_name, 'ECM') || strncmp(arm_name, 'PSM', 3))
        joints_home(3) = 0.12;
    end
    r.move_jp(joints_home).wait();
    % wiggle first two joints, matlab index starts at 1
    amplitude = deg2rad(10.0);
    % first move
    start = r.setpoint_js();
    goal = start;
    goal(1:2) = amplitude;
    r.move_jp(goal).wait();
    % second move
    goal = start;
    goal(1:2) = -amplitude;
    r.move_jp(goal).wait();

    disp('---- Joint servo');
    % move to 0 position
    r.move_jp(joints_home).wait();
    % wiggle first two joints, matlab index starts at 1
    amplitude = deg2rad(10.0);
    duration = 10.0; % seconds
    samples = duration * rate;
    % create a new goal starting with current position
    start = r.setpoint_js();
    goal = start;
    reset(ros_rate);
    for i = 0:samples
        goal(1) = start(1) + amplitude * (1.0 - cos(i * deg2rad(360.0) / samples));
        goal(2) = start(2) + amplitude * (1.0 - cos(i * deg2rad(360.0) / samples));
        r.servo_jp(goal);
        waitfor(ros_rate);
    end
