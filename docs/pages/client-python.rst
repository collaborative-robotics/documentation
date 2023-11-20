************
Installation
************

ROS 1
=====

To build the ROS 1 CRTK Client library, we recommend to use the catkin
build tools, i.e. ``catkin build``. You will need to clone this repository
as well as the repository with CRTK specific ROS messages:

.. code-block:: bash

   cd ~/catkin_ws/src   # or wherever your catkin workspace is
   git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
   git clone https://github.com/collaborative-robotics/crtk_python_client crtk/crtk_python_client
   catkin build

Once these packages are built, you should source your ``setup.bash``:
``source ~/catkin_ws/devel/setup.bash``. At that point, you should be able
to import the crtk python package in Python using import crtk.

ROS 2
=====

You will first need to clone this repository as well as the repository
with CRTK specific ROS messages:

.. code-block:: bash

   cd ~/ros2_ws/src   # or wherever your ROS 2 workspace is
   git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
   git clone https://github.com/collaborative-robotics/crtk_python_client crtk/crtk_python_client
   cd ~/ros2_ws;
   colcon build

Once these packages are built, you should source your ``setup.bash``:
``source ~/ros_ws/install/setup.bash``. At that point, you should be able
to import the crtk python package in Python using import crtk.

*******************
Setting up a client
*******************

Overview
========

You can find some examples in the scripts directory. Overall, the
approach is always the same, i.e. create a class and populate it with
``crtk.utils``. You can then use an instance of this class. For
example:

.. code-block:: python

    class crtk_move_cp_example:
        # constructor
        def __init__(self, device_namespace):
            # ROS initialization
            if not rospy.get_node_uri():
                rospy.init_node('crtk_move_cp_example', anonymous = True, log_level = rospy.WARN)
            # populate this class with all the ROS topics we need
            self.crtk_utils = crtk.utils(self, device_namespace)
            self.crtk_utils.add_measured_cp()
            self.crtk_utils.add_move_cp()

What is happening behind the scene:

* ``device_namespace`` is the ROS namespace used by the
  device. E.g. if the namespace is ``left``, we assume the device will
  have its CRTK ROS topics under ``/left``

* ``get_node_uri()`` and ``init_node()`` are not strictly needed but
  helps if the user did not properly initialize the ROS node

* Add an instance of ``crtk.utils`` in your class. The first parameter
  indicates which Python object should be "populated", i.e. which
  object will have the CRTK methods added to its dictionary

 * ``add_measured_cp()``:

   * Creates a subscriber for the topic, e.g. ``/left/measured_cp``

   * Registers a built-in callback for the topic. The callback will store the latest ``measured_cp`` ROS message in ``crtk_utils``

   * Provides a method to read the latest ``measured_cp`` message as a PyKDL frame

   * Adds the method ``measured_cp()`` to the user class (``crtk_move_cp_example``)

 * ``add_move_cp()``:

   * Creates a publisher for the topic, e.g. ``/left/move_cp``

   * Provides a method to send a PyKDL frame (goal), internally converts to a ROS message

   * Adds the method ``move_cp()`` to the user class (crtk_move_cp_example)

Once the class is defined, the user can use it:

.. code-block:: python

    example = crtk_move_cp_example('left')
    position = example.measured_cp()
    position.p[2] += 0.05 # move by 5 cm
    example.move_cp(position)


Operating state
===============

``crtk.utils.add_operating_state`` adds:

* State status ``operating_state()`` and helper queries: ``is_enabled()``, ``is_homed()``, ``is_busy()``

* State command ``operating_state_command()`` and helper commands: ``enable()``, ``disable()``, ``home()``, ``unhome()``

* Timer/event utilities:

  * For subscribers: ``wait_for_valid_data()``

  * For publishers (used by move commands): ``wait_for_busy()``

  * For state changes (used by ``enable()``, ``home()``...): ``wait_for_operating_state()``


Robot motion
============

``crtk.utils`` supports the following CRTK features:

* Subscribers:

  * ``add_setpoint_js``, ``add_setpoint_cp``

  * ``add_measured_js``, ``add_measured_cp``, ``add_measured_cv``, ``add_measured_cf``...

* Publishers:

  * ``add_servo_jp``, ``add_servo_jf``, ``add_servo_cp``, ``add_servo_cf``...

  * ``add_move_jp``, ``add_move_cp``

All methods relying on subscribers to get data have the following two optional parameters: ``age`` and ``wait``:

.. code-block:: python

    setpoint_cp(age = None, wait = None)

The parameter age specifies how old the data can be to be considered
valid and wait specifies how long to wait for the next message if the
data currently cached is too old. By default, both are based on the
expected interval provided when creating an instance of
``crtk.utils``. The expected interval should match the publishing rate
from the CRTK device. Setting the age to zero means that any cached
data should be used and the method shouldn't wait for new messages.

All move commands (``move_jp`` and ``move_cp``) return a ROS time
object. This is the time just before sending (i.e., publishing) the
move command to the device. This timestamp can be used to wait for
motion completion using:

.. code-block:: python

    # wait until robot is not busy, i.e. move has ended
    h = example.move_cp(goal) # record time move was sent
    h.wait()
    # compact syntax
    example.move_cp(goal).wait()
    # other example, wait until move has started
    example.move_cp(goal).wait(is_busy = True)

The method ``wait_for_busy()`` used by ``handle.wait()`` depends on the CRTK
device operating state and can be added to the example class using
``crtk.utils.add_operating_state``. See previous section.


**************
Using a client
**************

For the dVRK, one can use the classes ``dvrk.arm``, ``dvrk.psm``,
``dvrk.mtm``... that use the ``crtk.utils`` to provide as many
features as possible. This is convenient for general purpose testing,
for example in combination with iPython to test snippets of code. In
general, it is recommended to use your own class and only add the
features you need to reduce the number of ROS messages and callbacks.

The dVRK arm class implementation can be found in the "dvrk_python package.

Example of use:

.. code-block:: python

    import dvrk
    p = dvrk.arm('PSM1')
    p.enable()
    p.home()

    # get measured joint state
    [position, velocity, effort, time] = p.measured_js()
    # get only position
    position = p.measured_jp()
    # get position and time
    [position, time] = p.measured_jp(extra = True)

    # move in joint space
    import numpy
    p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))

    # move in cartesian space
    import PyKDL
    # start position
    goal = p.setpoint_cp()
    # move 5cm in z direction
    goal.p[2] += 0.05
    p.move_cp(goal).wait()

    import math
    # start position
    goal = p.setpoint_cp()
    # rotate tool tip frame by 25 degrees
    goal.M.DoRotX(math.pi * 0.25)
    p.move_cp(goal).wait()
