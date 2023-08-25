.. _API_robot_motion:

General concepts
================

Motions
*******

We identified the following basic types of motions for collaborative
robotics. All these commands can be defined either in the cartesian or
joint space.

``servo``
+++++++++

Direct access to the low-level controller:

* **Use cases:**

  * User has a smooth and continuous trajectory coming from a master
    arm or recorded trajectory and can send commands at a high rate.
    
  * Closing the loop in velocity mode using a fast external sensor
    with a task based Jacobian.
    
  * Haptic feedback on master arm.

* **Type:** These commands can be either position, velocity or effort
  based. Positions can be provided relative from the latest setpoint
  position.

* **Continuity:** Users should send continuous commands. The low-level
  controller is not required to generate intermediate setpoints to
  ensure that the command is feasible (i.e., setpoint should be close
  to current state). The low-level controller can enforce limits,
  e.g., reject a command if the difference from the previous command
  is greater than a defined threshold.

* **Time:** Users are expected to send commands periodically at a rate
  close to the low-level rate. These commands are preemptive.

``interpolate``
+++++++++++++++

Simple interpolation:

* **Use cases:**
  
  * User has a smooth trajectory coming from a master arm or recorded
    trajectory, but **cannot** send commands at a high rate (e.g., 50Hz
    visual tracking, remote tele-operation).
    
  * In general, similar usage as ``servo` but the application can not
    send commands fast enough to provide a smooth command so the
    low-level controller needs to interpolate the user commands
    (smooths but adds latency).
    
* **Type:** These commands can be either position, velocity or effort
  based. Positions can be provided relative from the latest setpoint
  position.
  
* **Continuity**: Users must send continuous commands. The low-level
  controller will compute intermediary positions to smooth the motion.
  
* **Time:** Users are expected to send commands periodically at a rate
  lower than the low-level controller. The velocity of the motion is
  defined by the user commands. These commands are preemptive.
