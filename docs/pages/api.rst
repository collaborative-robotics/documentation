.. _API:

***
API
***

Overview
========

All payloads will have a standard header that includes at least a
timestamp. The timestamp is an absolute time, following the Unix
convention of time since January 1, 1970.

CRTK commands are separated in three main groups:

* Robot Operating State

  * Query the current state (e.g., is power on, is tool attached)

  * Initiate a state transition (e.g., enable power, disable power,
    specify tool)

* Robot Motion

  * Query the measured or desired robot position, velocity, or effort

  * Move the robot based on a desired position, velocity, or effort

* Status, warning, error messages (not defined yet!)


.. _API_operating_state:

Operating state
===============

.. include:: api-operating-state.rst

.. _API_robot_motion:

Robot motion
============

.. include:: api-robot-motion.rst
