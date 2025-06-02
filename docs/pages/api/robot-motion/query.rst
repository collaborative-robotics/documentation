Query commands
##############

.. _measured_js:

``measured_js``
***************

Measured joint state.

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**
  * ``time header.stamp``: time of measurement [*required*]

  * ``string header.frame_id``: reference frame (this should match the
    ``header.frame_id`` string in ``measured_cp``) [*required*]

  * ``string name[]``: array of joint names [*required*]

  * ``float64 position[]``: array of measured joint positions [*optional*]

  * ``float64 velocity[]``: array of measured joint velocities [*optional*]

  * ``float64 effort[]``: array of measured joint efforts [*optional*]

* **Notes:**

  * velocity should be provided if the low-level controller has the
    ability to perform velocity estimation

  * effort should be provided if the low-level controller has torque
    sensors or current feedback per joints


.. _measured_cp:

``measured_cp``
***************

Measured cartesian position.

* **Payload:** ``geometry_msgs/PoseStamped`` (before 01/2022 payload
  was ``geometry_msgs/TransformStamped``, see `issue #1 <https://github.com/collaborative-robotics/documentation/issues/1>`_)

* **Specification:**

  * ``time header.stamp``: time of measurement, if the measured
    cartesian position is based on a measured joint position, the time
    stamp should be the same as ``measured_js`` [*required*]

  * ``string header.frame_id``: reference frame [*required*]

  * ``string frame_id``: moving frame [*not available on ROS*]

  * ``Transform transform``: translation and rotation for the measured
    cartesian position (e.g. forward kinematics based on measured
    joint position from :ref:`measured_js`) [*required*]


.. _measured_cv:

``measured_cv``
***************

Measured cartesian velocity (twist).

* **Payload:** ``geometry_msgs/TwistStamped``

* **Specification:**

  * ``time header.stamp``: time of measurement, if the measured twist
    is based on a measured joint velocity, the time stamp should be
    the same as ``measured_js`` [*required*]

  * ``string header.frame_id``: reference frame, see :ref:`measured_cp` [*required*]

  * ``string frame_id``: moving frame, see :ref:`measured_cp` [*not available on ROS*]

  * ``Twist twist``: linear and angular components for the measured
    cartesian velocity (e.g. Jacobian applied to measured joint
    velocities) [*required*]

* **Notes:**

  * This command will not be available if the low-level controller
    doesn't have a way to estimate joint velocity.


.. _measured_cf:

``measured_cf``
***************

Measured cartesian force (wrench).

* **Payload:** ``geometry_msgs/WrenchStamped``

* **Specification:**

  * ``time header.stamp``: time of measurement, if the measured wrench
    is based on a measured joint efforts, the time stamp should be the
    same as measured_js [*required*]

  * ``string header.frame_id``: reference frame, see :ref:`measured_cp` [*required*]

  * ``string frame_id``: moving frame, see :ref:`measured_cp` [*not available on ROS*]

  * ``Wrench wrench``: force and torque components for the measured
    cartesian wrench (e.g. Jacobian applied to measured joint efforts)
    [*required*]

* **Notes:**

  * This command will not be available if the low-level controller
    doesn't have a way to estimate joint efforts.


.. _setpoint_js:

``setpoint_js``
***************

Joint setpoint (low-level controller).

* **Payload:** `sensor_msgs/JointState`

* **Specification:**

  * ``time Header.stamp``: time associated to last servo command. This
    can be defined by a direct servo command or an intermediary set
    point calculated by interpolate or move.

  * ``string header.frame_id``: reference frame, see :ref:`measured_js` [*required*]

  * ``string name[]``: array of joint names [*required*]

  * ``float64 position[]``: array of setpoint joint positions [see notes]

  * ``float64 velocity[]``: array of setpoint joint velocities [see notes]

  * ``float64 effort[]``: array of setpoint joint efforts [see notes]

* **Notes:**

  * At least one of the 3 vectors (position, velocity and effort) should be used.

  * Generally, this message should return *at least* the setpoint
    corresponding to the last motion command (for example, a
    ``position`` setpoint if the last motion specified a
    position). Depending on the type of low-level controller, other
    quantities could be included, as in the following examples:

    * If the controller hardware implements motor current (torque or
      effort) control, the low-level controller will convert a
      specified ``position`` or ``velocity`` setpoint to an ``effort``
      setpoint, ``F_llc``. In this case, the effort setpoint can also
      be included.

    * If the controller hardware implements position control, the
      low-level controller will integrate a ``velocity`` setpoint to
      become a ``position`` setpoint. In this case, both the
      ``position`` and ``velocity`` setpoint could be included.

  * When using the commands ``interpolate`` or ``move``, ``position``
    and ``velocity`` at time ``t`` should be computed by the
    interpolator or the trajectory generator (``p(t)`` and ``v(t)``).

  * If the command is defined in cartesian space, the corresponding
    joint space value should be provided (inverse kinematics for
    ``position``, use Jacobian for ``velocity`` and ``effort``).

  * Summary:

    .. list-table::
       :widths: 30 20 20 20
       :header-rows: 1

       * - **command level**
         - **``position``**
         - **``velocity``**
         - **``effort``**
       * - ``servo_{j,c}p``
 	 - ``setpoint``
 	 - n/a
 	 - ``F_llc`` or n/a
       * - ``servo_{j,c}v``
 	 - n/a
 	 - setpoint
 	 - ``F_llc`` or n/a
       * - ``servo_{j,c}f``
 	 - n/a
 	 - n/a
 	 - setpoint
       * - ``interpolate_{j,c}p``
 	 - ``p(t)``
 	 - ``V_llc`` or ``v(t)``
 	 - ``F_llc`` or n/a
       * - ``interpolate_{j,c}v``
 	 - n/a
 	 - ``V_llc`` or ``v(t)``
 	 - ``F_llc`` or n/a
       * - ``interpolate_{j,c}f``
 	 - n/a
 	 - n/a
 	 - ``f(t)``
       * - ``move_{j,c}p``
 	 - ``p(t)``
 	 - ``V_llc`` or ``v(t)``
 	 - ``F_llc`` or n/a


.. _setpoint_cp:

``setpoint_cp``
***************

Cartesian position setpoint (low-level controller).

* **Payload:** ``geometry_msgs/PoseStamped`` (before 01/2022 payload
  was ``geometry_msgs/TransformStamped``, see  `issue #1 <https://github.com/collaborative-robotics/documentation/issues/1>`_)

* **Specification:**

  * ``time header.stamp``: see ``setpoint_js`` [*required*]

  * ``string header.frame_id``: reference frame [*required*]

  * ``string frame_id``: moving frame [required]

  * ``Transform transform``: translation and rotation for the
    commanded cartesian position (e.g. forward kinematics based on
    joint positions from ``setpoint_js``) [*required*]

* **Notes:**

  * This query is valid only if the ``position`` field in
    ``setpoint_js`` is valid, i.e. when the motion commands are
    position based, i.e. ``{servo,interpolate,move}_{j,c}p``. For all
    other motion commands, the data should be marked as invalid by
    zeroing the ``time header.stamp``.


.. _setpoint_cv:

``setpoint_cv``
***************

Cartesian velocity setpoint (low-level controller).

See :ref:`setpoint_cp` and :ref:`setpoint_js`.


.. _setpoint_cf:

``setpoint_cf``
***************

Cartesian force setpoint (low-level controller).

See :ref:`setpoint_cp` and :ref:`setpoint_js`.


.. _goal_js:

``goal_js``
***********

Joint goal (mid-level controller).

This command is not fully specified yet. It should at least report
the end goal from ``{interpolate,move}_{c,j}{p,v,f}``.


.. _goal_cp:

``goal_cp``
***********

Cartesian position goal (mid-level controller).

See :ref:`goal_js`


.. _goal_cv:

``goal_cv``
***********

Cartesian velocity goal (mid-level controller).

This command is not fullt specified yer. It should at least report
the goal from ``interpolate_{j,c}v``
