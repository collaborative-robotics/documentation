Motion commands
###############

.. _servo_jp:

``servo_jp``
************

Set position joint setpoint (low-level).

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js`` [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but recommended*]

  * ``float64 position[]``: array of setpoint joint positions [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]

* **Notes:**

  * These commands are pre-emptive, the latest command received will
    set the position setpoint used by the low-level controller.

  * `stamp`, `frame_id` and `name` are not used by the command so they
    could be left empty. It is nevertheless recommended to use them
    for data collection or further validation.


.. _servo_jr:

``servo_jr``
************

Set position joint relative setpoint (low-level).
* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js`` [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but recommended*]

  * ``float64 position[]``: array of setpoint joint relative position [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]

* **Notes:** See :ref:`servo_jp`.


.. _servo_jv:

``servo_jv``
************

Set velocity joint setpoint (low-level).

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js`` [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but recommended*]

  * ``float64 position[]``: [*not used*]

  * ``float64 velocity[]``: array of setpoint joint velocities [*required*]

  * ``float64 effort[]``: [*not used*]

* **Notes:** See :ref:`servo_jp`.


.. _servo_jf:

``servo_jf``
************

Set effort joint setpoint (low-level).


.. _servo_cp:

``servo_cp``
************

Set position cartesian setpoint (low-level)


.. _servo_cr:

``servo_cr``
************

Set position cartesian relative setpoint (low-level)


.. _servo_cv:

``servo_cv``
************

Set velocity cartesian setpoint (low-level)


.. _servo_cf:

``servo_cf``
************

Set effort cartesian setpoint (low-level)


.. _interpolate_jp:

``interpolate_jp``
******************

Set position joint goal (with interpolation). See :ref:`servo_jp`.


.. _interpolate_jr:

``interpolate_jr``
******************

Set position joint relative goal (with interpolation). See :ref:`servo_jr`.


.. _interpolate_js:

``interpolate_jv``
******************

Set velocity joint goal (with interpolation). See :ref:`servo_jv`.


.. _interpolate_jf:

``interpolate_jf``
******************

Set effort joint goal (with interpolation). See :ref:`servo_jf`.


.. _interpolate_cp:

``interpolate_cp``
******************

Set position cartesian goal (with interpolation). See :ref:`servo_cp`.


.. _interpolate_cr:

``interpolate_cr``
******************

Set position cartesian relative goal (with interpolation). See  :ref:`servo_cr`


.. _interpolate_cv:


``interpolate_cv``
******************

Set velocity cartesian goal (with interpolation). See :ref:`servo_cv`.


.. _interpolate_cf:

``interpolate_cf``
******************

Set effort cartesian goal (with interpolation).  See :ref:`servo_cf`.


.. _move_jp:

``move_jp``
***********

Set position joint goal (with trajectory generation).

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command
    [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js``
    [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but
    recommended*]

  * ``float64 position[]``: array of goal joint positions [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]


.. _move_jr:

``move_jr``
***********

Set position joint relative goal (with trajectory generation).

* **Payload:** ``sensor_msgs/JointState``

* **Specification:**

  * ``time Header.stamp``: time associated to the ``servo`` command
    [*not used but recommended*]

  * ``string header.frame_id``: reference frame, see ``measured_js``
    [*not used but recommended*]

  * ``string name[]``: array of joint names [*not used but
    recommended*]

  * ``float64 position[]``: array of goal joint relative positions
    [*required*]

  * ``float64 velocity[]``: [*not used*]

  * ``float64 effort[]``: [*not used*]


.. _move_cp:

``move_cp``
***********

Set position cartesian goal (with trajectory generation). See :ref:`servo_cp`.


.. _move_cr:

``move_cr``
***********

Set position cartesian relative goal (with trajectory generation). See :ref:`servo_cr`.
