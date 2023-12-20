.. _clients:

################
Client Libraries
################

The client libraries provide some tools to facilitate the development
of a CRTK compatible client over ROS, i.e. create a "proxy" class to
communicate with an existing CRTK compatible ROS device.  We currently
support two client libraries, one for Python and one for Matlab.

The main class in the client library is ``crtk.utils``. It can be used
to quickly populate an existing class by adding CRTK like
methods. These methods will handle the following for you:

* Declare all required ROS publishers and wrap publisher calls in
  methods to send data to the device

* Declare all required ROS subscribers and provide callbacks to
  receive the data from the device

* Convert ROS messages to more convenient data types.  The Python
  client use numpy arrays for joint values and PyKDL types for
  cartesian data.  The Matlab client uses vectors and matrices.

* Some events to manage asynchronous communication between the device
  and the "proxy" class (e.g. end of ``move`` command, change of
  state...)

The class ``crtk.utils`` is designed to add CRTK features "a la
carte", i.e. it doesn't assume that all CRTK features are
available. This allows to:

* Match only the features that are available on the CRTK devices one
  wants to use (server side)

* Reduce the number of features to those strictly needed for the
  application (client side). Reducing the number of ROS topics used
  helps in terms of performance.

In version 1.2, ``crtk`` also includes a ROS Abstraction Layer.  The
``ral`` goal is to hide which version of ROS is used so the end-user
script can run with either ROS 1 or ROS 2.  The ``ral`` doesn't cover
all the ROS features (either in Python or Matlab) so not all scripts
can be totally portable.

.. note::

   As of December 2023, the Python client libraries have been used
   extensively and are well tested.  The Matlab version is not as
   mature.  More specifically the RAL implementation is buggy on ROS 2
   and missing in the ROS 1 implementation.

.. _client_python:

########################
Client Libraries: Python
########################

.. include:: client-python.rst

.. _client_matlab:

########################
Client Libraries: Matlab
########################

.. include:: client-matlab.rst
