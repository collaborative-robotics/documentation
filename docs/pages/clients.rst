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
  and the "proxy" class.

The class ``crtk.utils`` is designed to add CRTK features "a la
carte", i.e. it doesn't assume that all CRTK features are
available. This allows to:

* Match only the features that are available on the CRTK devices one
  wants to use (server side)

* Reduce the number of features to those strictly needed for the
  application (client side). Reducing the number of ROS topics used
  helps in terms of performance.

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
