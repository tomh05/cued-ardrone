CUED AR.Drone Projects (2012-2013)
==================================

This is the shared repository containing program code and files relating to the
2012-2013 CUED AR.Drone Masters' projects.

Getting started
---------------

This repository is a ROS stack. It should be cloned into one of the directories in `ROS_PACKAGE_PATH`.

Clone the repository. For read-only access:

    $ git clone https://github.com/tomh05/cued-ardrone.git

For read-write access, after setting up a [GitHub SSH key](https://github.com/settings/ssh):

    $ git clone git@github.com:tomh05/cued-ardrone.git

This repository contains [git
submodules](http://git-scm.com/book/en/Git-Tools-Submodules). After cloning the
repository, and assuming that you are in the `cued-ardrone` directory:

    $ git submodule init
    $ git submodule update

See the [git submodule documentation](http://git-scm.com/book/en/Git-Tools-Submodules) for more information.

Build the AR.Drone driver. See the documentation in the `ardrone_autonomy` directory. The short version is:

    $ cd ardrone_autonomy
    $ ./build_sdk.sh
    $ rosmake

