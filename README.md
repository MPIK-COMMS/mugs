==============================================================================
 MUG - Library for Mobile and Unrestrained Gazetracking
==============================================================================

------------------------------------------------------------------------------
 Installation - Linux
------------------------------------------------------------------------------

1. Install prerequisites and dependencies:

* CMake, Boost, Eigen3

    sudo apt-get cmake-curses-gui libboost-all-dev libeigen3-dev 

* libGp

    Download and install from:
    https://bitbucket.org/mblum/libgp
    

2. Build library:

In mug directory, type:

    ccmake .
    make -j4
    sudo make install


3. Build and run example (optional):

    cd examples
    ccmake .
    make -j4

    ./train_model





