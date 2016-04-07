# MUGS - Mobile and Unrestrained Gazetracking Software
![MUGS logo](doc/images/mug.png)

## Installation - Linux

### Install prerequisites and dependencies:

#### CMake, Boost, Eigen3

```bash
sudo apt-get install cmake-curses-gui libboost-all-dev libeigen3-dev 
```

#### libGp

Download and install from:
https://bitbucket.org/mblum/libgp
    

#### Build library:

In mug directory, type:
```bash
cmake .
make -j4
sudo make install
```

### Build and run example (optional):
```bash
cd examples
cmake .
make -j4
./train_model
```




