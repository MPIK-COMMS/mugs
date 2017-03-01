<img src="doc/images/mug.png" align="right" height="150" />
# MUGS - Mobile and Unrestrained Gazetracking Software

## Introduction
MUGS provides both, geometric as well as regression methods, for calculating the gaze vector
of a subject and it's intersection with a planar display in a free moving scenario. It also 
provides helpful interfaces for recording and analyzing data.

## Installation - Linux

### Install prerequisites and dependencies:

#### CMake, Boost, Eigen3

```bash
sudo apt-get install cmake libboost-all-dev libeigen3-dev 
```

#### libGp

Download and install from:
https://github.com/mblum/libgp
    

#### Build library:

In mugs directory, type:
```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

If you want to build the tests, Google Test is needed. Tests can be build by replacing the cmake command above with
```bash
cmake -DBUILD_TESTS=ON ..
```

#### Build programs
In mugs directory, type:
```bash
cd programs/program_of_your_choice
mkdir build
cd build
cmake ..
make
```

## Contact

If you have questions or issues please contact us

- Lewis Chuang: lewis.chuang@tuebingen.mpg.de
- Jonas Ditz: jonas.ditz@tuebingen.mpg.de

## License

Copyright (c) 2013, 2017 Max Planck Institute for Biological Cybernetics <br>
All rights reserved.
 
MUGS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MUGS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MUGS.  If not, see <http://www.gnu.org/licenses/>.
