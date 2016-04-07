# MUGS - Mobile and Unrestrained Gazetracking Software
<img src="doc/images/mug.png" align="center" />

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

## Contact

If you have questions or issues please contact us

- Lewis Chuang: lewis.chuang@tuebing.mpg.de
- Jonas Ditz: jonas.ditz@tuebingen.mpg.de

## License

Copyright (c) 2013, 2016 Max Planck Institute for Biological Cybernetics
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
