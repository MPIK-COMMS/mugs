/* 
 * Copyright (c) 2013, 2016 Max Planck Institute for Biological Cybernetics
 * All rights reserved.
 * 
 * This file is part of MUGS - Mobile and Unrestrained Gazetracking Software.
 *
 * MUGS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MUGS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MUGS.  If not, see <http://www.gnu.org/licenses/>. 
 */

#include "mug/lsl_interface.h"

#include "liblsl/lsl_cpp.h"
#include <stdlib.h>

using namespace lsl;
using namespace mug;

bool LslInterface::readFromLSL(bool calibrate, Sample &s)
{
    if(calibrate) 
    {
        // receive data and time stamps
        float headSample[6];
	float eyeSample[4];
	float dotSample[2];
	double head_ts = this->headInlet.pull_sample(&headSample[0], 6);
	double eye_ts = this->eyeInlet.pull_sample(&eyeSample[0], 4);
	double dot_ts = this->dotInlet.pull_sample(&dotSample[0], 2);
	
	// store data to Sample
	s.timestamp = eye_ts;
	s.H_pos[0] = headSample[0]; s.H_pos[1] = headSample[1]; s.H_pos[2] = headSample[2];
	s.H_o[0] = headSample[3]; s.H_o[1] = headSample[4]; s.H_o[2] = headSample[5];
	s.px_left = eyeSample[0]; s.py_left = eyeSample[1];
	s.px_right = eyeSample[2]; s.py_right = eyeSample[3];
	s.target_pos[0] = dotSample[0]; s.target_pos[1] = dotSample[1];
    }
    else
    {
        
    }
}
