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
	float stimSample[2];
	double head_ts = this->headInlet.pull_sample(&headSample[0], 6);
	double eye_ts = this->eyeInlet.pull_sample(&eyeSample[0], 4);
	double stim_ts = this->stimInlet.pull_sample(&stimSample[0], 2);
	
	// store data to Sample
	s.timestamp = eye_ts;
	s.H_pos[0] = headSample[this->h_x]; s.H_pos[1] = headSample[this->h_y]; s.H_pos[2] = headSample[this->h_z];
	s.H_o[0] = headSample[this->h_yaw]; s.H_o[1] = headSample[this->h_pitch]; s.H_o[2] = headSample[this->h_roll];
	s.px_left = eyeSample[this->eLeft_x]; s.py_left = eyeSample[this->eLeft_y];
	s.px_right = eyeSample[this->eRight_x]; s.py_right = eyeSample[this->eRight_y];
	s.target_pos[0] = stimSample[this->stim_x]; s.target_pos[1] = stimSample[this->stim_y];
    }
    else
    {
        // receive data and time stamps
        float headSample[6];
	float eyeSample[4];
	double head_ts = this->headInlet.pull_sample(&headSample[0], 6);
	double eye_ts = this->eyeInlet.pull_sample(&eyeSample[0], 4);
	
	// store data to Sample
	s.timestamp = eye_ts;
	s.H_pos[0] = headSample[this->h_x]; s.H_pos[1] = headSample[this->h_y]; s.H_pos[2] = headSample[this->h_z];
	s.H_o[0] = headSample[this->h_yaw]; s.H_o[1] = headSample[this->h_pitch]; s.H_o[2] = headSample[this->h_roll];
	s.px_left = eyeSample[this->eLeft_x]; s.py_left = eyeSample[this->eLeft_y];
	s.px_right = eyeSample[this->eRight_x]; s.py_right = eyeSample[this->eRight_y];
	s.target_pos[0] = -1; s.target_pos[1] = -1;
    }
}
