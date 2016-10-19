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

#include "liblsl/lsl_cpp.h"
#include <stdlib.h>
#include <stdio.h>

#include "mug/lsl_interface.h"
#include "mug/sample.h"

using namespace lsl;
using namespace mug;

void LslInterface::fetchDataWStim(Samples samples, int terminal)
{
    // resolve head stream
    std::cout << "[MUGS LslInterface] Resolving head tracker stream..." << std::endl;
    std::vector<stream_info> resultsHead = resolve_stream("name", this->headStreamName);
    std::cout << "  DONE\n" << std::endl;
    // resolve eye stream
    std::cout << "[MUGS LslInterface] Resolving eye tracker stream..." << std::endl;
    std::vector<stream_info> resultsEye = resolve_stream("name", this->eyeStreamName);
    std::cout << "  DONE\n" << std::endl;
    // resolve stimulus stream
    std::cout << "[MUGS LslInterface] Resolving stimulus stream..." << std::endl;
    std::vector<stream_info> resultsStim = resolve_stream("name", this->stimStreamName);
    std::cout << "  DONE\n" << std::endl;
    
    // get inlets for all three streams
    stream_inlet head(resultsHead[0]);
    stream_inlet eye(resultsEye[0]);
    stream_inlet stimulus(resultsStim[0]);
    
    Sample s;
    while(true)
    {
        this->readFromLSL(head, eye, stimulus, s);
	
	std::cout << s.H_pos[0] << " " << s.H_pos[1] << " " << s.H_pos[2] << " " 
	          << s.H_o[0] << " " << s.H_o[1] << " " << s.H_o[2] << " " 
		  << s.px_left << " " << s.px_right << " " 
		  << s.py_left << " " << s.py_right << " " 
		  << s.target_pos[0] << " " << s.target_pos[1] << "\n";
	
	if ((s.target_pos[0] == terminal) or (s.target_pos[1] == terminal))
	{
	    head.close_stream();
	    eye.close_stream();
	    stimulus.close_stream();
	    break;
	}
	
	samples.push_back(s);
    }
    
    std::cout << std::endl;
    
    return;
}

void LslInterface::fetchDataWoStim(Samples samples)
{
    // resolve head stream
    std::cout << "[MUGS LslInterface] Resolving head tracker stream..." << std::endl;
    std::vector<stream_info> resultsHead = resolve_stream("name", this->headStreamName);
    std::cout << "  DONE\n" << std::endl;
    // resolve eye stream
    std::cout << "[MUGS LslInterface] Resolving eye tracker stream..." << std::endl;
    std::vector<stream_info> resultsEye = resolve_stream("name", this->eyeStreamName);
    std::cout << "  DONE\n" << std::endl;
    
    // get inlets for both streams
    stream_inlet head(resultsHead[0]);
    stream_inlet eye(resultsEye[0]);
    
    Sample s;
    while(true)
    {
        this->readFromLSL(head, eye, s);
	samples.push_back(s);
    }
    
    return;
}


void LslInterface::readFromLSL(stream_inlet &hInlet, 
		               stream_inlet &eInlet,
		               stream_inlet &sInlet,
		               Sample &s)
{

    // receive data and time stamps
    float headSample[6];
    float eyeSample[4];
    float stimSample[2] = {-1, -1};
    double head_ts = hInlet.pull_sample(&headSample[0], 6);
    double eye_ts = eInlet.pull_sample(&eyeSample[0], 4);
    double stim_ts = sInlet.pull_sample(&stimSample[0], 2);

    // store data to Sample
    s.timestamp = eye_ts;
    s.H_pos[0] = headSample[this->h_x]; s.H_pos[1] = headSample[this->h_y]; s.H_pos[2] = headSample[this->h_z];
    s.H_o[0] = headSample[this->h_yaw]; s.H_o[1] = headSample[this->h_pitch]; s.H_o[2] = headSample[this->h_roll];
    s.px_left = eyeSample[this->eLeft_x]; s.py_left = eyeSample[this->eLeft_y];
    s.px_right = eyeSample[this->eRight_x]; s.py_right = eyeSample[this->eRight_y];
    s.target_pos[0] = stimSample[this->stim_x]; s.target_pos[1] = stimSample[this->stim_y];
}

void LslInterface::readFromLSL(stream_inlet &hInlet, 
		               stream_inlet &eInlet,
		               Sample &s)
{

    // receive data and time stamps
    float headSample[6];
    float eyeSample[4];
    double head_ts = hInlet.pull_sample(&headSample[0], 6);
    double eye_ts = eInlet.pull_sample(&eyeSample[0], 4);

    // store data to Sample
    s.timestamp = eye_ts;
    s.H_pos[0] = headSample[this->h_x]; s.H_pos[1] = headSample[this->h_y]; s.H_pos[2] = headSample[this->h_z];
    s.H_o[0] = headSample[this->h_yaw]; s.H_o[1] = headSample[this->h_pitch]; s.H_o[2] = headSample[this->h_roll];
    s.px_left = eyeSample[this->eLeft_x]; s.py_left = eyeSample[this->eLeft_y];
    s.px_right = eyeSample[this->eRight_x]; s.py_right = eyeSample[this->eRight_y];
}
