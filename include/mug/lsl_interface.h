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

#ifndef LSLINTERFACE_H
#define LSLINTERFACE_H

#include "liblsl/lsl_cpp.h"

using namespace lsl;

namespace mug
{
 
    class Sample;
    
    /**
     * \brief Interface class to collect data from LSL. 
     * \author Jonas Ditz
     */
    class LslInterface
    {
    public:
        stream_inlet headInlet;                    ///< Stream inlet that provides head coordinates
	stream_inlet eyeInlet;                     ///< Stream inlet that provides eye coordinates
	stream_inlet stimInlet;                    ///< Stream inlet that provides stimulus coordinates
	int h_x, h_y, h_z, h_yaw, h_pitch, h_roll; ///< Channel number of head coordinates.
	int eLeft_x, eLeft_y, eRight_x, eRight_y;  ///< Channel number of eye coordinates.
	int stim_x, stim_y;                        ///< Channel number of stimulus coordinates
        
        /**
	 * \brief Constructor of the LslInterface class. It assigns values to 
	 * headInlet, eyeInlet and stimInlet. The user can also specify which channel 
	 * of the LSL streams corresponds to which coordinate. The channel numbers 
	 * can be specified using arrays of the form [x, y, z, yaw, pitch, roll] for 
	 * the head LSL stream channels, [left eye x, left eye y, right eye x, right eye y] 
	 * for the eye LSL stream channels and [x, y] for the stimulus LSL stream channels.
	 * \param[in] hName Name of the stream that provides head coordinates.
	 * \param[in] eName Name of the stream that provides eye coordinates.
	 * \param[in] sName Name of the stream that provides stimulus coordinates. If no stimulus 
	 *                  is used in the experiment use an empty string.
	 * \param[in] headChannels Array that stores the channel numbers for the head coordinates.
	 * \param[in] eyeChannels Array that stores the channel numbers for the eye coordinates.
	 * \param[in] stimChannels Array that stores the channel numbers for the stimlus coordinates.
	 */
        LslInterface(std::string hName, 
		     std::string eName, 
		     std::string sName,
	             int headChannels[6], 
		     int eyeChannels[4],
		     int stimChannels[2])
	{
	    this->headInlet(loadStream(hName));
	    this->eyeInlet(loadStream(eName));
	    if (sName != "") {this->stimInlet(loadStream(sName));}
	    this->h_x = headChannels[0]; this->h_y = headChannels[1]; this->h_z = headChannels[2];
	    this->h_yaw = headChannels[3]; this->h_pitch = headChannels[4]; this->h_roll = headChannels[5];
	    this->eLeft_x = eyeChannels[0]; this->eLeft_y = eyeChannels[1];
	    this->eRight_x = eyeChannels[2]; this->eRight_y = eyeChannels[3];
	    this->stim_x = stimChannels[0]; this->stim_y = stimChannels[1];
	}
	
	/**
	 * \brief Constructor of the LslInterface class. It assigns values to 
	 * headInlet, eyeInlet and stimInlet. This constructor assigns default 
	 * values to the LSL stream channel numbers. The channel numbers are:
	 *   x coordinate of head           at channel 0   of head LSL stream
	 *   y coordinate of head           at channel 1   of head LSL stream
	 *   z coordinate of head           at channel 2   of head LSL stream
	 *   yaw coordinate of head         at channel 3   of head LSL stream
	 *   pitch coordinate of head       at channel 4   of head LSL stream
	 *   roll coordinate of head        at channel 5   of head LSL stream
	 * 
	 *   x coordinate of the left eye   at channel 0   of eye LSL stream
	 *   y coordinate of the left eye   at channel 1   of eye LSL stream
	 *   x coordinate of the right eye  at channel 2   of eye LSL stream
	 *   y coordinate of the right eye  at channel 3   of eye LSL stream
	 * 
	 *   x coordinate of stimulus       at channel 0   of stimulus LSL stream
	 *   y coordinate of stimulus       at channel 1   of stimulus LSL stream
	 * \param[in] hName Name of the stream that provides head coordinates.
	 * \param[in] eName Name of the stream that provides eye coordinates.
	 * \param[in] sName Name of the stream that provides stimulus coordinates. If no stimulus 
	 *                  is used in the experiment use an empty string.
	 */
	LslInterface(std::string hName, 
		     std::string eName, 
		     std::string sName)
	{
            this->headInlet(loadStream(hName));
	    this->eyeInlet(loadStream(eName));
	    if (sName != "") {this->stimInlet(loadStream(sName));}
	    this->h_x = 0; this->h_y = 1; this->h_z = 2;
	    this->h_yaw = 3; this->h_pitch = 4; this->h_roll = 5;
	    this->eLeft_x = 0; this->eLeft_y = 1;
	    this->eRight_x = 2; this->eRight_y = 3;
	    this->stim_x = 0; this->stim_y = 1;
	}
	
	/**
	 * \brief Destructor that will close the streams.
	 */
	~LslInterface()
	{
	    this->headInlet.~stream_inlet();
	    this->eyeInlet.~stream_inlet();
	    this->eyeInlet.~stream_inlet();
	}
        
        /**
	 * \brief Load one \ref Sample from LSL.
	 * \param[in] calibrate Determine wether to read head, eye 
	 * and dot coordinates (calibrate == true) or only head and eye coordinates
	 * (calibrate == false).
	 * \param[out] s \ref Sample to store data in.
	 */
        void readFromLSL(bool calibrate, Sample &s);
	
    private:
        stream_inlet loadStream(std::string streamName)
	{
	    // resolve the stream of interest and make an inlet
	    std::vector<steam_info> results = resolve_stream("name", streamName);
	    return(results[0]);
	}
    };
}

#endif