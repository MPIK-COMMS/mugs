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
#include <vector>

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
        std::string headStreamName;                ///< Name of the stream that provides head coordinates
	std::string eyeStreamName;                 ///< Name of the stream inlet that provides eye coordinates
	std::string stimStreamName;                ///< Name of the stream inlet that provides stimulus coordinates
	int h_x, h_y, h_z, h_yaw, h_pitch, h_roll; ///< Channel number of head coordinates.
	int eLeft_x, eLeft_y, eRight_x, eRight_y;  ///< Channel number of eye coordinates.
	int stim_x, stim_y;                        ///< Channel number of stimulus coordinates
        
        /**
	 * \brief Constructor of the LslInterface class. Needs names of LSL streams for 
	 * head, eye and stimulus coordinates. The user can also specify which channel 
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
	    this->headStreamName = hName;
	    this->eyeStreamName = eName;
	    this->stimStreamName = sName;
	    this->h_x = headChannels[0]; this->h_y = headChannels[1]; this->h_z = headChannels[2];
	    this->h_yaw = headChannels[3]; this->h_pitch = headChannels[4]; this->h_roll = headChannels[5];
	    this->eLeft_x = eyeChannels[0]; this->eLeft_y = eyeChannels[1];
	    this->eRight_x = eyeChannels[2]; this->eRight_y = eyeChannels[3];
	    this->stim_x = stimChannels[0]; this->stim_y = stimChannels[1];
	}
	
	/**
	 * \brief Constructor of the LslInterface class. Needs names of LSL streams for 
	 * head, eye and stimulus coordinates. This constructor assigns default 
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
            this->headStreamName = hName;
	    this->eyeStreamName = eName;
	    this->stimStreamName = sName;
	    this->h_x = 0; this->h_y = 1; this->h_z = 2;
	    this->h_yaw = 3; this->h_pitch = 4; this->h_roll = 5;
	    this->eLeft_x = 0; this->eLeft_y = 1;
	    this->eRight_x = 2; this->eRight_y = 3;
	    this->stim_x = 0; this->stim_y = 1;
	}
	
	/**
	 * \brief Destructor.
	 */
	~LslInterface() {}
        
        /**
	 * \brief Fetch data of an experiment with a presented stimulus.
	 * \param[in] terminal Integer value that is send by the stimulus stream 
	 *                     to indicate the end of the experiment. (default = -100)
	 * \return Vector of mug::Sample objects.
	 */
	std::vector<Sample> fetchDataWStim(int terminal = -100);
	
	/**
	 * \brief fetch data of an experiment without a presented stimulus.
	 * \return Vector of mug::Sample objects
	 */
	std::vector<Sample> fetchDataWoStim();
	
    private:
        /**
	 * \brief Load one mug::Sample from LSL.
	 * \param[in] hInlet LSL stream_inlet object that provides access to 
	 *                   the head coordinates.
	 * \param[in] eInlet LSL stream_inlet object that provides access to
	 *                   the eye coordinates.
	 * \param[in] sInlet LSL stream_inlet object that provvides access to 
	 *                   the stimulus coordinates.
	 * \param[out] s mug::Sample to store data in.
	 */
        void readFromLSL(stream_inlet &hInlet, 
			 stream_inlet &eInlet,
			 stream_inlet &sInlet,
			 Sample &s);
	
	/**
	 * \brief Load one mug::Sample from LSL.
	 * \param[in] hInlet LSL stream_inlet object that provides access to 
	 *                   the head coordinates.
	 * \param[in] eInlet LSL stream_inlet object that provides access to
	 *                   the eye coordinates.
	 * \param[out] s mug::Sample to store data in.
	 */
        void readFromLSL(stream_inlet &hInlet, 
			 stream_inlet &eInlet,
			 Sample &s);
    };
}

#endif