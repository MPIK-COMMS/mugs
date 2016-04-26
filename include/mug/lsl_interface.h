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
 
    class Sample;
    
    /**
     * \brief Interface class to collect data from LSL, directly. 
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
	 * headInlet, eyeInlet and stimInlet.
	 * \param[in] hName Name of the stream that provides head coordinates.
	 * \param[in] eName Name of the stream that provides eye coordinates.
	 * \param[in] sName Name of the stream that provides stimulus coordinates.
	 */
        LslInterface(std::string hName, std::string eName, std::string sName,
	             std::array<int> headChannels = [0,1,2,3,4,5], 
		     std::array<int> eyeChannels  = [1,2,3,4],
		     std::array<int> stimChannels = [0,1])
	{
	    this->headInlet(loadStream(hName));
	    this->eyeInlet(loadStream(eName));
	    this->stimInlet(loadStream(sName));
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
	 * \return True if the reading was successful, false otherwise.
	 */
        bool readFromLSL(bool calibrate, Sample &s);
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