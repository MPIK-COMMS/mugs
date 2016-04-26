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
        String headStreamName;   ///< Name of the stream that provides head coordinates
	String eyeStreamName;    ///< Name of the stream that provides eye coordinates
	String dotStreamName;    ///< Name of the stream that provides dot coordinates
        
        /**
	 * \brief Constructor of the LslInterface class. It assigns values to 
	 * headStreamName, eyeStreamName and dotStreamName.
	 * \param[in] hName Name of the stream that provides head coordinates.
	 * \param[in] eName Name of the stream that provides eye coordinates.
	 * \param[in] dName Name of the stream that provides dot coordinates.
	 */
        LslInterface(String hName, String eName, String dName)
	{
	  this->headStreamName = hName;
	  this->eyeStreamName = eName;
	  this->dotStreamName = dName;
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
        
    };
}

#endif