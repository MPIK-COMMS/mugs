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

#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <mug/eye_model.h>

namespace mug 
{
    
    /**
     * \brief This function converts Euclidean coordinates into Polar coordinates.
     * \param[in] x X coordinate of the point.
     * \param[in] y Y coordinate of the point.
     * \return Polar coordinates (theta, r) of the point.
     */
    inline Eigen::Vector2f cart2pol (float x, float y)
    {
        Eigen::Vector2f pol;
        pol[0] = atan2(x,y);   // calculate theta
        pol[1] = hypot(x,y);   // calculate r
        return pol;
    }
    
    /**
     * \brief Simple approach to calculates the first derivative (velocities) of a timeseries
     *        of points.
     * \param[in] points Timeseries for which the derivative should be calculated.
     * \param[out] derivative First derivative (velocities) of the timeseries of points.
     */
    template<typename T>
    inline void simpleDerivative (std::vector<T> & points,
                           std::vector<T> & derivative)
    {
        derivative.push_back(0);
        for (typename std::vector<T>::iterator it = points.begin(), prev=points.end(); it != points.end();prev=it, ++it)
        {
	    // skip first
	    if(it == points.begin()){continue;}
	    derivative.push_back(*(it) - *(prev));
        }
    }
    
    /**
     * \brief Remove of all marker changes that occured during smooth persuits.
     * \param[out] marker Vector of indices of marker changes.
     */
    inline void removeSmoothPersuitMarker(std::vector<int> & marker)
    {
        for (std::vector<int>::iterator itMarker = marker.begin(); itMarker != marker.end(); )
	{
	    int currentIndex = *itMarker;
	    // peek the next index
	    std::vector<int>::iterator itNext = itMarker;
	    int nextIndex = *(++itNext);
	    if (nextIndex - currentIndex == 1) {
	        itMarker = marker.erase(itMarker);
	    } else {
	        ++itMarker;
	    }
	}
    }
        
    /**
     * \brief Remove all samples that are specified by the vector of indices 
     *        given as a second argument.
     * \param[out] s The samples are removed from this Samples object.
     * \param[in] removableAreas Vector of indices, that specifies which 
     *            samples should be removed.
     */
    inline void removeSamples (std::vector<Sample> & s, std::vector<Eigen::Vector2i> removableAreas)
    {
        for (std::vector<Eigen::Vector2i>::iterator it = removableAreas.begin(); it != removableAreas.end(); ++it)
        {
            s.erase(s.begin() + (*it)[0], s.begin() + (*it)[1]);
        }
    }
    
    /**
     * \brief This function corrects Polar coordinates of the eye for artifacts, which occure 
     *        due to the conversion of Euclidean coordinates into Polar cordinates.
     * \param[out] theta Vector of float that stores the theta value of polar coordinates 
     *             of the eye.
     */
    void correctPolArtifacts (std::vector<float> & theta);

    /**
     * \brief Calculates the mean position of both eyes. Additionally,
     *        this function determines the indices of all position changes 
     *        of the marker.
     * \param[in] s Samples object that stores all samples.
     * \return Mean position of eye coordinates for both eyes.
     */
    Eigen::Vector4f meanPosAndMarkerChanges (std::vector<Sample> & s, std::vector<int> & makerChanges);
    
    /**
     * \brief Filter that identifies sample points, which are recorded before the subject actually look at 
     *        the target (remove fixation onset). The filtering is done by a velocity based approach.
     * \param[out] s Samples object that stores all samples.
     * \param[in] mt ModelType to specify, which eye should be used to filter the samples.
     * \param[in] samplerate Integer that indicates the samplerate for the used sample object.
     * \param[in] remove If set to true all data points that occure betwenn target and fixation onset will be 
     *                   removed.
     * \return Vector of pairs of indices, which mark areas of data points that occured befor fixation onset.
     */
    std::vector<Eigen::Vector2i> onsetFilter_velocity (std::vector<Sample> & s, ModelType mt, int samplerate, bool remove);
}

#endif