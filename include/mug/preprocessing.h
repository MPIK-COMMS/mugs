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
#include <algorithm>
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
     * \brief Checks for a given index whether this point is a extrema in the neighborhood 
     *        specified by order.
     * \param[in] data One-dimensional vector of floats representing function values.
     * \param[in] index Index of the value that is considered in this function call.
     * \param[in] order Range of the neighborhood for local extrema search.
     * \return True if the data point is a local extrema, false otherwise.
     */
    template<typename T>
    inline bool isLocalExtrema (std::vector<T> const& data, int index, int order)
    {
        // auxiliary variables
        typedef typename std::vector<T>::const_iterator extremaIt;
	bool min = true;
	bool max = true;
	
	// Set the begin point for the local neighborhood.
	// If the potential local extrema is at the beginning of the vector (less than order positions away),
	// the begin iterator is set to data.begin(). Otherwise, it is set to data.begin()+index-order.
	extremaIt begin = ((data.begin()+index) - data.begin() < order) ? data.begin() : data.begin()+index-order;
	
	// Set the end pointer for the local neighborhood.
	// If the potential local extrema is at the end of the vector (less than order positions away),
	// the end iterator is set to data.end(). Otherwise, it is set to data.begin()+index+order+1.
	extremaIt end = (data.end() - (data.begin()+index) < order) ? data.end() : data.begin()+index+order+1;
	
        for(extremaIt it = begin; it != end; ++it)
	{
	    // Skip the entry that we are currently considering as the potential local extrema
	    if (it != (data.begin()+index)) {
	        // Check whether the current value is greater, smaller, or equal to the potential local extrema
	        if(*it >= *(data.begin()+index)) {
	            max = false;
	        }
	        if(*it <= *(data.begin()+index)) {
	            min = false;
		}
	    }
	}
	return (max || min);
    }
    
    /**
     * \brief Finds the indices of local extrema within a one-dimensional vector.
     * \param[in] data One-dimensional vector of function values.
     * \param[in] order Number of neighbors that have to be smaller or greater
     *                  on both sides of a value in order to be marked as an extrema.
     * \param[out] extrema Vector of indices of local extrema.
     */
    inline void getLocalExtrema (std::vector<float> const& data, int order, std::vector<int> & extrema)
    {
        for(std::vector<float>::const_iterator it = data.begin(); it != data.end(); ++it)
        {
            int currentIndex = it - data.begin();
            if(isLocalExtrema(data, currentIndex, order)) {
	        extrema.push_back(currentIndex);
	    }
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
     * \param[in] order Size of the local neighborhood during local extrema search.
     * \param[in] remove If set to true all data points that occure betwenn target and fixation onset will be 
     *                   removed.
     * \return Vector of pairs of indices, which mark areas of data points that occured befor fixation onset.
     */
    std::vector<Eigen::Vector2i> onsetFilter_velocity (std::vector<Sample> & s, ModelType mt, int samplerate, int order, bool remove);
}

#endif