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

namespace mug 
{
    class Sample;
    enum ModelType;
    
    /**
     * \brief This function converts Euclidean coordinates into Polar coordinates.
     * \param[in] x X coordinate of the point.
     * \param[in] y Y coordinate of the point.
     * \return Polar coordinates (theta, r) of the point.
     */
    Eigen::Vector2f cart2pol (float x, float y)
    {
        Vector2f pol;
        pol[0] = atan2(x,y);   // calculate theta
        pol[1] = hypot(x,y);   // calculate r
        return pol;
    }
    
    /**
     * \brief This function corrects Polar coordinates of the eye for artifacts, which occure 
     *        due to the conversion of Euclidean coordinates into Polar cordinates.
     * \param[out] pol Vector of Eigen::Vector2f that stores the Polar coordinates 
     *             of the eye.
     */
    void correctPolArtifacts (std::vector<Eigen::Vector2f> & pol);
    
    /**
     * \brief Simple approach to calculates the first derivative (velocities) of a timeseries
     *        of points.
     * \param[in] points Timeseries for which the derivative should be calculated.
     * \param[out] derivative First derivative (velocities) of the the timeseries of points.
     */
    void simpleDerivative (std::vector<Eigen::Vector2f> & points,
                           std::vector<float> & derivative)
    {
        derivative.push_back(0);
        for (std::vector<Eigen::Vector2f>::iterator it = points.begin()+1; it != points.end(); ++it)
        {
	    derivative.push_back((*it)[0] - (*(it-1))[0]);
        }
    }

    /**
     * \brief Calculates the mean position of both eyes. Additionally,
     *        this function determines the indices of all position changes 
     *        of the marker.
     * \param[in] s Samples object that stores all samples.
     * \return Mean position of eye coordinates for both eyes.
     */
    Eigen::Vector4f meanPosAndMarkerChanges (std::vector<Sample> & s, std::vector<int> & makerChanges);
    
    /**
     * \brief Remove all samples that are specified by the vector of indices 
     *        given as a second argument.
     * \param[out] s The samples are removed from this Samples object.
     * \param[in] removableAreas Vector of indices, that specifies which 
     *            samples should be removed.
     */
    void removeSamples (std::vector<Sample> & s, std::vector<int[2]> removableAreas)
    {
        for (std::vector<int[2]>::iterator it = removableAreas.begin(); it != removableAreas.end(); ++it)
        {
            s.erase(s.begin() + *it[0], s.begin() + *it[1]);
        }
    }
    
    /**
     * \brief Filter that identifies sample points in a Samples object, which belong to a saccade.
     *        The filtering is done by a velocity based approach.
     * \param[out] s Samples object that stores all samples.
     * \param[in] mt ModelType to specify, which eye should be used to filter the samples.
     * \param[in] remove Boolean value. Set to true to remove sample points that
     *            are identified to be saccadic.
     * \return Vector of indices, which belong to saccadic sample points.
     */
    std::vector<int> saccadeFilter_velocity (std::vector<Sample> & s, ModelType mt, bool remove);
}

#endif