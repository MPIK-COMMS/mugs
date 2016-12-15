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

#include <mug/preprocessing.h>

#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <mug/sample.h>
#include <mug/eye_model.h>

using namespace Eigen;
using namespace mug;

void mug::correctPolArtifacts(std::vector<Eigen::Vector2f> & pol)
{
    // threshold to decide whether the current jump is a polar artifact
    float threshold = 2 * M_PI - 0.15;
    for (std::vector<Eigen::Vector2f>::iterator it = pol.begin()+1; it != pol.end(); ++it)
    {
        float velo = std::abs((*it)[0] - (*(it-1))[0]);
	if (velo >= threshold) {
	    if ((*it)[0] > 0){
	        (*it)[0] -= 2 * M_PI;
	    } else {
	        (*it)[0] += 2 * M_PI;
	    }
	}
    }
}

Vector4f mug::meanPosAndMarkerChanges (std::vector<Sample> & s, std::vector<int> & markerChanges)
{
    float center_px_left = 0, center_py_left = 0, center_px_right = 0, center_py_right = 0;
    Vector4f meanPosition;
    int sampleSize = s.size();
    Vector2f currentPosition = s[0].target_pos;
    
    // iterate over samples and sum up the eye coordinates as well as fill array of 
    // marker position changes. Ignore marker changes that occured during smooth persuits
    for(Samples::iterator it = s.begin(); it != s.end(); ++it)
    {
        if (it->target_pos != currentPosition)
	{
	    markerChanges.push_back(it - s.begin());
	    currentPosition = it->target_pos;
	}
        center_px_left += it->px_left;
        center_py_left += it->py_left;
        center_px_right += it->px_right;
        center_py_right += it->py_right;
    }
    
    // calculate mean value of x and y coordinate for both eyes
    meanPosition[0] = center_px_left / sampleSize;
    meanPosition[1] = center_py_left / sampleSize;
    meanPosition[2] = center_px_right / sampleSize;
    meanPosition[3] = center_py_right / sampleSize;
    return meanPosition;
}

std::vector<int> mug::saccadeFilter_velocity (std::vector<Sample> & s, ModelType mt, bool remove)
{
    // Get mean eye positions and points of changes of the marker position.
    std::vector<int> markerChanges;
    Vector4f meanPos = meanPosAndMarkerChanges(s, markerChanges);
    
    // Convert centered Cartesean into polar coordinates
    std::vector<Vector2f> polar_eye;
    if(mt == EYE_RIGHT)
    {
        for(Samples::iterator it = s.begin(); it != s.end(); ++it)
        {
            polar_eye.push_back(cart2pol(it->px_right - meanPos[2], it->py_right - meanPos[3]));
        }
    }else{
        for(Samples::iterator it = s.begin(); it != s.end(); ++it)
        {
            polar_eye.push_back(cart2pol(it->px_left - meanPos[0], it->py_left - meanPos[1]));
        }
    }
    
    // remove artifacts caused by conversion into polar coordinates
    correctPolArtifacts(polar_eye);
    
    // calculate first derivative of theta
    std::vector<float> derivative;
    simpleDerivative(polar_eye, derivative);
    
    //TODO finish filter method
    return markerChanges; //this is just a auxiliary return statement! Needs to be changed in the final version
}
