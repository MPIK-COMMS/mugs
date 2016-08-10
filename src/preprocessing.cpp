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

#endif

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <mug/sample.h>
#include <mug/eye_model.h>

using namespace Eigen;
using namespace mug;

Vector2f cart2pol(float x, float y)
{
    Vector2f pol;
    pol[0] = atan2(x,y);   // calculate theta
    pol[1] = hypot(x,y);   // calculate r
    return pol;
}

Vector4f meanPosAndMarkerChanges (const std::vector<Sample> & s, std::vector<int> & markerChanges)
{
    float center_px_left, center_py_left, center_px_right, center_py_right;
    Vector4f meanPosition;
    int sampleSize = s.size();
    int index = 0;
    Vector2f currentPosition = s[0].target_pos;
    
    // iterate over samples and sum up the eye coordinates as well as fill array of 
    // marker position changes.
    for(Samples::iterator it = s.begin(); it != s.end(); ++it)
    {
        if (*it.target_pos != currentPosition)
	{
	    markerChanges.push_back(index);
	}
        center_px_left += *it.px_left; 
        center_py_left += *it.py_left;
        center_px_right += *it.px_right;
        center_py_right += *it.py_right;
        index += 1;
    }
    
    // calculate mean value of x and y coordinate for both eyes
    meanPosition[0] = center_px_left / sampleSize;
    meanPosition[1] = center_py_left / sampleSize;
    meanPosition[2] = center_px_right / sampleSize;
    meanPosition[3] = center_py_right / sampleSize;
    return meanPosition;
}

void removeSamples (std::vector<Sample> & s, std::vector<int> removableSamples)
{
    s.erase(
        std::remove_if(s.begin(), s.end(), [](Samples::interator it){
	    return std::find(removableSamples.begin(), removableSamples.end(), it - s.begin()) != removableSamples.end();})
    );
}

std::vector<int> saccadeFilter_velocity (std::vector<Sample> & s, ModelType mt, bool remove = true)
{
    // Get mean eye positions and points of changes of the marker position.
    std::vector<int> markerChanges;
    Vector4f meanPos = meanPosAndMarkerChanges(s, markerChanges);
    
    // Convert Cartesean into polar coordinates
    std::vector<Vector2f> polar_eye;
    if(mt == EYE_RIGHT)
    {
        for(Samples::iterator it = s.begin(); it != s.end(); ++it)
        {
            polar_eye.push_back(cart2pol(*it.px_right - meanPos[2], *it.py_right - meanPos[3]));
        }
    }else{
        for(Samples::iterator it = s.begin(); it != s.end(); ++it)
        {
            polar_eye.push_back(cart2pol(*it.px_left - meanPos[0], *it.py_left - meanPos[1]));
        }
    }
}