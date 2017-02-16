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

#include "persistence1d.hpp"

using namespace Eigen;
using namespace mug;

void mug::correctPolArtifacts(std::vector<float> & theta)
{
    // threshold to decide whether the current jump is a polar artifact
    float threshold = 2 * M_PI - 0.15;
    for (std::vector<float>::iterator it = theta.begin(), prev = theta.end(); it != theta.end(); prev=it, ++it)
    {
        if (it == theta.begin()){continue;}
        float velo = std::abs(*(it) - *(prev));
	if (velo >= threshold) {
	    if (*(it) > 0){
	        *(it) -= 2 * M_PI;
	    } else {
	        *(it) += 2 * M_PI;
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

std::vector<Eigen::Vector2i> mug::onsetFilter_velocity (std::vector<Sample> & s, ModelType mt, int samplerate, bool remove)
{
    // Get mean eye positions and points of changes of the marker position.
    std::vector<int> markerChanges;
    Vector4f meanPos = meanPosAndMarkerChanges(s, markerChanges);
    
    // remove all marker changes, which are corresponding to smooth persuits
    removeSmoothPersuitMarker(markerChanges);
    
    // Convert centered Cartesean into polar coordinates
    std::vector<float> theta, r;
    if(mt == EYE_RIGHT)
    {
        for(Samples::iterator it = s.begin(); it != s.end(); ++it)
        {
	    Vector2f polar = cart2pol(it->px_right - meanPos[2], it->py_right - meanPos[3]);
            theta.push_back(polar[0]);
	    r.push_back(polar[1]);
        }
    }else{
        for(Samples::iterator it = s.begin(); it != s.end(); ++it)
        {
            Vector2f polar = cart2pol(it->px_left - meanPos[2], it->py_left - meanPos[3]);
            theta.push_back(polar[0]);
	    r.push_back(polar[1]);
        }
    }
    
    // remove artifacts caused by conversion into polar coordinates
    correctPolArtifacts(theta);
    
    // calculate first derivative of theta
    std::vector<float> derivative;
    simpleDerivative(theta, derivative);
    
    // find local extrema in derivative of theta
    std::vector<int> extrema_theta;
    p1d::Persistence1D p;
    p.RunPersistence(derivative);
    p.GetExtremaIndices(extrema_theta, extrema_theta, 0);
    
    // find local extrema in r
    std::vector<int> extrema_r;
    p.RunPersistence(r);
    p.GetExtremaIndices(extrema_r, extrema_r, 0);
    
    // sort the extrema indices
    std::sort(extrema_theta.begin(), extrema_theta.end());
    std::sort(extrema_r.begin(), extrema_r.end());
    
    // compute the areas that should be removed
    std::vector<Eigen::Vector2i> removableAreas;
    for(std::vector<int>::iterator itMarker = markerChanges.begin(); itMarker != markerChanges.end(); ++itMarker)
    {
        std::vector<int>::iterator theta_peak = std::upper_bound(extrema_theta.begin(), extrema_theta.end(), *itMarker);
	std::vector<int>::iterator r_peak = std::upper_bound(extrema_r.begin(), extrema_r.end(), *itMarker);
	
	// check whether r_peak and theta_peak are not to far away
	Eigen::Vector2i area;
	if (*theta_peak - *r_peak > samplerate/10)
	{
	    area[0] = *itMarker;
	    area[1] = *theta_peak + (int)std::ceil(samplerate/10);
	    removableAreas.push_back(area);
	}
	else
	{
	    area[0] = *itMarker;
	    area[1] = *r_peak + (int)std::ceil(samplerate/10);
	    removableAreas.push_back(area);
	}
    }
    
    // remove all data points that occure between target and fixation onset
    if (remove)
    {
      removeSamples(s, removableAreas);
    }
    
    return removableAreas;
}
