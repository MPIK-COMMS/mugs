/* 
 * Copyright (c) 2013, 2017 Max Planck Institute for Biological Cybernetics
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
 * 
 * Author: Bjoern Browatzki
 */

#ifndef UTIL_H
#define UTIL_H

#include <mug/sample.h>

using namespace mug; 

double __dist(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
{
    return sqrt((p1-p2).array().square().sum());
}

std::vector<Sample> filterSamples(const std::vector<Sample> &samples)
{
    std::vector<Sample> filtered;

    Eigen::Vector2f lastTarget;
    int lastOnset = 0;

    for (int i=0; i<samples.size(); i++)
    {
        const Sample &s = samples[i];
        int t = s.timestamp;
        
        if (__dist(s.target_pos, lastTarget) > 10)
        {
            lastOnset = t;
            lastTarget = s.target_pos;
        }

        if (t - lastOnset > 1250)
        {
            filtered.push_back(s);
        }
    }
    return filtered;
}


std::vector<Sample> takeFirstNSeconds(int nsecs, const std::vector<Sample> &samples)
{
    int timeStart = samples[0].timestamp;
    std::vector<Sample> subset;
    for (int i=0; i<samples.size(); i++)
    {
        if (samples[i].timestamp - timeStart < nsecs*1000)
        {
            subset.push_back(samples[i]);
        }
    }
    return subset;
}

std::vector<Sample> takeAfterNSeconds(int nsecs, const std::vector<Sample> &samples)
{
    int timeStart = samples[0].timestamp;
    std::vector<Sample> subset;
    for (int i=0; i<samples.size(); i++)
    {
        if (samples[i].timestamp - timeStart > nsecs*1000)
        {
            subset.push_back(samples[i]);
        }
    }
    return subset;
}

std::vector<Sample> takeRandomSubset(int n, const std::vector<Sample> &samples)
{
    if (n > samples.size())
    {
        n = samples.size();
    }
    std::vector<int> ids;
    for (int i=0; i<samples.size(); i++)
    {
        ids.push_back(i);
    }
    std::random_shuffle(ids.begin(), ids.end());

    std::vector<Sample> subset;
    for (int i=0; i<n; i++)
    {
        subset.push_back(samples[ids[i]]);
    }
    return subset;
}

std::vector<Sample> reduceSampleRate(int n, const std::vector<Sample> &samples)
{
    std::vector<Sample> res;
    for (int i=0; i<samples.size(); i++)
    {
        if (i % n == 0)
            res.push_back(samples[i]);
    }
    return res;
}

#endif
