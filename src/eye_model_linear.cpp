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

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <mug/eye_model_linear.h>

using namespace Eigen;
using namespace mug;

Vector2f EyeModelLinear::simpleRegression(const VectorXf &X, const VectorXf &Y)
{
    float mX = X.mean(); 
    float mY = Y.mean(); 

    float cov = 0;
    float var = 0;
    for (int i = 0; i < X.size(); i++)
    {
        float dx = X[i] - mX;
        cov += dx * (Y[i] - mY); 
        var += dx * dx; 
    }

    if (var < 0.0001)
       var = 0.0001; 

    float m = cov/var;
    float b = mY - m*mX;

    return Vector2f(m,b);
}

void EyeModelLinear::fit(const std::vector<Sample> &samples) {}


void EyeModelLinear::fit(const std::vector<Vector2f> &pupilPositions,  const std::vector<Vector2f> &gazeAngles)
{
    assert(pupilPositions.size() == gazeAngles.size());

    int n = pupilPositions.size();
    VectorXf px(n), py(n);
    VectorXf yaw(n), pitch(n);
    for (int i = 0; i < n; i++)
    {
        px[i]    = pupilPositions[i][0];
        py[i]    = pupilPositions[i][1];
        yaw[i]   = gazeAngles[i][0];
        pitch[i] = gazeAngles[i][1];
    }

    modelYaw   = simpleRegression(px, yaw);
    modelPitch = simpleRegression(py, pitch);
}
