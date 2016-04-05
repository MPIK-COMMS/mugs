/* 
 * Software License Agreement (BSD License) 
 *
 * MUG - Mobile and Unrestrained Gazetracking
 * Copyright (c) 2013, Max Planck Institute for Biological Cybernetics
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
