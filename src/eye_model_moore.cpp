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
#include <mug/eye_model_moore.h>
#include <dlib/optimization.h>

using namespace Eigen;
using namespace mug;

typedef dlib::matrix<double,0,1> column_vector;

class OptMooreCoeffs
{
public:
    OptMooreCoeffs(const MatrixXd &X_, const MatrixXd &Y_)
        : X(X_), Y(Y_)
    {}

    double operator()(const column_vector& p) const
    {
        MatrixXd A(3,3);
        A(0,0) = p(0);
        A(0,1) = p(1);
        A(0,2) = p(2);
        A(1,0) = p(3);
        A(1,1) = p(4);
        A(1,2) = p(5);
        A(2,0) = 0.0;
        A(2,1) = 0.0;
        A(2,2) = 1.0;

        double mse = 0;
        for (int i=0; i<X.rows(); i++)
        {
            Vector3d Ys = A * X.row(i).transpose();
            double d1 = Y(i,0) - Ys(0);
            double d2 = Y(i,1) - Ys(1);
            mse += sqrt(d1*d1 + d2*d2);
        }
        //std::cout << "  mse: " << mse << "  " <<   dlib::trans(p);
        return mse;
    }
public:
    MatrixXd X;
    MatrixXd Y;
};

void EyeModelMoore::fit(const std::vector<Vector2f> &pupilPositions,  const std::vector<Vector2f> &gazeAngles)
{
    assert(pupilPositions.size() == gazeAngles.size());

    const int n = pupilPositions.size();

    MatrixXd Y(n, 3);
    MatrixXd X(n, 3);
    for (int i = 0; i < n; i++)
    {
        X(i, 0) = pupilPositions[i][0];
        X(i, 1) = pupilPositions[i][1];
        X(i, 2) = 1;

        double yaw   = gazeAngles[i][0];
        double pitch = gazeAngles[i][1];
        Y(i, 0) = sin(yaw) * cos(pitch);
        Y(i, 1) = -sin(pitch);
        Y(i, 2) = 1;
    }

    const int param_dim = 6;
    column_vector coeffs = dlib::uniform_matrix<double>(param_dim, 1, 0);

    double f = dlib::find_min_bobyqa(OptMooreCoeffs(X,Y), 
            coeffs, 
            coeffs.size()*2+1,                                 // number of interpolation points
            dlib::uniform_matrix<double>(param_dim, 1, -M_PI), // lower bounds
            dlib::uniform_matrix<double>(param_dim, 1,  M_PI), // upper bounds
            0.10,
            0.0002,
            5000
            );
    //std::cout << " f moore: " << f << "  \n " << starting_point << std::endl;

    A = MatrixXd(3,3);
    A(0,0) = coeffs(0);
    A(0,1) = coeffs(1);
    A(0,2) = coeffs(2);
    A(1,0) = coeffs(3);
    A(1,1) = coeffs(4);
    A(1,2) = coeffs(5);
    A(2,0) = 0.0;
    A(2,1) = 0.0;
    A(2,2) = 1.0;
}

