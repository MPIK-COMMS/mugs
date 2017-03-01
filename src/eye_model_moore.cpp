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

void EyeModelMoore::fit(const std::vector<Sample> &samples) {}

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

