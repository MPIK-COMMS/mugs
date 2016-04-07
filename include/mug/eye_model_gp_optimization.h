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

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_

#include <vector>


namespace mug
{
    class Sample;

    /** 
     * \brief Finds hyper-paramerter for Gaussian Processes through log-likelihood maximization.
     * Employs conjugate gradient algorithm for non-linear optimization.
     * \author Bjoern Browatzki 
     */
    class GpOptimization
    {
    public:
        void run(const std::string &cov,const Eigen::MatrixXd &X, const Eigen::VectorXd &yu, const Eigen::VectorXd &yv, 
            Eigen::VectorXd &paramsU, Eigen::VectorXd &paramsV, int numOptimizationSamples);
    private:

    };
}

#endif
