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

#include <dlib/optimization.h>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <limits>

#include "mug/eye_model_gp_optimization.h"
#include "mug/eye_model_gp.h"
#include "mug/sample.h"

#include "gp.h"

using namespace dlib;
using namespace Eigen;
using namespace mug;

typedef matrix<double,0,1> column_vector;

class gpLogL
{
public:
    gpLogL(libgp::GaussianProcess *gp_)
        : gp(gp_), nIters(0)
    {}

    double operator()(const column_vector& p) const
    {
        nIters++;
        if (nIters >= 100)
            return 1;

        // set parameters of covariance function
        Eigen::VectorXd params(p.size());
        for (int i=0; i<params.size(); i++)
            params(i) = p(i);

        gp->covf().set_loghyper(params);

        //double log_likelihood = gp->log_likelihood()/gp->get_sampleset_size();
        double log_likelihood = gp->log_likelihood();

        //std::cout << "  " << nIters <<  "  ll:" << log_likelihood << " p:" << params.transpose() << std::endl;
        //std::cout << "  " << nIters <<  "  ll:" << log_likelihood << " p:" << params.transpose() << std::endl;

        return log_likelihood;

    }

public:
    mutable int nIters;
    libgp::GaussianProcess *gp;
};


class gpLogL_der
{
public:
    gpLogL_der(libgp::GaussianProcess *gp_)
        : gp(gp_), nIters(0)
    {}

    column_vector operator()(const column_vector& p) const
    {
        nIters++;

        Eigen::VectorXd log_likelihood_gradient = gp->log_likelihood_gradient();

        //std::cout << "max grad val: " << log_likelihood_gradient.maxCoeff() << std::endl;
        double gmax = log_likelihood_gradient.maxCoeff();
        double norm = sqrt(log_likelihood_gradient.array().square().sum());

        log_likelihood_gradient /= norm;
        column_vector der(log_likelihood_gradient.size());
        for (int i=0; i<der.size(); i++)
            der(i) = log_likelihood_gradient(i);

        //std::cout << nIters << " g-norm:" << norm << "  grad: "<< log_likelihood_gradient.transpose() << std::endl;
        return der;
    }
private:
    mutable int nIters;
    libgp::GaussianProcess *gp;
};


void nl_optimization_thread(
        const std::string &cov, 
        const MatrixXd &X, 
        const VectorXd &y, 
        column_vector &starting_point,  
        const column_vector lower_bounds, 
        const column_vector upper_bounds,
        int nTrain,
        int nValid)
{
    int i = 0;

    libgp::GaussianProcess gp(X.cols(), cov);

    // first nTrain ids are used as training set, remaining ids as validation
    std::vector<int> ids; 
    for (int i=0; i<X.rows();i++)
    {
        ids.push_back(i);
    }
    std::random_shuffle(ids.begin(), ids.end());
    assert(nValid <= ids.size() - nTrain);

    // add data
    for (int i=0; i<nTrain; i++)
    {
        int id  = ids[i];
        gp.add_pattern(VectorXd(X.row(id)).data(), y[id]);
    }

    // find hyper-parameters using log-likelihood maximisation
    gpLogL ll(&gp); 
    find_max(cg_search_strategy(), 
                objective_delta_stop_strategy(1.0), 
                //gradient_norm_stop_strategy(0.5),
                ll,
                gpLogL_der(&gp), 
                starting_point, -1);
}

void GpOptimization::run(const std::string &cov, const Eigen::MatrixXd &X, const Eigen::VectorXd &yu, const Eigen::VectorXd &yv, 
        Eigen::VectorXd &paramsU, Eigen::VectorXd &paramsV, int numTrainingSamples)
{
    assert(paramsU.size() > 0);
    assert(paramsU.size() == paramsV.size());
    int param_dim = paramsU.size();

    std::cout << std::fixed << std::setprecision(2);

    // Split into test and validation set
    int nTrain = X.rows() * 0.25;
    if (numTrainingSamples > 0)
        nTrain = std::min(numTrainingSamples, (int)X.rows()-1);
    const int nValid  = std::min((int)X.rows()-nTrain, nTrain*2);     

    //
    // Find hyper-parameters for GP
    // 

    //std::cout << "Parameter optimization..." << std::endl; 

    // Specify search parameters for NL-optimization
    column_vector starting_point_u(param_dim);
    column_vector starting_point_v(param_dim);
    column_vector lower_bounds_u(param_dim);
    column_vector lower_bounds_v(param_dim);
    column_vector upper_bounds_u(param_dim);
    column_vector upper_bounds_v(param_dim);
    for (int i=0; i<param_dim; i++)
    {
        starting_point_u(i) = .0;
        starting_point_v(i) = .0;
        lower_bounds_u(i)   = -2.0;
        lower_bounds_v(i)   = -2.0;
        upper_bounds_u(i)   = 13.0;
        upper_bounds_v(i)   = 13.0;
    }

    // Run parameter optimizations for U and V coords in parallel...
    boost::thread threads[2];
    threads[0] = boost::thread(nl_optimization_thread, cov, X, yu, 
            boost::ref(starting_point_u), lower_bounds_u, upper_bounds_u, nTrain, nValid);
    threads[1] = boost::thread(nl_optimization_thread, cov, X, yv,
            boost::ref(starting_point_v), lower_bounds_v, upper_bounds_v, nTrain, nValid);

    // Wait for all threads to finish
    for (size_t i = 0; i < 2; i++)
    {
        threads[i].join();
    }

    // Return optimal parameters
    for (int i=0; i<param_dim; i++)
    {
        paramsU(i) = starting_point_u(i);
        paramsV(i) = starting_point_v(i);
    }

    //std::cout << "Optimization completed." << std::endl;
    //std::cout << "Size training set  :" << " " << nTrain << std::endl;
    //std::cout << "Size validation set:" << " " << nValid << std::endl;
    //std::cout << "ParamsU:" << paramsU.transpose() << std::endl;
    //std::cout << "ParamsV:" << paramsV.transpose() << std::endl;

}

