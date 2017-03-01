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

#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <fstream>

#include "mug/eye_model_gp.h"
#include "mug/eye_model_gp_optimization.h"
#include "mug/sample.h"
#include "gp.h"

using namespace Eigen;
using namespace mug;

const int EyeModelGp::INPUT_DIM_MONOCULAR = 8; ///< 2 eye coordinates, 6 head coordinates
const int EyeModelGp::INPUT_DIM_BINOCULAR = 10; ///< 4 eye coordinates, 6 head coordinates

void fit_gp_thread(libgp::GaussianProcess &gp, const MatrixXd &X, const VectorXd y)
{
    for (int i=0; i<X.rows(); i++)
    {
        gp.add_pattern(VectorXd(X.row(i)).data(), y[i]);
        if (i > 3000 && i % 500 == 0) 
        {
            std::cout << "fitted samples: " << i << std::endl;
        }
    }
}

void EyeModelGp::fit(const std::vector< Vector2f >& pupilPositions, const std::vector< Vector2f >& gazeAngles)
{
    if (pupilPositions.empty())
    {
        std::cerr << "No samples supplied for gaze model training" << std::endl;
	return;
    }
    
    if (pupilPositions.size() != gazeAngles.size())
    {
        std::cerr << "Different numbers of pupil positions and gaze angles provided. Numbers has to be equal!" << std::endl;
	return;
    }
  
    std::vector<Sample> samples;
    for (unsigned int i = 0; i<pupilPositions.size(); ++i)
    {
        Sample s;
	s.px_left = pupilPositions[i][0];
	s.py_left = pupilPositions[i][1];
	s.yaw = gazeAngles[i][0];
	s.pitch = gazeAngles[i][1];
	samples.push_back(s);
    }
    
    fit(samples);
}

void EyeModelGp::fit(const std::vector<Sample> &samples, 
        const std::string cov, bool optimizeParameters, 
        int numOptimizationSamples)
{
    if (samples.empty())
    {
        std::cerr << "No samples supplied for gaze model training." << std::endl;
        return;
    }

    // convert sample vectors to Eigen matrices
    Eigen::MatrixXd X, Xs;
    Eigen::VectorXd yu, yus;
    Eigen::VectorXd yv, yvs;
    sampleVecToMat(samples, X, yu, yv);

    int input_dim = X.cols();
    gpU = std::auto_ptr<libgp::GaussianProcess>(new libgp::GaussianProcess(input_dim, cov));
    gpV = std::auto_ptr<libgp::GaussianProcess>(new libgp::GaussianProcess(input_dim, cov));

    if (optimizeParameters)
    {
        GpOptimization opt;
        paramsU = Eigen::VectorXd(param_dim());
        paramsV = Eigen::VectorXd(param_dim());
        opt.run(cov, X, yu, yv, paramsU, paramsV, numOptimizationSamples);
        gpU->covf().set_loghyper(paramsU);
        gpV->covf().set_loghyper(paramsV); 
    }
    else
    {
        // use existing params 
    }

    assert(X.rows() == yu.size());
    assert(X.rows() == yv.size());

    //std::cout << "Fitting GP (" << X.rows() << " samples)..." << std::endl; 
    boost::thread threads[2];
    threads[0] = boost::thread(fit_gp_thread, boost::ref(*gpU), X, yu);
    threads[1] = boost::thread(fit_gp_thread, boost::ref(*gpV), X, yv);
    // Wait for threads to finish
    for (size_t i = 0; i < 2; i++)
    {
        threads[i].join();
    } 
}



Eigen::Vector2f EyeModelGp::predict(const Sample &s, double &varX, double &varY) const
{
    VectorXd x;
    double yu, yv;
    convertToGpInput(s, x, yu, yv); 

    varX = const_cast<libgp::GaussianProcess*>(gpU.get())->var(x.data());
    varY = const_cast<libgp::GaussianProcess*>(gpV.get())->var(x.data());

    return predict(x); 
}
Eigen::Vector2f EyeModelGp::predict(const Sample &s) const
{
    VectorXd x;
    double yu, yv;
    convertToGpInput(s, x, yu, yv); 
    return predict(x); 
}

Eigen::Vector2f EyeModelGp::predict(const Eigen::VectorXd &x) const
{
    if (! isTrained())
    {
        std::cerr << "Train model first!" << std::endl;
        return Eigen::Vector2f();
    }

    if (x.size() != input_dim())
    {
        std::cerr << "Invalid size of x! Is " << x.size() << ", expected " << input_dim() << std::endl;
        return Eigen::Vector2f();
    }

    Eigen::Vector2f f;
    // casting necessary because gp.f() is not declared as const
    f[0] = const_cast<libgp::GaussianProcess*>(gpU.get())->f(x.data()); 
    f[1] = const_cast<libgp::GaussianProcess*>(gpV.get())->f(x.data());
    //f[0] = const_cast<libgp::GaussianProcess*>(gpU.get())->f(&x[0]); 
    //f[1] = const_cast<libgp::GaussianProcess*>(gpV.get())->f(&x[1]);
    return f;
}

double EyeModelGp::getConfidence(const Sample &s) const
{
    VectorXd x;
    double yu, yv;
    convertToGpInput(s, x, yu, yv);
    return getConfidence(x);
}

double EyeModelGp::getConfidence(const VectorXd &x) const
{
    if (! isTrained())
    {
        std::cerr << "Train model first!" << std::endl;
        return -1;
    }

    if (x.size() != input_dim())
    {
        std::cerr << "Invalid size of x! Is " << x.size() << ", expected " << input_dim() << std::endl;
        return -1;
    }
  
    double varU = const_cast<libgp::GaussianProcess*>(gpU.get())->var(x.data());
    double varV = const_cast<libgp::GaussianProcess*>(gpV.get())->var(x.data());
    return (sqrt(varU) + sqrt(varV))/2;
}


void EyeModelGp::save(const std::string &filename)
{
    gpU->write((filename+".u").c_str());
    gpV->write((filename+".v").c_str());
}

void EyeModelGp::convertToGpInput(const Sample &s, Eigen::VectorXd &x, double &yu, double &yv) const
{
    switch (this->mt)
    {
        case HEAD_ONLY: 
	{
            x = Eigen::VectorXd(6);
            x << s.H_pos[0],s.H_pos[1],s.H_pos[2],
               s.H_o[0],s.H_o[1],s.H_o[2];
            yu = s.target_pos[0];
            yv = s.target_pos[1];
            break;
	}
        case EYE_OFFSET: 
	{
            x = Eigen::VectorXd(8);
            x << s.H_pos[0],s.H_pos[1],s.H_pos[2],
               s.H_o[0],s.H_o[1],s.H_o[2],
               s.du/100, s.dv/100;
            yu = s.target_pos[0];
            yv = s.target_pos[1];
            break;
	}
        case PUPIL: 
	{
            x = Eigen::VectorXd(2);
            x << 
               (double)s.px_left, (double)s.py_left,
               //(double)s.px_right, (double)s.py_right;
            yu = s.yaw;
            yv = s.pitch;
            break;
	}
        case EYE_LEFT:
	{
            x = Eigen::VectorXd(INPUT_DIM_MONOCULAR);
            x << s.H_pos[0], s.H_pos[1], s.H_pos[2],
	       s.H_o[0], s.H_o[1], s.H_o[2], 
               (double)s.px_left, (double)s.py_left;
            yu = s.target_pos[0];
            yv = s.target_pos[1];
            break;
	}
        case EYE_RIGHT:
	{
            x = Eigen::VectorXd(INPUT_DIM_MONOCULAR);
            x << s.H_pos[0], s.H_pos[1], s.H_pos[2],
	       s.H_o[0], s.H_o[1], s.H_o[2], 
               (double)s.px_right, (double)s.py_right;
            yu = s.target_pos[0];
            yv = s.target_pos[1];
            break;
	}
        case EYE_BOTH:
	{
            x = Eigen::VectorXd(INPUT_DIM_BINOCULAR);
            x << s.H_pos[0], s.H_pos[1], s.H_pos[2],
	       s.H_o[0], s.H_o[1], s.H_o[2],
               (double)s.px_left,  (double)s.py_left,
               (double)s.px_right, (double)s.py_right;
            yu = s.target_pos[0];
            yv = s.target_pos[1];
            break;
	}
    }
}

void EyeModelGp::sampleVecToMat(const std::vector<Sample> &samples,
        Eigen::MatrixXd &X, Eigen::VectorXd &yu, Eigen::VectorXd &yv) const
{
    if (samples.empty())
    {
        std::cout << "Info: No samples to convert." << std::endl;
        return;
    }

    // find out input dimension     
    Eigen::VectorXd x; double tu, tv;
    convertToGpInput(samples[0], x, tu, tv);
    int input_dim = x.size();

    X = Eigen::MatrixXd(samples.size(), input_dim);
    yu = Eigen::VectorXd(samples.size());
    yv = Eigen::VectorXd(samples.size());
    for (int i=0; i<samples.size(); i++)
    {
        Eigen::VectorXd x; double tu, tv;
        convertToGpInput(samples[i], x, tu, tv);
        yu[i] = tu;
        yv[i] = tv;
        X.row(i) = x;
    }
}
