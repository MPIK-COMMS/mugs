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

#ifndef EYE_MODEL_GP_H
#define EYE_MODEL_GP_H

#include <vector>
#include <memory>
#include <mug/eye_model.h>
#include "gp.h"


namespace mug
{

    class Sample;

    /** 
     * \brief Eye model using Gaussian Process Regression to map pupil positions 
     * to gaze angles. 
     * \author Bjoern Browatzki and Jonas Ditz
     */
    class EyeModelGp : public EyeModel
    {
    public:
        static const int INPUT_DIM_MONOCULAR;
        static const int INPUT_DIM_BINOCULAR;

	/** 
         * \brief Create mapping from pupil positions to gaze angles (yaw, pitch).
         * \param[in] pupilPositions Vector of 2D positions in the eye tracker camera image 
         * \param[in] gazeAngles Vector of (yaw, pitch) tuples
         */
        void fit(const std::vector<Eigen::Vector2f> &pupilPositions, 
                const std::vector<Eigen::Vector2f> &gazeAngles);
	
        /** 
         * \brief Create mapping from pupil positions to gaze angles (yaw, pitch).
         * \param[in] samples Vector of mug::Sample containing pupil image positions 
         * \param[in] cov String specifying kernel for building covariance matrix (default = "CovSum ( CovSEard, CovNoise)")
         * \param[in] optimizeParameters If true, GP hyper-parameters are optimized (default = true)
         * \param[in] numOptimizationSamples Number of samples used for hyper-parameter optimization (default = 200)
         */
        void fit(const std::vector<Sample> &samples, 
                const std::string cov = default_cov(), 
                bool optimizeParameters = true, 
                int numOptimizationSamples = 200);

        /** 
         * \brief Predict gaze angles based on pupil position
         * \note If you are only interested in predictions but not in their
         * expected variance, use \ref predict(const Sample &s) instead for 
         * increased performance.
         * \param[in] s Sample object containing UV pupil position 
         * \param[out] varX Estimated prediction variance in X [px]
         * \param[out] varY Estimated prediction variance in Y [px]
         * \return 2D vector containing yaw and pitch angle in radians.
         */
        Eigen::Vector2f predict(const Sample &s, double &varX, double &varY) const;

        /** 
         * \brief Predict gaze angles based on pupil position
         * \param[in] s Sample object containing UV pupil position 
         * \return 2D vector containing yaw and pitch angle in radians.
         */
        Eigen::Vector2f predict(const Sample &s) const;
	
	/**
	 * \brief Calculate confidence in the predicted gaze angles
	 * \param[in] s Sample object containing UV pupil position
	 * \return Confidence in calculated gaze angles
	 */
	double getConfidence(const Sample &s) const;

        void save(const std::string &filename);

        inline bool isTrained() const 
        { 
            return (gpU.get() && gpV.get());
        }

        inline int input_dim() const
        {
            if (gpU.get())
                return gpU->get_input_dim();
            else
                return 0;
        }

        static std::string default_cov() { return "CovSum ( CovSEard, CovNoise)"; }
        inline std::string cov() { return gpU->covf().to_string(); }


    private:
        std::auto_ptr<libgp::GaussianProcess> gpU;
        std::auto_ptr<libgp::GaussianProcess> gpV;

        inline int param_dim() const 
        { 
            if (gpU.get())
                return gpU->covf().get_param_dim();
            else
                return 0;
        }


        void sampleVecToMat(const std::vector<Sample> &samples, Eigen::MatrixXd &X, 
                Eigen::VectorXd &yu, Eigen::VectorXd &yv) const;
        void convertToGpInput(const Sample &s, Eigen::VectorXd &x, double &yu, double &yv) const;
        //static Eigen::VectorXd convertToGpInput(const Eigen::Vector3f H_pos, const Eigen::Vector3f H_o, 
                //float px, float py);
        //static Eigen::VectorXd convertToGpInput(const Eigen::Vector3f H_pos, const Eigen::Vector3f H_o, 
            //float px_l, float py_l, float px_r, float py_r);
	
	Eigen::Vector2f predict(const Eigen::VectorXd &x) const;
	
	double getConfidence(const Eigen::VectorXd &x) const;
	
        /// Gaussian Process hyper-parameters
        Eigen::VectorXd paramsU;
        Eigen::VectorXd paramsV;
    };

}

#endif

