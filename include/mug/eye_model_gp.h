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
	 * \brief Constructor of the EyeModelGp class.
	 */
	EyeModelGp()
	{
	    this->mt = EYE_BOTH;
	}
	
	/**
	 * \brief Constructor of the EyeModelGp class.
	 * \param[in] mt ModelType that will be used for this instance of EyeModelLinear.
	 */
	EyeModelGp(ModelType mt)
	{
	    this->mt = mt;
	}
	
	/**
	 * \brief Setter function for the member variable mt
	 * \param[in] mt ModelType value for member variable mt
	 */
	void setModelType (ModelType mt) { this->mt = mt; }
	
	/**
	 * \brief Getter function for the member variable mt
	 */
	ModelType getModelType () { return this->mt; }

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
	ModelType mt;       /// specifies which eye should be used for regression

        inline int param_dim() const 
        { 
            if (gpU.get())
                return gpU->covf().get_param_dim();
            else
                return 0;
        }


        void sampleVecToMat(const std::vector<Sample> &samples, 
		Eigen::MatrixXd &X, Eigen::VectorXd &yu, Eigen::VectorXd &yv) const;
	
        /**
	 * \brief Convert \ref Sample object into a matrix that can used as input for GPR.
	 * \param[in] s Sample object.
	 * \param[out] x Vector used to store the input data.
	 * \param[out] yu Stores x coordinate of the target.
	 * \param[out] yv Stores y coordinate of the target.
	 */
        void convertToGpInput(const Sample &s, Eigen::VectorXd &x, double &yu, double &yv) const;

	
	Eigen::Vector2f predict(const Eigen::VectorXd &x) const;
	
	double getConfidence(const Eigen::VectorXd &x) const;
	
        /// Gaussian Process hyper-parameters
        Eigen::VectorXd paramsU;
        Eigen::VectorXd paramsV;
    };

}

#endif

