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

#ifndef LINEAR_EYE_MODEL_H
#define LINEAR_EYE_MODEL_H

#include <vector>
#include <Eigen/Dense>

#include <mug/eye_model.h>
#include "mug/sample.h"

namespace mug
{
    /** 
     * \brief   Eye model using linear regression to create linear mapping from  
     *          pupil positions to gaze angles. 
     * \note    This model is inaccurate for large eye movements but 
     *          computationally very efficient. 
     * \author  Bjoern Browatzki and Jonas Ditz
     */
    class EyeModelLinear : public EyeModel
    {
    public:
        /** 
         * \brief Perform linear regression to fit pupil positions to gaze angles (yaw, pitch).
         * \param[in] pupilPositions Vector of 2D positions in the eye tracker camera image 
         * \param[in] gazeAngles Vector of (yaw, pitch) tuples
         */
        void fit(const std::vector<Eigen::Vector2f> &pupilPositions,  
                 const std::vector<Eigen::Vector2f> &gazeAngles);

        /** 
         * \brief Predict gaze angles based on pupil position
         * \param[in] s Sample object containing UV pupil position 
         * \return 2D vector containing yaw and pitch angle in radians.
         */
        Eigen::Vector2f predict(const Sample &s) const
        {
	    Eigen::VectorXf pupil = Eigen::VectorXf(2);
	    pupil[0] = s.px_left;
	    pupil[1] = s.py_left;
	    return predict(pupil);
	}
	
	/**
	 * \brief Calculate confidence in predicted gaze angles
	 * \param[in] s Sample object containing UV pupil position.
	 * \return Confidence in predicted ganze angles.
	 */
	inline double getConfidence(const Sample &s) const
	{
	    // Currently, no convidence evaluation implemented!
	    return 0.0;
	};

    private:
        Eigen::Vector2f modelYaw;       /// coefficients of linear model for yaw 
        Eigen::Vector2f modelPitch;     /// coefficients of linear model for pitch
       
        /** 
         * \brief Computes simple linear regression.
         * \param[in] X Row matrix of input data.
         * \param[in] Y Row matrix of target data.
         * \return Parameters of fitted line
         */
        Eigen::Vector2f simpleRegression(const Eigen::VectorXf &X, const Eigen::VectorXf &Y);
	
	/** 
         * \brief Predict gaze angles based on pupil position based on linear model.
         * \param[in] pupil UV pupil position. Pupil is expected to be 2D vector (monocular).
         * \return 3D vector containing yaw and pitch angle in radians.
         */
        inline Eigen::Vector2f predict(const Eigen::VectorXf &pupil) const 
        {
            return Eigen::Vector2f(
                    modelYaw[0]   * pupil[0] + modelYaw[1],
                    modelPitch[0] * pupil[1] + modelPitch[1]);
        };
    };
}

#endif

