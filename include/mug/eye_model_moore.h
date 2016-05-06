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

#ifndef EYE_MODEL_MOORE_H
#define EYE_MODEL_MOORE_H

#include <vector>
#include <Eigen/Dense>

#include <mug/eye_model.h>
#include "mug/sample.h"

namespace mug
{
    /** 
     * \brief Eye model implementation based on Moore et al., 1996, 'A geometric basis for measurement 
     * of three-dimensional eye position using image processing'.
     * \author Bjoern Browatzki and Jonas Ditz
     */
    class EyeModelMoore : public EyeModel
    {
    public:
        /** 
         * \brief Create mapping from pupil positions to gaze angles (yaw, pitch).
         * \param[in] pupilPositions Vector of 2D positions in the eye tracker camera image 
         * \param[in] gazeAngles Vector of (yaw, pitch) tuples
         */
        void fit(const std::vector<Eigen::Vector2f> &pupilPositions,  
                 const std::vector<Eigen::Vector2f> &gazeAngles);

        /** 
         * \brief Predict gaze angles based on pupil position
         * \param[in] s Sample object containing UV pupil position.
         * \return 2D vector containing yaw and pitch angle in radians.
         */
        virtual Eigen::Vector2f predict(const Sample &s) const
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
	    // Currently, no confidence evaluation implemented!
	    return 0.0;
	};

    private:
        Eigen::MatrixXd A;  /// eye model coefficients
        
        /** 
         * \brief Predict gaze angles based on pupil position
         * \param[in] pupil UV pupil position 
         * \param[out] 2D vector containing yaw and pitch angle in radians.
         */
        inline Eigen::Vector2f predict(const Eigen::VectorXf &pupil) const 
        {
            assert(pupil.size() >= 2);

            double px = pupil[0];
            double py = pupil[1];

            double pitch = -A(1,0) * px - A(1,1) * py - A(1,2);
            pitch = asin(std::max(std::min(1.0,pitch), -1.0)); 
            double a = (A(0,0) * px + A(0,1) * py + A(0,2))/ cos(pitch);
            double yaw = asin(std::max(std::min(1.0,a), -1.0)); 

            return Eigen::Vector2f(yaw, pitch);
        };
    };
}

#endif

