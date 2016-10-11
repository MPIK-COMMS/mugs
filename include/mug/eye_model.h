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

#ifndef EYE_MODEL_H
#define EYE_MODEL_H

#include <vector>

namespace mug
{
  
    class Sample;
  
    /**
     * \brief Enumeration type to specify, which eye should be used in the 
     *        regression.
     */
    enum ModelType
    {
        EYE_LEFT  = 1,
        EYE_RIGHT = 2,
        EYE_BOTH  = 3,
        PUPIL     = 4,
        EYE_OFFSET = 5,
        HEAD_ONLY  = 6
    };

    /**
     * \brief Enumeration type to distinguish eye models that are used for the geometric
     *        approach and eye models implemented for the regression approach.
     */
    enum Approach
    {
        GEOMETRIC  = 1,
        REGRESSION = 2
    };

    /** 
     * \brief Base class for eye model implementations.
     * \author Bjoern Browatzki and Jonas Ditz
     */
    class EyeModel
    {
    public:
        /** 
         * \brief Create mapping from pupil positions to gaze angles (yaw, pitch).
         * \param[in] pupilPositions Vector of 2D positions in the eye tracker camera image 
         * \param[in] gazeAngles Vector of (yaw, pitch) tuples
         */
        virtual void fit(const std::vector<Eigen::Vector2f> &pupilPositions, 
                const std::vector<Eigen::Vector2f> &gazeAngles) = 0;

        /** 
         * \brief Predict gaze angles based on pupil position
         * \param[in] s Sample object containing UV pupil position.
         * \return 2D vector containing yaw and pitch angle in radians.
         */
        virtual Eigen::Vector2f predict(const Sample &s) const = 0;
	
	
	/**
	 * \brief Calculate confidence in predicted gaze angles
	 * \param[in] s Sample object containing UV pupil position.
	 * \return Confidence in predicted ganze angles.
	 */
	virtual double getConfidence(const Sample &s) const = 0;

    };
}

#endif

