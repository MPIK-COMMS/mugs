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
	 * \brief Constructor of the EyeModelMoore class.
	 */
	EyeModelMoore()
	{
	    this->mt = EYE_LEFT;
	}
	
	/**
	 * \brief Constructor of the EyeModelMoore class.
	 * \param[in] mt ModelType that will be used for this instance of EyeModelLinear.
	 */
	EyeModelMoore(ModelType mt)
	{
	    this->mt = mt;
	}
	
	/**
	 * \brief Setter function for the member variable mt
	 * \param[in] mt ModelType value for member variable mt
	 */
	void setModelType (ModelType mt)
	{
	    this->mt = mt;
	}
	
	/**
	 * \brief Getter function for the member variable mt
	 */
	ModelType getModelType () const { return this->mt; }

        /**
         * \brief Getter function for the approach that this eye model was implemented for.
         */
        Approach getApproach () const { return GEOMETRIC; }
        
        /** 
         * \brief Create mapping from pupil positions to gaze angles (yaw, pitch).
	 *        This is just a functionless auxiliary function.
         * \param[in] samples Vector of mug::Sample containing pupil image positions
         */
        void fit(const std::vector<Sample> &samples);
      
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
	    switch(this->mt)
	    {
		case EYE_RIGHT:
		{
		    Eigen::VectorXf pupil = Eigen::VectorXf(2);
	            pupil[0] = s.px_right;
	            pupil[1] = s.py_right;
	            return predict(pupil);
		}
		default:
		{
	            Eigen::VectorXf pupil = Eigen::VectorXf(2);
	            pupil[0] = s.px_left;
	            pupil[1] = s.py_left;
	            return predict(pupil);
		}
	    }
	}
	
	/**
	 * \brief Calculate confidence in predicted gaze angles. Confidence 
	 *        calculation for geometric model is not implemented, currently.
	 * \param[in] s Sample object containing UV pupil position.
	 * \return Confidence in predicted ganze angles.
	 */
	inline double getConfidence(const Sample &s) const
	{
	    return 0.0;
	};

    private:
        Eigen::MatrixXd A;  /// eye model coefficients
        ModelType mt;       /// specifies which eye should be used for regression
        
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

