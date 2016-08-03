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

#ifndef GAZE_TRACKER_H
#define GAZE_TRACKER_H

#include <string>
#include <Eigen/Dense>

#include <mug/sample.h>
#include <mug/screen_model.h>
#include <mug/eye_model.h>
#include <mug/head_eye_transform.h>

namespace mug
{
    /** \brief Class for mobile gaze tracking
     *  \note typename: EyeModelT: type of eye model used for calculating gaze angles 
     *  \author Bjoern Browatzki (bjoern.browatzki@tuebingen.mpg.de) and Jonas Ditz (jonas.ditz@tuebingen.mpg.de)
     */
    template <typename EyeModelT>
    class GazeTracker
    {
    public:
        GazeTracker(const ScreenModel &scr) 
            : screen(scr)
        {} 

        /** 
         * \brief Calibrate eye model and head eye transform using training data in dataFile.
         * \param[in] dataFile File containing training data
         */
        inline void calibrate(const std::string &dataFile)
        {
            Samples trainingData = loadSamples(dataFile);
            calibrate(trainingData);
        }

        /** 
         * \brief Calibrate eye model and head eye transform using trainingData.
         * \param[in] trainingData Vector of training samples
         */
        inline void calibrate(const Samples &trainingData)
        {
            findEyeTransform<EyeModelT>(trainingData, screen, T_trans, T_rot);
	    
            // calculate true gaze angles based on current head eye transform
            std::vector<Vector2f> pupils;
            std::vector<Vector2f> angles;
            for (std::vector<Sample>::const_iterator s = trainingData.begin(); s != trainingData.end(); s++)
            {
                float yaw, pitch;
                screen.calcGazeAngles(s->target_pos, s->H_pos, s->H_o, T_trans, T_rot, yaw, pitch);
                pupils.push_back(Vector2f(s->px_left, s->py_left));
                angles.push_back(Vector2f(yaw, pitch));
            }

            eyeModel.fit(pupils, angles);
        }

        /** 
         * \brief Calculates gaze vector and projects it onto display surface.
         * \return 2D screen coordinate and confidence.
         */
        inline Eigen::Vector3f getScreenUV(const Sample &s) const
        {
            Eigen::Vector2f gazeAngles = eyeModel.predict(s);
	    Eigen::Vector2f scrProjection = screen.project(s.H_pos, s.H_o, T_trans, T_rot, gazeAngles[0], gazeAngles[1]);
	    double conf = eyeModel.getConfidence(s);
	    return Eigen::Vector3f(
	        scrProjection[0],
		scrProjection[1],
		conf);
        }
        
        /** 
         * \brief Calculates gaze vector and projects it onto display surface.
         * \return 2D screen coordinate and confidence.
         */
        inline Eigen::Vector3f getScreenUV(const Sample &s, ModelType mt) const
        {
            Eigen::Vector2f gazeAngles = eyeModel.predict(s, mt);
	    Eigen::Vector2f scrProjection = screen.project(s.H_pos, s.H_o, T_trans, T_rot, gazeAngles[0], gazeAngles[1]);
	    double conf = eyeModel.getConfidence(s);
	    return Eigen::Vector3f(
	        scrProjection[0],
		scrProjection[1],
		conf);
        }

    private:
        EyeModelT eyeModel;         ///< Used to map pupil positions to gaze angles
        const ScreenModel &screen;  ///< Represents display surface
        Eigen::Vector3f T_trans;    ///< Translation from head origin to eye origin
        Eigen::Vector3f T_rot;      ///< Rotation of eye in head reference frame
    };
}

#endif

