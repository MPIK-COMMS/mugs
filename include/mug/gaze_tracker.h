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
	    std::cout << "calculating eye transform..." << std::endl;
            findEyeTransform<EyeModelT>(trainingData, screen, T_trans, T_rot);
            std::cout << "done" << std::endl;
	    
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
        inline Eigen::Vector3f getScreenUV(const Eigen::Vector3f &H_pos, const Eigen::Vector3f &H_o, 
                const Sample &s) const
        {
            Eigen::Vector2f gazeAngles = eyeModel.predict(s);
	    Eigen::Vector2f scrProjection = screen.project(H_pos, H_o, T_trans, T_rot, gazeAngles[0], gazeAngles[1]);
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

