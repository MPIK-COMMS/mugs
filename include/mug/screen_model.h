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

#ifndef SCREEN_MODEL_H
#define SCREEN_MODEL_H

namespace mug
{
    /** 
     * \brief Base class for screen model implementations.
     */
    class ScreenModel
    {
    public:
        /** 
         * \brief Calculates yaw and pitch angles from observer's eye to 
         * screen surface coordinate.
         * \param[in] targetUV Screen coordinate of target point [px]
         * \param[in] H_pos Head position in Cartesian coordinates [m]
         * \param[in] H_o Head orientation in Euler angles [rad]
         * \param[in] T_trans Head-eye offset vector [m]
         * \param[in] T_rot Eye orientation in head frame (Euler angles) [rad]
         * \param[out] yaw Eye azimuth [rad]
         * \param[out] pitch Eye inclination [rad]
         */
        virtual void calcGazeAngles(const Eigen::Vector2f &targetUV, 
                const Eigen::Vector3f &H_pos, const Eigen::Vector3f &H_o, 
                const Eigen::Vector3f &T_trans, const Eigen::Vector3f &T_rot,
                float &yaw, float &pitch) const = 0;
        
        /** 
         * \brief Returns intersection point of gaze vector and screen 
         * surface as UV screen coordinate.
         * \param[in] H_pos Head position in Cartesian coordinates [m]
         * \param[in] H_o Head orientation in Euler angles [rad]
         * \param[in] T_trans Head-eye offset vector [m]
         * \param[in] T_rot Eye orientation in head frame (Euler angles) [rad]
         * \param[in] yaw Eye azimuth [rad]
         * \param[in] pitch Eye inclination [rad]
         * \return  2D screen coordinate [px]
         */
        virtual Eigen::Vector2f project(const Eigen::Vector3f &H_pos, const Eigen::Vector3f &H_o, 
                const Eigen::Vector3f &T_trans, const Eigen::Vector3f &T_rot, 
                float yaw, float pitch) const = 0;
    };
}

#endif

