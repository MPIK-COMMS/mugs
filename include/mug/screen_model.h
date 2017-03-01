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

