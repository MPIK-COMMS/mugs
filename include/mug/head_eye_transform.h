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

#ifndef HEAD_EYE_TRANSFORM_H
#define HEAD_EYE_TRANSFORM_H

#include <mug/sample.h>
#include <mug/screen_model.h>
#include <mug/eye_model.h>
#include <mug/eye_model_moore.h>

#include <Eigen/Dense>
#include <dlib/optimization.h>


using namespace dlib;
using namespace Eigen;


namespace mug
{
    typedef matrix<double,0,1> column_vector;

    template<typename EyeModelType>
    class OptEyeOffset
    {
    public:
        OptEyeOffset(const std::vector<Sample> &samples_, const ScreenModel &screen)
            : samples(samples_.begin(), samples_.end()), scr(screen)
        {}

        double operator()(const column_vector& p) const
        {
            Vector3f T_trans = Vector3f(p(0), p(1), p(2));
            Vector3f T_rot   = Vector3f(p(3), p(4), p(5));

            std::vector<Vector2f> pupils;
            std::vector<Vector2f> angles;
            for (Samples::const_iterator s = samples.begin(); s != samples.end(); s++)
            {
                float yaw, pitch;
                scr.calcGazeAngles(s->target_pos, s->H_pos, s->H_o, T_trans, T_rot, yaw, pitch);
                pupils.push_back(Vector2f(s->px_left, s->py_left));
                angles.push_back(Vector2f(yaw, pitch));
            }

            EyeModelType eyeModel;
            eyeModel.fit(pupils, angles);
    
            return getMse(eyeModel, T_trans, T_rot, samples, scr);
        }

    private:
        inline double getMse(const EyeModel &m, const Vector3f & T_trans, const Vector3f &T_rot, 
                const Samples &samples, const ScreenModel &screen) const
        {
            if (samples.empty())
            {
                std::cerr << "getMse: no samples" << std::endl;
                exit(0);
            }
            double mse = 0;
            for (std::vector<Sample>::const_iterator it = samples.begin(); it != samples.end(); it++)
            {
                const Sample &s = *it;
            
                //Vector2f pupil(s.px_left, s.py_left);
                Vector2f gazeAngles = m.predict(s);

                Vector2f uv = screen.project(s.H_pos, s.H_o, T_trans, T_rot, gazeAngles[0], gazeAngles[1]);
                double eu = uv[0] - s.target_pos[0];
                double ev = uv[1] - s.target_pos[1];
                mse += (eu*eu + ev*ev);
            }
            return mse/samples.size();
        }

        Samples samples;
        const ScreenModel &scr;
    };


    template<typename EyeModelT> 
    inline double findEyeTransform(
        const Samples &samples, const ScreenModel &screen,
        Vector3f &T_trans, Vector3f &T_rot)
    {
        int param_dim = 6;
        column_vector starting_point(param_dim);
        column_vector lower_bounds(param_dim);
        column_vector upper_bounds(param_dim);

        lower_bounds   = -.15, -.00, -0.20 , -.90, -.90, -.90  ;
        upper_bounds   =  .0,  .15,  0.0 ,  .90,  .90,  .90  ;
        starting_point =  -0.15, 0.03, -0.10, 0,0,0;

        double f = find_min_bobyqa(
                OptEyeOffset<EyeModelT>(samples, screen), 
                starting_point, 
                starting_point.size()*2+1,    // number of interpolation points
                lower_bounds,
                upper_bounds,
                0.05,
                0.0002,
                5000
                );
        T_trans = Vector3f(starting_point(0), starting_point(1), starting_point(2));
        T_rot   = Vector3f(starting_point(3), starting_point(4), starting_point(5));
        std::cout << " f eye: " << f << "  T: " << T_trans.transpose() << "  " << T_rot.transpose() << std::endl;
        return f;
    }
    
#if MOORE_NO_ROTATION
    template<> 
    inline double OptEyeOffset<EyeModelMoore>::operator()(const column_vector &p) const
    {
        Vector3f T_trans(p(0), p(1), p(2));
        Vector3f T_rot(0,0,0);

        std::vector<Vector2f> pupils;
        std::vector<Vector2f> angles;
        for (Samples::const_iterator s = samples.begin(); s != samples.end(); s++)
        {
            float yaw, pitch;
            scr.calcGazeAngles(s->target_pos, s->H_pos, s->H_o, T_trans, T_rot, yaw, pitch);
            pupils.push_back(Vector2f(s->px_left, s->py_left));
            angles.push_back(Vector2f(yaw, pitch));
        }

        EyeModelMoore eyeModel;
        eyeModel.fit(pupils, angles);

        return getMse(eyeModel, T_trans, T_rot, samples, scr);
    }

    template<> 
    inline double findEyeTransform<mug::EyeModelMoore>(
        const Samples &samples, const ScreenModel &screen,
        Vector3f &T_trans, Vector3f &T_rot)
    {
        int param_dim = 3;
        column_vector starting_point(param_dim);
        column_vector lower_bounds(param_dim);
        column_vector upper_bounds(param_dim);

        lower_bounds   = -.15, -.00, -0.20;
        upper_bounds   =  .0,  .15,  0.0;
        starting_point =  -0.05, 0.03, -0.10;

        double f = find_min_bobyqa(
                OptEyeOffset<mug::EyeModelMoore>(samples, screen), 
                starting_point, 
                starting_point.size()*2+1,    // number of interpolation points
                lower_bounds,
                upper_bounds,
                0.05,
                0.0002,
                5000
                );
        T_trans = Vector3f(starting_point(0), starting_point(1), starting_point(2));
        T_rot   = Vector3f(0,0,0);
        std::cout << " f eye: " << f << "  T: " << T_trans.transpose() << "  " << T_rot.transpose() << std::endl;
        return f;
    }

#endif

}

#endif

