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

#ifndef SCREEN_MODEL_FLAT_H
#define SCREEN_MODEL_FLAT_H

#include <vector>

#include <Eigen/Dense>

#include <mug/screen_model.h>
#include <mug/sample.h>
#include <mug/eye_model.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace mug
{
    class ScreenModelFlat;

    /** 
     * \brief Functor implementing target function for optimization of screen model coefficients.
     */
    template <class EyeModelT, class ScreenModelT>
    class EvalParams
    {
    public:
        EvalParams(const std::vector<std::vector<Sample> > &samples_)
            : sampleList(samples_.begin(), samples_.end()), nIters(0)
        {}

        /** 
        * \brief Evaluation function called in each optimization iteration. 
        * Returns prediction error given current screen coefficients p. 
        * \param[in] p Vector (c1, c2, c3, o1, o2, o3) specifying center and orientation.
        * \return Mean squared error
        */
        double operator()(const column_vector& p) const
        {
            nIters++;

            Vector3f center(p(0),p(1),p(2));
            Vector3f orient(p(3),p(4),p(5));
            ScreenModelT scr(center, orient);

            double mse = 0;
            for (int i = 0; i < sampleList.size(); i++)
            {
                Vector3f T_trans, T_rot;
                mse += findEyeTransform<EyeModelT>(sampleList[i], scr, T_trans, T_rot);
            }

            mse /= sampleList.size();
            std::cout << nIters << "  mse: " << mse << "  " <<   dlib::trans(p);

            return mse;
        }
    private:
        mutable int nIters;
        std::vector<Samples> sampleList;
    };

    /** 
     * \brief Optimization for finding screen model coefficients.
     */
    template <typename EyeModelT>
    class ScreenModelFlatOpt
    {
        public:

        /** 
        * \brief Runs non-linear optimization to fit screen center and orientation to datasets.
        * \param[in] datasets Vector of training datasets
        * \param[out] center Found screen center point (3D)
        * \param[out] orientation Found screen orientation
        */
        void run(const std::vector<Samples> &datasets, Eigen::Vector3f &center, Eigen::Vector3f &orientation)
        {
            const int param_dim = 6;
            column_vector params(param_dim);
            column_vector lower_bounds(param_dim);
            column_vector upper_bounds(param_dim);

            // search space
            lower_bounds   = -0.40, -0.70, 1.00, -M_PI/4, M_PI/180 * -15, -M_PI/4;
            upper_bounds   =  0.30,  0.30, 1.60,  M_PI/4, M_PI/180 * 15,  M_PI/4;

            // starting point
            params         =  0.0,  -0.10, 1.30, -0.00,   M_PI/180 * 5,  -0.00;

            const double rho_start     = 0.15;    // initial trust region radius
            const double rho_stop      = 0.001;   // stopping trust region radius
            const double maxIterations = 5000;    // max number of objective function evaluations

            std::cout << "Optimizing screen coefficients..." << std::endl;

            for (int i=0; i < 1; i++)
            {
                double f = find_min_bobyqa(
                        EvalParams<EyeModelT, ScreenModelFlat>(datasets), 
                        params, 
                        params.size()*2+1,    // number of interpolation points
                        lower_bounds,
                        upper_bounds,
                        rho_start,
                        rho_stop,
                        maxIterations
                        );

                std::cout << " f: " << f << std::endl;
            }
            center      << params(0), params(1), params(2);
            orientation << params(3), params(4), params(5);
        }
    };


    /** 
     * \brief ScreenModel implementation of a flat display surface. 
     * ScreenModelFlat flat assumes that the display is a planar surface, 
     * defined by a center point, an orientation, width and height.
     */
    class ScreenModelFlat : public ScreenModel
    {
    public:
        typedef Eigen::Hyperplane<float, 3>       Planef;
        typedef Eigen::ParametrizedLine<float, 3> Linef;

        ScreenModelFlat()
        {}

        /** 
         * \brief Constructor. Creates a planar surface based on the supplied parameters.
         * \param[in] c screen center [m]
         * \param[in] o screen orientation in Euler angles [rad]
         * \param[in] w_ screen width [m]
         * \param[in] h_ screen height [m]
         * \param[in] res_x_ screen resolution X [px]
         * \param[in] res_y_ screen resolution Y [px]
         * \return  2D screen coordinate [px]
         */
        ScreenModelFlat(const Vector3f &c, const Vector3f &o, double w_ = 2.15, double h_ = 1.62, 
                int res_x_ = 1024, int res_y_ = 768)
        {
            create(c, o, w_, h_, res_x_, res_y_);
        }

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
        Eigen::Vector2f project(const Eigen::Vector3f &H_pos, const Eigen::Vector3f &H_o, 
                const Eigen::Vector3f &T_trans, const Eigen::Vector3f &T_rot, 
                float yaw, float pitch) const;

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
        void calcGazeAngles(const Eigen::Vector2f &targetUV, 
                const Eigen::Vector3f &H_pos, const Eigen::Vector3f &H_o, 
                const Eigen::Vector3f &T_trans, const Eigen::Vector3f &T_rot,
                float &yaw, float &pitch) const;

        /** 
         * \brief Saves screen model coefficients to file.
         * \param[in] filename Name and path of to be created 
         */
        void save(const std::string &filename) const;

        /** 
         * \brief Loads screen model coefficients from file and initializes surface.
         * \param[in] filename Name and path of file to be loaded
         */
        void load(const std::string &filename);


        /** 
         * \brief Finds screen coefficients by fitting model to supplied data.
         * Optimization is carried out based on the specified eye model \ref EyeModel "EyeModelT".
         * \param[in] dataFiles Vector of filenames. Each file contains a set of training samples.  
         */
        template<class EyeModelT>
        void calibrate(const std::vector<std::string> &dataFiles)
        {
            // load data
            std::vector<Samples> datasets;
            for (std::vector<std::string>::const_iterator f = dataFiles.begin(); 
                    f != dataFiles.end(); f++)
            {
                Samples samples = loadSamples(*f);
                samples = reduceSampleRate(16, samples);
                datasets.push_back(samples);
            }

            // run non-linear optimization to estimate center and orientation
            ScreenModelFlatOpt<EyeModelT> opt;
            opt.run(datasets, center, orientation);

            // apply found parameters
            create(center, orientation);
        }

        inline Eigen::Vector3f getCenter() const
        {
            return center;
        }
        inline Eigen::Vector3f getOrientation() const
        {
            return orientation;
        }

    private:
        Planef plane;                   ///< Plane in 3D space 

        Eigen::Vector3f topLeft;        ///< 3D coordinate of top left screen corner
        Eigen::Vector3f topRight;       ///< 3D coordinate of top right screen corner
        Eigen::Vector3f bottomRight;    ///< 3D coordinate of bottom right screen corner

        Eigen::Vector3f center;         ///< 3D coordinate of screen center point
        Eigen::Vector3f orientation;    ///< Screen orientation [pitch, roll, yaw]

        int width_px;                   ///< Width in pixels
        int height_px;                  ///< Height in pixels
        double width_m;                 ///< Width in meters
        double height_m;                ///< Height in meters

        /** 
         * \brief Returns normalized vector.
         * \param[in] v Vector
         * \return Normalized vector
         */
        inline Eigen::Vector3f normalize(const Eigen::Vector3f &v) const
        {
            return v/sqrt(v.dot(v));
        }

        /** 
         * \brief Return angle between two 3D vectors. Vectors do not need to be normalized.
         * \param[in] n First vector
         * \param[in] u Second vector
         * \return Angle in radians
         */
        inline double angle(const Eigen::Vector3f &n, const Eigen::Vector3f &u) const
        {
            double norm = sqrt(n.dot(n))*sqrt(u.dot(u));
            double a = n.dot(u) / norm;
            return asin(std::max(std::min(1.0, a),-1.0));
        }

        /** 
         * \brief Converts 2D screen coordinate to 3D world coordinate 
         * \param[in] uv 2D point
         * \return 3D point
         */
        inline Eigen::Vector3f getPixelCoordXYZ(const Eigen::Vector2f &uv) const
        {
            float u = uv[0];
            float v = uv[1];

            if (u <= 0) u = 0;
            if (u >= width_px) u = width_px;
            if (v <= 0) v = 0;
            if (v >= height_px) v = height_px;

            Eigen::Vector3f vec_u = topRight - topLeft;
            Eigen::Vector3f vec_v = bottomRight - topRight;
            Eigen::Vector3f pos_x = topLeft   + u/width_px * vec_u;
            Eigen::Vector3f pos   = pos_x     + v/height_px * vec_v;

            return pos;
        }
            
        /** 
         * \brief Converts 3D world coordinate to 2D screen coordinate 
         * \param[in] xyz 3D point
         * \return 2D point
         */
        inline Eigen::Vector2f getPixelCoordUV(const Eigen::Vector3f &xyz) const
        {
            Linef lu = Linef::Through(topLeft, topRight);
            Linef lv = Linef::Through(topRight, bottomRight);

            float dv = lu.distance(xyz) / lu.distance(bottomRight);
            float du = lv.distance(xyz) / lv.distance(topLeft);

            return Eigen::Vector2f(width_px-du*width_px,dv*height_px);
        }

        inline void getEyePose(const Eigen::Vector3f &H_pos, const Eigen::Vector3f &H_o, 
                const Eigen::Vector3f &T_trans, const Eigen::Vector3f &T_rot,
                Eigen::Vector3f &E_pos, Eigen::Vector3f &E_o) const
        {
            Eigen::Matrix3f r ; r =  
                Eigen::AngleAxisf(-H_o[2], Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(H_o[0], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(H_o[1], Eigen::Vector3f::UnitY());

            E_pos = H_pos + (r * T_trans);
            E_o   = H_o + T_rot;
        }

        inline std::vector<Sample> reduceSampleRate(int n, const std::vector<Sample> &samples)
        {
            std::vector<Sample> res;
            for (int i=0; i<samples.size(); i++)
            {
                if (i % n == 0)
                    res.push_back(samples[i]);
            }
            return res;
        }

        void create(const Eigen::Vector3f &c, const Eigen::Vector3f &o, 
                double w_ = 2.15, double h_ = 1.62,
                int res_x_ = 1024, int res_y_ = 768);
    };
}
    

#endif

