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

#ifndef SAMPLE_H
#define SAMPLE_H

#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <iostream>

using namespace Eigen;

namespace mug
{
    /** 
     * \brief Class structure representing a gazetracking data sample
     */
    class Sample
    {
        public:
            unsigned int timestamp;     ///< Recording time in [ms]
            float px_left;              ///< Image X of left pupil 
            float py_left;              ///< Image Y of left pupil 
            float px_right;             ///< Image X of right pupil 
            float py_right;             ///< Image Y of right pupil 
            Vector3f H_pos;             ///< Head position in [m]
            //Quaternionf H_o;          ///< Head orientation as quaternion 
            Vector3f H_o;               ///< Head orientation in Euler angles [rad]
            Vector2f target_pos;        ///< Screen position of fixation target [px]
            float yaw;                  ///< Eye azimuth [rad]
            float pitch;                ///< Eye inclination [rad]
    };

    /** 
     * \brief Dataset containing vector of \ref Sample objects
     */
    typedef std::vector<Sample> Samples;

    /** 
     * \brief Write sample to stream
     * \param[in] file Output stream to write to
     * \param[in] s The samples to save
     */
    inline void saveSample(std::ofstream &file, const Sample &s)
    {
        file.precision(10);
        file << s.timestamp << " " 
            << s.H_pos.transpose() << " "
            << s.H_o.transpose() << " " 
            << s.px_left << " " << s.py_left << " " 
            << s.px_right << " " << s.py_right << " " 
            << 0 << " " << 0 << " " // dummies, to maintain compatibility with old data files
            << 0 << " " << 0 << " " // 
            << s.target_pos.transpose() 
            << std::endl;

    }

    /** 
     * \brief Save data to file. Each \ref Sample will be stored as one line in output file.
     * \param[in] samples Data to be saved
     * \param[in] filename Name of output file
     */
    inline void saveSamples(const std::vector<Sample> &samples, const std::string &filename)
    {
        std::ofstream file(filename.c_str());
        if (! file.is_open())
            return;

        for (int i=0; i< samples.size(); i++)
        {
            saveSample(file, samples[i]);
        }

        file.close();
    }

    /** 
     * \brief Read a sample from an input stream
     * \param[in] file Input stream
     * \param[out] s \ref Sample object to store data in
     * \return True if the load was successful, False otherwise.
     */
    inline bool loadSample(std::ifstream &file, Sample &s)
    {
        // dummy vars to load data files containing Eyelink predictions
        float gx_left, gx_right, gy_left, gy_right; 

        if (file >> s.timestamp 
                >> s.H_pos[0] >> s.H_pos[1] >> s.H_pos[2]
                >> s.H_o[2] >> s.H_o[1] >> s.H_o[0] // pitch and yaw are switched in data files (2<->0)
                >> s.px_left >> s.py_left
                >> s.px_right >> s.py_right
                >> gx_left >> gy_left
                >> gx_right >> gy_right
                >> s.target_pos[0] >> s.target_pos[1])
        {
            // Values are in ranges [PI,2*PI] and [0, PI]  
            // Shift so that values are in range [0,2*PI]
            if (s.H_o[0] < 0)
                s.H_o[0] += 2*M_PI;
            s.H_o[0] -= M_PI;
            s.H_o[0] *= -1;
            // Pupil values are in range [5000-25000]. Scale down for numeric reasons. 
            s.px_left  /= 10000;
            s.py_left  /= 10000;
            s.px_right /= 10000;
            s.py_right /= 10000;
            return true;
        }
        else
            return false;
    }

    /** 
     * \brief Load dataset from file
     * \param[in] filename Path and name of data file
     * \param[in] swap Boolean value that indicate whether yaw and pitch 
     * \return Vector of samples with one sample per line in input file.
     */
    inline std::vector<Sample> loadSamples(const std::string &filename)
    {
        std::vector<Sample> samples;
        std::ifstream file(filename.c_str());
        if (! file.is_open())
        {
            std::cerr << "Could not open file " << filename << std::endl;
            return samples;
        }

        Sample s;
        while (loadSample(file, s))
        {
            samples.push_back(s);
        }
        file.close();

        return samples;
    }
    
}

#endif
