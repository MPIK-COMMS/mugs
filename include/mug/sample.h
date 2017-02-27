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
#include <limits>

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
            float du, dv;               ///< Eye offset
            float yaw;                  ///< Eye azimuth [rad]
            float pitch;                ///< Eye inclination [rad]
    };

    /** 
     * \brief Class structure containing a vector of \ref Sample objects and
     * minimum and maximum values for each of the three domains: head position,
     * head orientation and eye position.
     */
    class Samples
    {
        public:
	    std::vector<Sample> samples;     ///< Vector of Sample objects 
	    float min_pos;                   ///< Minimum of the head position domain
	    float min_o;                     ///< Minimum of the head orientation domain
	    float min_eye;                   ///< Minimum of the eye position domain
	    float max_pos;                   ///< Maximum of the head position domain
	    float max_o;                     ///< Maximum of the head orientation domain
	    float max_eye;                   ///< Maximum of the eye position domain
	    
	    /**
	     * \brief Constructor
	     */
	    Samples ()
	      : min_pos(std::numeric_limits<float>::max()), min_o(std::numeric_limits<float>::max()),
	        min_eye(std::numeric_limits<float>::max()), max_pos(std::numeric_limits<float>::min()),
	        max_o(std::numeric_limits<float>::min()), max_eye(std::numeric_limits<float>::min())
	    {}
	    
	    /**
	     * \brief Add a Samples object to the class variable samples.
	     * \param[in] s Sample object to be added to samples;
	     */
	    inline void addSample (const Sample & s)
	    {
	        samples.push_back(s);
	    }
	    
	    /**
	     * \brief get the number of samples.
	     * \return Size of the vector samples.
	     */
	    inline unsigned int getSamplesSize()
	    {
	        return samples.size();
	    }
    };

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
     * \brief Scale the range of the tracking data to [0,2].
     * \param[out] samples Vector containing tracking data.
     * \param[in] min_eye Minimum value of the eye position domain.
     * \param[in] max_eye Maximum value of the eye position domain.
     * \param[in] min_o Minimum value of the head orientation domain.
     * \param[in] max_o Maximum value of the head orientation domain.
     * \param[in] min_pos Minimum value of the head position domain.
     * \param[in] max_pos Maximum value of the head position domain.
     */
    inline void rescaleData(std::vector<Sample> &samples, const float &min_eye, const float &max_eye,
                            const float &min_o, const float &max_o,
			    const float &min_pos, const float &max_pos)
    {
        for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
	{
	    it->px_left = (2*(it->px_left - min_eye))/(max_eye - min_eye);
	    it->py_left = (2*(it->py_left - min_eye))/(max_eye - min_eye);
	    it->px_right = (2*(it->px_right - min_eye))/(max_eye - min_eye);
	    it->py_right = (2*(it->py_right - min_eye))/(max_eye - min_eye);
	    
	    //it->H_o[0] = (2*(it->H_o[0] - min_o))/(max_o - min_o);
	    //it->H_o[1] = (2*(it->H_o[1] - min_o))/(max_o - min_o);
	    //it->H_o[2] = (2*(it->H_o[2] - min_o))/(max_o - min_o);
	    
	    //it->H_pos[0] = (2*(it->H_pos[0] - min_pos))/(max_pos - min_pos);
	    //it->H_pos[1] = (2*(it->H_pos[1] - min_pos))/(max_pos - min_pos);
	    //it->H_pos[2] = (2*(it->H_pos[2] - min_pos))/(max_pos - min_pos);
	}
    }

    /** 
     * \brief Read a sample from an input stream
     * \param[in] file Input stream
     * \param[out] s \ref Sample object to store data in
     * \param[out] min_eye Minimum of the eye position domain.
     * \param[out] max_eye Maximum of the eye position domain.
     * \param[out] min_o Minimum of the head orientation domain.
     * \param[out] max_o Maximum of the head orientation domain.
     * \param[out] min_pos Minimum of the head position domain.
     * \param[out] max_pos Maximum of the head position domain.
     * \return True if the load was successful, False otherwise.
     */
    inline bool loadSample(std::ifstream &file, Sample &s, float &min_eye, float &max_eye, 
			   float &min_o, float &max_o, float &min_pos, float &max_pos)
    {
        // dummy vars to load data files containing Eyelink predictions
        float gx_left, gx_right, gy_left, gy_right; 

        if (file >> s.timestamp 
                >> s.H_pos[0] >> s.H_pos[1] >> s.H_pos[2]
                >> s.H_o[0] >> s.H_o[1] >> s.H_o[2]
                >> s.px_left >> s.py_left
                >> s.px_right >> s.py_right
                >> gx_left >> gy_left
                >> gx_right >> gy_right
                >> s.target_pos[0] >> s.target_pos[1])
        { 
            // determine the min and max of the tracking data for the domains eye, head orientation and
	    // head position
	    float auxMin_eye = std::min(s.px_left, std::min(s.py_left, std::min(s.px_right, s.py_right)));
	    float auxMax_eye = std::max(s.px_left, std::max(s.py_left, std::max(s.px_right, s.py_right)));
	    float auxMin_o = std::min(s.H_o[0], std::min(s.H_o[1], s.H_o[2]));
	    float auxMax_o = std::max(s.H_o[0], std::max(s.H_o[1], s.H_o[2]));
	    float auxMin_pos = std::min(s.H_pos[0], std::min(s.H_pos[1], s.H_pos[2]));
	    float auxMax_pos = std::max(s.H_pos[0], std::max(s.H_pos[1], s.H_pos[2]));
	    if (min_eye > auxMin_eye) {min_eye = auxMin_eye;}
	    if (max_eye < auxMax_eye) {max_eye = auxMax_eye;}
	    if (min_o < auxMin_o) {min_o = auxMin_o;}
	    if (max_o < auxMax_o) {max_o = auxMax_o;}
	    if (min_pos < auxMax_pos) {min_pos = auxMax_pos;}
	    if (max_pos < auxMax_pos) {max_pos = auxMax_pos;}

	    return true;
        }
        else
            return false;
    }

    /** 
     * \brief Load dataset from file
     * \param[in] filename Path and name of data file 
     * \return Vector of samples with one sample per line in input file.
     */
    inline Samples loadSamples(const std::string &filename)
    {
        Samples samples;
        std::ifstream file(filename.c_str());
        if (! file.is_open())
        {
            std::cerr << "Could not open file " << filename << std::endl;
            return samples;
        }

        Sample s;
        while (loadSample(file, s, samples.min_eye, samples.max_eye, samples.min_o, samples.max_o, samples.min_pos, samples.max_pos))
        {
            samples.addSample(s);
        }
        file.close();

        return samples;
    }
    
}

#endif
