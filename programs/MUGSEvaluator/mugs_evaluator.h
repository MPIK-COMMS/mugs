/* 
 * Copyright (c) 2017 Max Planck Institute for Biological Cybernetics
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
 * 
 * Author: Jonas Ditz [jonas.ditz@tuebingen.mpg.de], Bjoern Browatzki
 */

#include <string.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

enum FileFormat
{
  CSV = 1,
};


/**
 * \brief This function stores calculated gaze positions as csv files.
 *        The structure of the csv file is:
 *            Gaze position X, Gaze position Y, Confidence
 * \param[in] gaze Eigen::Vector3f that holds the gaze position and 
 *                 confidence information.
 * \param[in] file Output stream to the gaze file.
 */
inline void writeCSV(const Eigen::Vector3f & gaze, std::ofstream &file)
{
  file << gaze[0] << "," << gaze[1] << "," << gaze[2] << std::endl;
}

/**
 * \brief Writer function to write the header of a csv-styled gaze 
 *        output file.
 * \param[in] file Output stream to the gaze file.
 */
inline void writeCSVheader(std::ofstream &file)
{
  file << "# Gaze position X [px], Gaze position Y [px], Confidence" << std::endl;
}

/**
 * \brief Auxiliary function to map FileFormat to the right writer-function.
 * \param[in] gaze Eigen::Vector3f that holds the gaze position and 
 *                 confidence information.
 * \param[in] file Output stream to the gaze file.
 * \param[in] ff This variable holds the information which writer-function 
 *               should be used.
 */
inline void writeGaze(const Eigen::Vector3f & gaze, std::ofstream &file, FileFormat ff)
{
  switch(ff) 
  {
    case CSV : writeCSV(gaze, file);
               break;
  }
}

/**
 * \brief Auxiliary function to map FileFormat to the right writer-function in 
 *        order to write the right header to the output file.
 * \param[in] file Output stream to the gaze file.
 * \param[in] ff This variable holds the information which writer-function 
 *               should be used.
 */
inline void writeHeader(std::ofstream &file, FileFormat ff)
{
  switch(ff) 
  {
    case CSV : writeCSVheader(file);
               break;
  }
}