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
 * 
 * Author: Jonas Ditz [jonas.ditz@tuebingen.mpg.de]
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <mug/sample.h>

#include <math.h>
#include <algorithm>
#include <Eigen/Dense>


TEST(SampleTest, RescaleEye){
  mug::Samples samples;
  
  float eyeLeftX[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float eyeLeftY[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float eyeRightX[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float eyeRightY[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  
  // fill the samples vector
  for (unsigned int i = 0; i < 20; ++i)
  {
    mug::Sample s;
    s.timestamp = i;
    s.px_left = eyeLeftX[i];
    s.py_left = eyeLeftY[i];
    s.px_right = eyeRightX[i];
    s.py_right = eyeRightY[i];
    samples.push_back(s);
  }
  
  mug::rescaleEye(samples, -44, 214);
    
  // test whether the eye coordinates are between 0 and 2
  for (unsigned int j = 0; j < 20; ++j)
  {
    EXPECT_LE(samples[j].px_left, 2) << "X coordinate of the left eye is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].px_left, 0) << "X coordinate of the left eye is smaller than 2 at position " << j;
    
    EXPECT_LE(samples[j].py_left, 2) << "Y coordinate of the left eye is greater than 2 at position " << j;
    EXPECT_GE(samples[j].py_left, 0) << "Y coordinate of the left eye is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].px_right, 2) << "X coordinate of the right eye is greater than 2 at position " << j;
    EXPECT_GE(samples[j].px_right, 0) << "X coordinate of the right eye is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].py_right, 2) << "Y coordinate of the right eye is greater than 2 at position " << j;
    EXPECT_GE(samples[j].py_right, 0) << "Y coordinate of the right eye is smaller than 0 at position " << j;
  }
}