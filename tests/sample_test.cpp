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
  std::vector<mug::Sample> samples;
  
  float eyeLeftX[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float eyeLeftY[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float eyeRightX[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float eyeRightY[20] = {50,43,67,101,23,46,-44,37,85,23,53,214,36,43,52,78,65,43,12,32};
  float orientation[20] = {0.2,1.4,2.2,-2.3,-0.2,3.0,-2.1,-1.2,2.3,1.2,1.6,1.3,1.5,1.2,3.2,3.0,2.1,0.5,0.7,0.8};
  float position[20] = {0.2,1.4,2.2,-2.3,-0.2,3.0,-2.1,-1.2,2.3,1.2,1.6,1.3,1.5,1.2,3.2,3.0,2.1,0.5,0.7,0.8};
  
  // fill the samples vector
  for (unsigned int i = 0; i < 20; ++i)
  {
    mug::Sample s;
    s.timestamp = i;
    s.H_o[0] = orientation[i];
    s.H_o[1] = orientation[i];
    s.H_o[2] = orientation[i];
    s.H_pos[0] = position[i];
    s.H_pos[1] = position[i];
    s.H_pos[2] = position[i];
    s.px_left = eyeLeftX[i];
    s.py_left = eyeLeftY[i];
    s.px_right = eyeRightX[i];
    s.py_right = eyeRightY[i];
    samples.push_back(s);
  }
  
  mug::rescaleData(samples, -44, 214, -2.3, 3.2, -2.3, 3.2);
    
  // test whether the eye coordinates are between 0 and 2
  for (unsigned int j = 0; j < 20; ++j)
  {
    EXPECT_LE(samples[j].px_left, 2) << "X coordinate of the left eye is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].px_left, 0) << "X coordinate of the left eye is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].py_left, 2) << "Y coordinate of the left eye is greater than 2 at position " << j;
    EXPECT_GE(samples[j].py_left, 0) << "Y coordinate of the left eye is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].px_right, 2) << "X coordinate of the right eye is greater than 2 at position " << j;
    EXPECT_GE(samples[j].px_right, 0) << "X coordinate of the right eye is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].py_right, 2) << "Y coordinate of the right eye is greater than 2 at position " << j;
    EXPECT_GE(samples[j].py_right, 0) << "Y coordinate of the right eye is smaller than 0 at position " << j;
    
    /*EXPECT_LE(samples[j].H_o[0], 2) << "H_o[0] is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].H_o[0], 0) << "H_o[0] is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].H_o[1], 2) << "H_o[1] is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].H_o[1], 0) << "H_o[1] is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].H_o[2], 2) << "H_o[2] is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].H_o[2], 0) << "H_o[2] is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].H_pos[0], 2) << "H_pos[0] is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].H_pos[0], 0) << "H_pos[0] is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].H_pos[1], 2) << "H_pos[1] is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].H_pos[1], 0) << "H_pos[1] is smaller than 0 at position " << j;
    
    EXPECT_LE(samples[j].H_pos[2], 2) << "H_pos[2] is greater than 2 at position " << j; 
    EXPECT_GE(samples[j].H_pos[2], 0) << "H_pos[2] is smaller than 0 at position " << j;*/
  }
}