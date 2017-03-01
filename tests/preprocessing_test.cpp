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
 * Author: Jonas Ditz [jonas.ditz@tuebingen.mpg.de]
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <mug/preprocessing.h>

#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <mug/sample.h>
#include <mug/eye_model.h>

TEST(PreprocessingTest, SimpleDerivative){
  // test for integer
  std::vector<int> testData_int;
  std::vector<int> testDeri_int;
  std::vector<int> testResult_int;
  
  // fill test data vetor
  testData_int.push_back(1); testData_int.push_back(1); testData_int.push_back(1); testData_int.push_back(1);
  testData_int.push_back(3); testData_int.push_back(3); testData_int.push_back(3); testData_int.push_back(3);
  testData_int.push_back(1); testData_int.push_back(1); testData_int.push_back(1); testData_int.push_back(1);
  
  // fill test result vector
  testResult_int.push_back(0); testResult_int.push_back(0); testResult_int.push_back(0); testResult_int.push_back(0);
  testResult_int.push_back(2); testResult_int.push_back(0); testResult_int.push_back(0); testResult_int.push_back(0);
  testResult_int.push_back(-2); testResult_int.push_back(0); testResult_int.push_back(0); testResult_int.push_back(0);
  
  mug::simpleDerivative(testData_int, testDeri_int);
  
  // test for float
  std::vector<float> testData_float;
  std::vector<float> testDeri_float;
  std::vector<float> testResult_float;
  
  // fill test data vetor
  testData_float.push_back(1.5); testData_float.push_back(1.5); testData_float.push_back(1.5); testData_float.push_back(1.5);
  testData_float.push_back(3.3); testData_float.push_back(3.3); testData_float.push_back(3.3); testData_float.push_back(3.3);
  testData_float.push_back(1.3); testData_float.push_back(1.3); testData_float.push_back(1.3); testData_float.push_back(1.3);
  
  // fill test result vector
  testResult_float.push_back(0.0); testResult_float.push_back(0.0); testResult_float.push_back(0.0); testResult_float.push_back(0.0);
  testResult_float.push_back(1.8); testResult_float.push_back(0.0); testResult_float.push_back(0.0); testResult_float.push_back(0.0);
  testResult_float.push_back(-2.0); testResult_float.push_back(0.0); testResult_float.push_back(0.0); testResult_float.push_back(0.0);
  
  mug::simpleDerivative(testData_float, testDeri_float);
  
  EXPECT_THAT(testResult_int, ::testing::ContainerEq(testDeri_int));
  for(unsigned int i = 0; i < testDeri_float.size(); ++i)
  {
    EXPECT_THAT(testDeri_float[i], ::testing::FloatEq(testResult_float[i]));
  }
}

TEST(PreprocessingTest, RemoveSmoothPersuitMarker){
  std::vector<int> markers;
  std::vector<int> result;
  
  // fill the markers vector
  markers.push_back(1); markers.push_back(2); markers.push_back(3); markers.push_back(4); markers.push_back(5);
  markers.push_back(6); markers.push_back(7); markers.push_back(10); markers.push_back(25); markers.push_back(40);
  markers.push_back(55); markers.push_back(80); markers.push_back(105); markers.push_back(133); markers.push_back(150);
  
  // fill the expected result vector
  result.push_back(7); result.push_back(10); result.push_back(25); result.push_back(40); result.push_back(55);
  result.push_back(80); result.push_back(105); result.push_back(133); result.push_back(150);
  
  mug::removeSmoothPersuitMarker(markers);
  
  EXPECT_THAT(result, ::testing::ContainerEq(markers));
}

TEST(PreprocessingTest, RemoveSamples){
  std::vector<mug::Sample> test_samples;
  std::vector<mug::Sample> result_samples;
  std::vector<Eigen::Vector2i> removeAreas;

  // create test and result data  
  for(unsigned int i = 0; i < 20; ++i)
  {
    mug::Sample s;
    s.timestamp = i;
    s.px_left = 1.5;
    s.py_left = 1.5;
    if (i < 10 || i > 15)
    {
      result_samples.push_back(s);
    }
    test_samples.push_back(s);
  }
  
  // specify the area that should be removed
  Eigen::Vector2i area;
  area[0] = 10; area[1] = 16;
  removeAreas.push_back(area);
  
  mug::removeSamples(test_samples, removeAreas);
  
  // test if both arrays are equal
  ASSERT_EQ(test_samples.size(), result_samples.size()) << "Vectors test_samples and result_samples are of unequal length";
  for(unsigned int i = 0; i < test_samples.size(); ++i)
  {
    EXPECT_EQ(test_samples[i].timestamp, result_samples[i].timestamp) << "Vectors test_samples and result_samples differ at index " << i;
  }
}

TEST(PreprocessingTest, GetLocalExtrema){
  std::vector<float> testData;
  std::vector<int> extrema_order1;
  std::vector<int> extrema_order3;
  std::vector<int> results_order1;
  std::vector<int> results_order3;
  
  // fill the data vector
  testData.push_back(1.5); testData.push_back(1.5); testData.push_back(1.5); testData.push_back(3.5);
  testData.push_back(3.3); testData.push_back(3.5); testData.push_back(3.8); testData.push_back(3.3);
  testData.push_back(1.3); testData.push_back(-0.4); testData.push_back(1.3); testData.push_back(1.3);
  
  // create the expected results 
  extrema_order1.push_back(3); extrema_order1.push_back(4); extrema_order1.push_back(6); extrema_order1.push_back(9);
  extrema_order3.push_back(6); extrema_order3.push_back(9);
  
  // calculate extrema 
  mug::getLocalExtrema(testData, 1, results_order1);
  mug::getLocalExtrema(testData, 3, results_order3);
  
  EXPECT_THAT(results_order1, ::testing::ContainerEq(extrema_order1));
  EXPECT_THAT(results_order3, ::testing::ContainerEq(extrema_order3));
}