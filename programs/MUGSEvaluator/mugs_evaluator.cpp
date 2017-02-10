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
 * Author: Jonas Ditz [jonas.ditz@tuebingen.mpg.de], Bjoern Browatzki
 */

#include <string.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <math.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <mug/gaze_tracker.h>
#include <mug/screen_model_flat.h>
#include <mug/eye_model_linear.h>
#include <mug/eye_model_moore.h>
#include <mug/eye_model_gp.h>

#include "util.h"

using namespace mug;

// A helper function to simplify the main part.
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " ")); 
    return os;
}

double evaluateTracker( const GazeTracker<EyeModelGp> &gt, Samples samples)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = sqrt(eu*eu + ev*ev);

    }

    merr_u /= samples.size();
    merr_v /= samples.size();

    std::cout << "Avg. errors u/v (px) : " << merr_u   << " " << merr_v << std::endl;

    return mse / samples.size();
}

double evaluateTracker( const GazeTracker<EyeModelLinear> &gt, Samples samples)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = sqrt(eu*eu + ev*ev);

    }

    merr_u /= samples.size();
    merr_v /= samples.size();

    std::cout << "Avg. errors u/v (px) : " << merr_u   << " " << merr_v << std::endl;

    return mse / samples.size();
}

double evaluateTracker( const GazeTracker<EyeModelMoore> &gt, Samples samples)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = sqrt(eu*eu + ev*ev);

    }

    merr_u /= samples.size();
    merr_v /= samples.size();

    std::cout << "Avg. errors u/v (px) : " << merr_u   << " " << merr_v << std::endl;

    return mse / samples.size();
}

double evaluateTracker( const GazeTracker<EyeModelGp> &gt, Samples samples, std::ofstream &file)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = sqrt(eu*eu + ev*ev);

        file.precision(10);
        file << uv[0] << " " << uv[1] << " " << uv[2] << std::endl;
    }

    merr_u /= samples.size();
    merr_v /= samples.size();

    std::cout << "Avg. errors u/v (px) : " << merr_u   << " " << merr_v << std::endl;

    return mse / samples.size();
}

double evaluateTracker( const GazeTracker<EyeModelLinear> &gt, Samples samples, std::ofstream &file)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = sqrt(eu*eu + ev*ev);

        file.precision(10);
        file << uv[0] << " " << uv[1] << " " << uv[2] << std::endl;
    }

    merr_u /= samples.size();
    merr_v /= samples.size();

    std::cout << "Avg. errors u/v (px) : " << merr_u   << " " << merr_v << std::endl;

    return mse / samples.size();
}

double evaluateTracker( const GazeTracker<EyeModelMoore> &gt, Samples samples, std::ofstream &file)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = sqrt(eu*eu + ev*ev);

        file.precision(10);
        file << uv[0] << " " << uv[1] << " " << uv[2] << std::endl;
    }

    merr_u /= samples.size();
    merr_v /= samples.size();

    std::cout << "Avg. errors u/v (px) : " << merr_u   << " " << merr_v << std::endl;

    return mse / samples.size();
}

int main(int argc, char ** argv)
{
    // Handle command line arguments.
    bool optimizeScreen;
    int reduceRate;
    std::string eyeModel, trainFile, testFile, outputFile, configFile, modelType;
    ModelType mt;
    try {
        // Declare a group of options that will be allowed
        // only on command line.
	po::options_description generic("Generic options");
	generic.add_options()
	    ("help,h", "Show this help message.")
	    ("version,v", "Print version string.")
	    ("config,c", po::value<std::string>(&configFile), "Name of a file of a configuration.")
	    ;
	
	// Declare a group of options that will be allowed 
	// both on command line and in a config file.
	po::options_description config("Configuration");
	config.add_options()
	    ("optimize,O", po::value<bool>(&optimizeScreen)->default_value(false), 
	     "Specify, whether or not to opimize the screen")
	    ("reduce,r", po::value<int>(&reduceRate)->default_value(1),
	     "Set the rate by which the dataset should be reduced in order to accelerate training and testing.")
	    ("model,m", po::value<std::string>(&eyeModel)->default_value("EyeModelGp"),
	     "Specify the eye model used for the gaze tracker. Possible values are EyeModelGp, EyeModelLinear and EyeModelMoore.")
	    ("type,t", po::value<std::string>(&modelType)->default_value("EYE_LEFT"),
	     "Specify the eye, which will be used for the regression. Possible values are EYE_LEFT, EYE_RIGHT, EYE_BOTH, PUPIL, EYE_OFFSET and HEAD_ONLY.")
	    ("output,o", po::value<std::string>(&outputFile)->default_value("predicted_gaze.txt"),
	     "Output file that is used to store the predicted gaze positions.")
	    ("train-file", po::value<std::string>(&trainFile), "Input file that contains the training data.")
	    ("test-file", po::value<std::string>(&testFile), "Input file that contains the test data.")
	    ;
	    
	po::options_description cmdline_options;
        cmdline_options.add(generic).add(config);

        po::options_description config_file_options;
        config_file_options.add(config);

        po::options_description visible("Allowed options");
        visible.add(generic).add(config);
	
	// Parse the arguments.
	po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(cmdline_options).run(), vm);
        po::notify(vm);
	
	// Check whether the user set the help flag.
	if (vm.count("help")) {
	    std::cout << "mugs_evaluator - A evaluation tool for MUGS\n\n"
	              << "With this program, one can perform gaze prediction using MUGS for a given\n"
		      << "training and test dataset. Both datasets has to be stored in a mugs file format.\n\n";
	    std::cout << visible << std::endl;
	    return 0;
	}
	
	// Check whether the user set the version flag.
	if (vm.count("version")) {
	    std::cout << "Evaluation tool for MUGS sample files, version 1.0" << std::endl;
	    return 0;
	}
	
	// Check whether the user specified a configuration file.
	if (vm.count("config")) {
	    std::ifstream ifs(configFile.c_str());
            if (!ifs) {
                std::cerr << "can not open config file: " << configFile << "\n";
                return 1;
            } else {
                po::store(po::parse_config_file(ifs, config_file_options), vm);
                po::notify(vm);
            }
	}
	
	// Check which eye model was specified by the user.
	if ((eyeModel != "EyeModelGp") && (eyeModel != "EyeModelLinear") && (eyeModel != "EyeModelMoore")) {
	    std::cerr << "Wrong argument for -m/--model: " << eyeModel
	              << "\nValid arguments are EyeModelGp, EyeModelLinear and EyeModelMoore." 
	              << std::endl;
            return 1;
	}
	
	// Check which eye was chosed for the regression
	if (modelType == "EYE_LEFT"){
	    mt = EYE_LEFT;
	} else if (modelType == "EYE_RIGHT"){
	    mt = EYE_RIGHT;
	} else if (modelType == "EYE_BOTH"){
	    mt = EYE_BOTH;
	} else if (modelType == "PUPIL"){
	    mt = PUPIL;
	} else if (modelType == "EYE_OFFSET"){
	    mt = EYE_OFFSET;
	} else if (modelType == "HEAD_ONLY"){
	    mt = HEAD_ONLY;
	} else {
	    std::cerr << "Wrong argument for -t/--type: " << modelType
	              << "\nValid arguments are EYE_LEFT, EYE_RIGHT, EYE_BOTH, PUPIL, EYE_OFFSET and HEAD_ONLY."
		      << std::endl;
            return 1;
	}
    }
    catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
	return 1;
    }
    catch (...) {
        std::cerr << "Exception of unknown type!\n";
	return 1;
    }

    std::cout << std::fixed << std::setprecision(2);

    // Create a standard screen model (with known coefficients).
    ScreenModelFlat screen(
            Vector3f(0.09,    0, 1.21),  // x,y,z
            Vector3f(0.03, 0.05, 0.06)); // rx, ry, rz
    
    //
    // Gaze tracker calibration
    //

    // Run screen calibration if screen coefficients are unknown
    if (optimizeScreen) {
        std::vector<std::string> screenCalibrationFiles;
        screenCalibrationFiles.push_back(trainFile);
	// use the eye model specified by the user
	if (eyeModel == "EyeModelGp"){screen.calibrate<EyeModelGp>(screenCalibrationFiles, mt);}
	else if (eyeModel == "EyeModelLinear"){screen.calibrate<EyeModelLinear>(screenCalibrationFiles, mt);}
	else if (eyeModel == "EyeModelMoore"){screen.calibrate<EyeModelMoore>(screenCalibrationFiles, mt);}
    }

    std::cout << "\nScreen coefficients:" << std::endl;
    std::cout << "  Center     : " << screen.getCenter().transpose() << std::endl;
    std::cout << "  Orientation: " << screen.getOrientation().transpose() << std::endl;

    // Load previously recorded calibration data
    Samples trainSet = reduceSampleRate(reduceRate, loadSamples(trainFile));

    // Create a gaze tracking object. 
    // We specify the eye model to be used and
    // pass an initialized screen model
    // The used eye model was chosen by the user
    std::ofstream outStream;
    if (eyeModel == "EyeModelGp"){
        GazeTracker<EyeModelGp> gt(screen, mt);
	// Calibrate gaze tracker using loaded data
        std::cout << "\nCalibrating gaze tracker using " << trainSet.size() << " data samples..." << std::endl;
        gt.calibrate(trainSet);
        std::cout << "\nCalibration completed successfully" << std::endl;

        // 
        // Predict PORs using calibrated gaze tracker
        // 
   
        std::cout << "\nEvaluating POR prediction on training data..." << std::endl;
        evaluateTracker(gt, trainSet);

        std::cout << "\nEvalutating POR prediction on test data..." << std::endl;

        // Load test data
        outStream.open(outputFile.c_str());
        Samples testSet = reduceSampleRate(reduceRate, loadSamples(testFile));

        // Run tracker on test data
        evaluateTracker(gt, testSet, outStream);
    } else if (eyeModel == "EyeModelLinear"){
        GazeTracker<EyeModelLinear> gt(screen, mt);
	// Calibrate gaze tracker using loaded data
        std::cout << "\nCalibrating gaze tracker using " << trainSet.size() << " data samples..." << std::endl;
        gt.calibrate(trainSet);
        std::cout << "\nCalibration completed successfully" << std::endl;

        // 
        // Predict PORs using calibrated gaze tracker
        // 
   
        std::cout << "\nEvaluating POR prediction on training data..." << std::endl;
        evaluateTracker(gt, trainSet);

        std::cout << "\nEvalutating POR prediction on test data..." << std::endl;

        // Load test data
        outStream.open(outputFile.c_str());
        Samples testSet = reduceSampleRate(reduceRate, loadSamples(testFile));

        // Run tracker on test data
        evaluateTracker(gt, testSet, outStream);
    } else if (eyeModel == "EyeModelMoore"){
        GazeTracker<EyeModelMoore> gt(screen, mt);
	// Calibrate gaze tracker using loaded data
        std::cout << "\nCalibrating gaze tracker using " << trainSet.size() << " data samples..." << std::endl;
        gt.calibrate(trainSet);
        std::cout << "\nCalibration completed successfully" << std::endl;

        // 
        // Predict PORs using calibrated gaze tracker
        // 
   
        std::cout << "\nEvaluating POR prediction on training data..." << std::endl;
        evaluateTracker(gt, trainSet);

        std::cout << "\nEvalutating POR prediction on test data..." << std::endl;

        // Load test data
        outStream.open(outputFile.c_str());
        Samples testSet = reduceSampleRate(reduceRate, loadSamples(testFile));

        // Run tracker on test data
        evaluateTracker(gt, testSet, outStream);
    }
    outStream.close();
}

