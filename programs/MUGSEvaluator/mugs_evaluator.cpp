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
#include <mug/eye_model_linear.h>
#include <mug/screen_model_flat.h>
#include <mug/eye_model_linear.h>
#include <mug/eye_model_moore.h>
#include <mug/eye_model_gp.h>

#include "util.h"

using namespace mug;

// A helper function to simplify the main part.
template<class T>
ostream& operator<<(ostream& os, const vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " ")); 
    return os;
}

double evaluateTracker( const GazeTracker<EyeModelSubject> &gt, Samples samples)
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

double evaluateTracker( const GazeTracker<EyeModelSubject> &gt, Samples samples, std::ofstream &file)
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
    std::string eyeModel, trainFile, testFile, outputFile, configFile;
    try {
        // Declare a group of options that will be allowed
        // only on command line.
	po::options_description generic("Generic options");
	generic.add_options()
	    ("help,h", "Show this help message.")
	    ("version,v", "Print version string.")
	    ("config,c", po::value<string>(&configFile), "Name of a file of a configuration.")
	    ;
	
	// Declare a group of options that will be allowed 
	// both on command line and in a config file.
	po::option_description config("Configuration");
	config.add_options()
	    ("optimize,O", po::value<bool>(&optimizeScreen)->default_value(false), 
	     "Specify, whether or not to opimize the screen (Default: false)")
	    ("model,m", po::value<std::string>(&eyeModel)->default_value("EyeModelGp"),
	     "Specify the eye model used for the gaze tracker (Default: EyeModelGp). Possible values are EyeModelGp, EyeModelLinear and EyeModelMoore.")
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
		      << "training and test dataset. Both datasets has be stored in a mugs file format.\n\n";
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
	if (eyeModel == "EyeModelGp") {
	    typedef mug::EyeModelGp EyeModelSubject;
            typedef mug::EyeModelGp EyeModelScreen;
	} else if (eyeModel == "EyeModelLinear") {
	    typedef mug::EyeModelLinear EyeModelSubject;
            typedef mug::EyeModelLinear EyeModelScreen;
	} else if (eyeModel == "EyeModelMoore") {
	    typedef mug::EyeModelMoore EyeModelSubject;
            typedef mug::EyeModelMoore EyeModelScreen;
	} else {
	    std::cerr << "Wrong argument for -m/--model: " << eyeModel
	              << "\nValid arguments are EyeModelGp, EyeModelLinear and EyeModelMoore." 
	              << std::endl;
            return 1;
	}
    }
    catch (exception& e) {
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
    if (optimizeScreen)
    {
        std::vector<std::string> screenCalibrationFiles;
        screenCalibrationFiles.push_back(trainFile);
        screen.calibrate<EyeModelScreen>(screenCalibrationFiles);
    }

    std::cout << "\nScreen coefficients:" << std::endl;
    std::cout << "  Center     : " << screen.getCenter().transpose() << std::endl;
    std::cout << "  Orientation: " << screen.getOrientation().transpose() << std::endl;

    // Load previously recorded calibration data
    mug::Samples trainSet = reduceSampleRate(4, loadSamples(trainFile));

    // Create a gaze tracking object. 
    // We specify the eye model to be used and
    // pass an initialized screen model
    mug::GazeTracker<EyeModelSubject> gt(screen);

    // Calibrate gaze tracker using loaded data
    std::cout << "\nCalibrating gaze tracker using " << trainSet.size() << " data samples..." << std::endl;
    gt.calibrate(trainSet);
    std::cout << "\nCalibration completed successfully" << std::endl;

    // 
    // Predict PORs using calibrated gaze tracker
    // 
   
    std::cout << "\nEvaluating POR prediction on training data..." << std::endl;
    evaluateTracker(gt, trainSet, trainGaze);

    std::cout << "\nEvalutating PORs prediction on test data..." << std::endl;

    // Load test data
    std::ofstream outStream;
    outStream.open(outputFile.c_str());
    mug::Samples testSet = reduceSampleRate(4, loadSamples(testFile));

    // Run tracker on test data
    evaluateTracker(gt, testSet, outStream);
    outStream.close();
}

