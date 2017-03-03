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

#include <string.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <math.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <mug/preprocessing.h>
#include <mug/eye_model.h>
#include <mug/sample.h>

using namespace mug;


// A helper function to simplify the main part.
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " ")); 
    return os;
}

int main(int argc, char ** argv)
{
    // Handle command line arguments.
    std::string configFile, inputFile, outputFile, modelType;
    int samplerate, order;
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
	    ("type,t", po::value<std::string>(&modelType)->default_value("EYE_LEFT"),
	     "Specify the eye, which will be used for filtering. Possible values are EYE_LEFT, EYE_RIGHT, EYE_BOTH, PUPIL, EYE_OFFSET and HEAD_ONLY.")
	    ("samplerate,s", po::value<int>(&samplerate)->default_value(60),
	     "Set this to the samplerate that will be used by the filtering algorithms. Make sure that this matches the samplerate of your recordings.")
	    ("order,d", po::value<int>(&order)->default_value(10),
	     "Size of the local neighborhood during the local extrema search.")
	    ("input,i", po::value<std::string>(&inputFile)->default_value(("samples.mugs")),
	     "Input file, which contains the dataset that should be filtered.")
	    ("output,o", po::value<std::string>(&outputFile)->default_value("filtered_samples.mugs"),
	     "Output file that is used to store the filtered dataset.")
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
	    std::cout << "MUGSFilter - A filtering tool for MUGS\n" 
	              << "Copyright (c) 2017 Max Planck Institute for Biological Cybernetics\n\n"
	              << "With this program, one can perform various filtering methods for a given\n"
		      << "dataset of sample points. The dataset has to be stored in a mugs file.\n\n"
		      << "This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you are\n"
		      << "welcome to redistribute it under certain conditions.\n\n";
	    std::cout << visible << std::endl;
	    return 0;
	}
	
	// Check whether the user set the version flag.
	if (vm.count("version")) {
	    std::cout << "Filtering tool for MUGS sample files, version 1.0" << std::endl;
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
    
    // print out License note
    std::cout << "MUGSFilter Copyright (c) 2017 Max Planck Institute for Biological Cybernetics\n"
              << "This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you are\n"
	      << "welcome to redistribute it under certain conditions.\n" << std::endl;
    
    Samples dataset = loadSamples(inputFile);
    std::cout << "Filtering data using a local neighborhood of " << order << " elements on both sides..." << std::endl;
    std::vector<Eigen::Vector2i> beforeFixOnset = onsetFilter_velocity(dataset.samples, mt, samplerate, order, true);
    std::cout << "  DONE" << std::endl;
    saveSamples(dataset.samples, outputFile);
}