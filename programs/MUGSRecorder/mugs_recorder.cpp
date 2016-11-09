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

#include <string.h>
#include <iostream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <mug/sample.h>
#include <mug/lsl_interface.h>

using namespace mug;

// A helper function to simplify the main part.
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " ")); 
    return os;
}

int main (int argc, char** argv)
{
    // Handle command line arguments.
    int terminal;
    std::string outputFile, headStream, eyeStream, stimStream, configFile;
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
	    ("terminal,t", po::value<int>(&terminal)->default_value(-100), 
	     "Dot position that is send to indicate the end of the stimulus sequence (Default: -100).")
	    ("output,o", po::value<std::string>(&outputFile)->default_value("samples.mugs"),
	     "Output file that is used to store the predicted gaze positions.")
	    ("head-stream,H", po::value<std::string>(&headStream), "Name of the LSL stream that sends the head coordinates.")
	    ("eye-stream,E", po::value<std::string>(&eyeStream), "Name of the LSL stream that sends the eye coordinates.")
	    ("stim-stream,S", po::value<std::string>(&stimStream)->default_value(""), 
	     "Name of the LSL stream that sends the stimulus coordinates.")
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
	    std::cout << "mugs_recorder - A recording tool for MUGS\n\n"
	              << "With this software one can record the head, eye and stimulus movements during\n"
		      << "an experiment. Head, eye and stimulus coordinates have to be sent with LSL.\n"
		      << "The results will be stored in a mugs file that can be immediately used for gaze\n"
		      << "prediction with MUGS.\n\n";
	    std::cout << visible << std::endl;
	    return 0;
	}
	
	// Check whether the user set the version flag.
	if (vm.count("version")) {
	    std::cout << "Recording tool for MUGS, version 1.0" << std::endl;
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
	
	// Abort if either head or eye stream is not set
	if ((vm.count("head-stream") == 0) || (vm.count("eye-stream") == 0)) {
	    std::cerr << "ERROR: Please set head AND eye stream" << std::endl;
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
    
    // create LslInterface object
    LslInterface lslInterface(headStream, eyeStream, stimStream);
    Samples samples;
    std::string filename = "test_data_output";
    
    if (stimStream != ""){lslInterface.fetchData(samples, filename, terminal);}
    else {lslInterface.fetchData(samples, filename);}

    //saveSamples(samples, outputFile);
}