#include <string.h>
#include <iostream>
#include <stdio.h>

#include <mug/gaze_tracker.h>
#include <mug/eye_model_linear.h>
#include <mug/screen_model_flat.h>
#include <mug/eye_model_linear.h>
#include <mug/eye_model_moore.h>
#include <mug/eye_model_gp.h>

#include "util.h"

using namespace mug;

//typedef mug::EyeModelLinear EyeModelSubject;
//typedef mug::EyeModelLinear EyeModelScreen;
typedef mug::EyeModelGp EyeModelSubject;
typedef mug::EyeModelGp EyeModelScreen;
//typedef mug::EyeModelMoore EyeModelSubject;
//typedef mug::EyeModelMoore EyeModelScreen;

std::string dataDir    = "./data/";


double evaluateTracker( const GazeTracker<EyeModelSubject> &gt, Samples samples, std::ofstream &file)
{
    double merr_u = 0; 
    double merr_v = 0;
    double mse    = 0;
    
    for (std::vector<Sample>::iterator it = samples.begin(); it != samples.end(); it++)
    {
        Sample &s = *it;

        Vector3f uv = gt.getScreenUV(s.H_pos, s.H_o, s);

        double eu = fabs(uv[0] - s.target_pos[0]);
        double ev = fabs(uv[1] - s.target_pos[1]);

        merr_u += eu;
        merr_v += ev;
        mse = eu*eu + ev*ev;

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
    bool optimizeScreen = false;

    std::cout << std::fixed << std::setprecision(2);

    // File containing calibration samples
    std::string dataFile = dataDir + "move_train.txt"; 
    std::string gazeFile = dataDir + "move_trainGaze.txt";
    std::ofstream trainGaze;
    trainGaze.open(gazeFile.c_str());

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
        screenCalibrationFiles.push_back(dataFile);
        screen.calibrate<EyeModelLinear>(screenCalibrationFiles);
    }

    std::cout << "\nScreen coefficients:" << std::endl;
    std::cout << "  Center     : " << screen.getCenter().transpose() << std::endl;
    std::cout << "  Orientation: " << screen.getOrientation().transpose() << std::endl;

    // Load previously recorded calibration data
    mug::Samples trainSet = reduceSampleRate(4, loadSamples(dataFile));

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
    trainGaze.close();
    dataFile = dataDir + "move_test.txt";
    gazeFile = dataDir + "move_testGaze.txt";
    std::ofstream testGaze;
    testGaze.open(gazeFile.c_str());
    mug::Samples testSet = filterSamples(loadSamples(dataFile));

    // Run tracker on test data
    evaluateTracker(gt, testSet, testGaze);
    testGaze.close();
}

