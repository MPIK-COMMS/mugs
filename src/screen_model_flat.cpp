/* 
 * Software License Agreement (BSD License) 
 *
 * MUG - Mobile and Unrestrained Gazetracking
 * Copyright (c) 2013, Max Planck Institute for Biological Cybernetics
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <mug/head_eye_transform.h>
#include <mug/screen_model_flat.h>
#include <mug/eye_model_linear.h>

using namespace mug;
using namespace Eigen;

typedef matrix<double,0,1> column_vector;

void ScreenModelFlat::save(const std::string &filename) const
{
    std::ofstream file;
    file.open(filename.c_str());
    file << center << std::endl;
    file << orientation << std::endl;
    file.close();
}   

void ScreenModelFlat::load(const std::string &filename) 
{
    std::ifstream file;
    file.open(filename.c_str());
    file >> center(0) >> center(1) >> center(2) 
        >> orientation(0) >> orientation(1) >> orientation(2);
    file.close();
    create(center, orientation);
}

// create screen by center and Euler angle orientation
void ScreenModelFlat::create(const Vector3f &c, const Vector3f &o, 
        double w_, double h_, int res_x_, int res_y_)
{
    // set screen dimensions
    width_m  = w_;      // [meters]
    height_m = h_;      // [meters]
    width_px  = res_x_; // [pixels]
    height_px = res_y_; // [pixels]

    // set position and orientation
    center      = c;
    orientation = o;

    // calculate screen corners  
    Matrix3f m;
    m =  
        AngleAxisf(-o[2], Vector3f::UnitZ())
        * AngleAxisf(o[0], Vector3f::UnitX())
        * AngleAxisf(o[1], Vector3f::UnitY());
    
    Vector3f nx = m * -Vector3f::UnitX();
    Vector3f ny = m * -Vector3f::UnitY();
    Vector3f nz = m * Vector3f::UnitZ();

    double a = atan(height_m/width_m); // angle to corners

    Vector3f tl = AngleAxisf(-a, ny) *  nx;
    Vector3f tr = AngleAxisf(+a, ny) * -nx;
    Vector3f br = AngleAxisf(-a, ny) * -nx;
    
    double diagL = sqrt(width_m*width_m + height_m*height_m)/2.0;
    tl *= diagL;
    tr *= diagL;
    br *= diagL;

    topLeft      = tl + c;
    topRight     = tr + c;
    bottomRight  = br + c;
    
    // create 3D plane for projections
    plane = Planef::Through(topLeft, topRight, bottomRight);
}


void ScreenModelFlat::calcGazeAngles(const Vector2f &targetUV, 
        const Vector3f &H_pos, const Vector3f &H_o, 
        const Vector3f &T_trans, const Vector3f &T_rot,
        float &yaw, float &pitch) const
{
    Vector3f target = getPixelCoordXYZ(targetUV);

    Vector3f E_pos, E_o;
    getEyePose(H_pos, H_o, T_trans, T_rot, E_pos, E_o);

    // head rotation matrix (RPY)
    Matrix3f r ; r =  
        AngleAxisf(-E_o[2], Vector3f::UnitZ())
        * AngleAxisf(E_o[0], Vector3f::UnitX())
        * AngleAxisf(E_o[1], Vector3f::UnitY());

    // move head to origin
    target -= E_pos;

    target = normalize(target);
    //std::cout << "targetNorm" << target.transpose() << std::endl;
    
    Vector3f origin(0,0,0);

    // roll 
    Matrix3f m;
    m = AngleAxisf(E_o[1], Vector3f::UnitY());

    // orientation axis of head
    Vector3f headAxis = r * Vector3f::UnitY();

    // transformation planes
    Planef planeYaw    = Planef::Through(origin, headAxis, r*Vector3f::UnitX());
    Planef planePitch  = Planef::Through(origin, headAxis, r*Vector3f::UnitZ());
    Planef planeProjection = Planef::Through(origin, m*Vector3f::UnitY() , r*Vector3f::UnitX());

    // project target on transformed X/Y-Plane
    Vector3f targetProj = planeProjection.projection(target);
    yaw = angle(targetProj, planePitch.normal());

    // rotate target by yaw
    Vector3f rotAxis = m * Vector3f::UnitZ();;
    Vector3f t_rot = AngleAxisf(-yaw, rotAxis) * target;

    // pitch is angle between new target location and head orientation 
    pitch = M_PI/2.0 - angle(t_rot, headAxis);
    if (planeYaw.normal().dot(t_rot) < 0)  
        pitch *= -1;

    yaw *= -1;
}

Vector2f ScreenModelFlat::project(const Vector3f &H_pos, const Vector3f &H_o, 
        const Vector3f &T_trans, const Vector3f &T_rot, 
        float yaw, float pitch) const
{
    Vector3f origin(0,0,0);

    Vector3f E_pos, E_o;
    getEyePose(H_pos, H_o, T_trans, T_rot, E_pos, E_o);

    Matrix3f r ; r =  
        AngleAxisf(-E_o[2], Vector3f::UnitZ())
        * AngleAxisf(E_o[0], Vector3f::UnitX())
        * AngleAxisf(E_o[1], Vector3f::UnitY());
    
    Vector3f tl = topLeft - E_pos;
    Vector3f tr = topRight - E_pos;
    Vector3f br = bottomRight - E_pos;

    Planef screen = Planef::Through(tl, tr, br);

    Vector3f gazeAxis = r * Vector3f::UnitY();

    Matrix3f m;
    m = AngleAxisf(E_o[1], Vector3f::UnitY());

    // head planes
    Planef planeYaw    = Planef::Through(origin, gazeAxis, r*Vector3f::UnitX());
    Planef planePitch  = Planef::Through(origin, gazeAxis, r*Vector3f::UnitZ());

    Vector3f tiltAxis = -planeYaw.normal().cross(gazeAxis);
    Vector3f rotAxis  = m * Vector3f::UnitZ();

    Matrix3f ryaw ;
    Matrix3f rpitch ;

    ryaw   =  AngleAxisf(-yaw, rotAxis);
    rpitch =  AngleAxisf(pitch, tiltAxis); 

    Vector3f t_rot = rpitch * gazeAxis;
    Vector3f targetReverse = ryaw * t_rot;

    Linef gaze = Linef::Through(origin, targetReverse);

    Vector3f fixationXYZ = gaze.intersectionPoint(screen);


    // translate back by eye offset
    fixationXYZ += E_pos;

    Vector2f fixationUV = getPixelCoordUV(fixationXYZ);

    return fixationUV;

}



