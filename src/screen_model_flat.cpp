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



