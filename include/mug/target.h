/* 
 * Copyright (c) 2013, 2017 Max Planck Institute for Biological Cybernetics
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

#ifndef __TARGET_H__
#define __TARGET_H__

#include <Eigen/Dense>
#include <boost/random.hpp>
#include <queue>
#include "SDL/SDL.h"

class Target
{
public:
    Target(int screenWidth, 
           int screenHeight, 
           int speed = 100, // [px per sec]
           int margin_ = 20)
        : screenW(screenWidth), screenH(screenHeight),
       margin(margin_), minDistToCurrLoc(100),
       randomX(margin_, screenWidth-margin), 
       randomY(margin_, screenHeight-margin)
    {
        rng.seed(time(0));

        // start at screen center
        p = Eigen::Vector2f(screenWidth/2.0, screenHeight/2.0);

        // create first random fixation point
        setNextPoint();

        last_time = SDL_GetTicks();
        time_waitOnPointUntil = 0;
        waitOnPointMS = 750;

        v  = speed;// px per sec
    }
    inline void update()
    {
        Uint32 dt = SDL_GetTicks() - last_time;
        last_time = SDL_GetTicks(); 

        const double tol = 2;
        if (abs(p[0] - waypoints.front()[0]) < tol &&
            abs(p[1] - waypoints.front()[1]) < tol)
        {
            time_waitOnPointUntil = SDL_GetTicks() + waitOnPointMS;
            waypoints.pop();
            if (waypoints.empty())
            {
                setNextPoint();
            }
        }

        if (time_waitOnPointUntil > SDL_GetTicks())
        {
            return;
        }

        double ds = dt/1000.0 * v;
        Eigen::Vector2f vec =  waypoints.front() - p;
        normalize(vec);
        p += vec * ds;
    }

    inline int x() const { return p[0]; }
    inline int y() const { return p[1]; }

private:
    inline Eigen::Vector2f setNextPoint() 
    {
        int rx, ry;
        do
        {
            rx = randomX(rng);
            ry = randomY(rng);
        }
        while (abs(rx-p[0]) < minDistToCurrLoc || abs(ry-p[1]) < minDistToCurrLoc);
        Eigen::Vector2f end(rx, ry) ;
        Eigen::Vector2f mid(p[0], end[1]);
        //std::cout << "mid " << randomX.min() << " " << mid.transpose() << std::endl;
        //std::cout << "end " << randomY.min() << " " << end.transpose() << std::endl << std::endl;
        waypoints.push(mid);
        waypoints.push(end);
    }

    inline void normalize(Eigen::Vector2f &x) const
    {
        x = x / sqrt(x.array().square().sum());
    }

    double dist(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
    {
        return sqrt((p1-p2).array().square().sum());
    }
    

    int margin;
    int minDistToCurrLoc;
    Eigen::Vector2f p;
    int screenW;
    int screenH;
    double v;
    std::queue<Eigen::Vector2f> waypoints;
    Uint32 last_time;
    Uint32 time_waitOnPointUntil;
    Uint32 waitOnPointMS;

    boost::mt19937 rng; 
    boost::random::uniform_int_distribution<> randomX;
    boost::random::uniform_int_distribution<> randomY;
};

#endif


