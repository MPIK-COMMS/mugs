#!/usr/bin/env python

""" Configuration script for MUGS
This script provides the configuration procedure for MUGS. 
After starting the script it will draw a dot on the display, 
which movements has to be followed by the subject. Recorded 
data is streamed to LSL and can be used to  train the 
Gaussian process model of MUGS.

Software License Agreement (BSD License) 

MUG - Mobile and Unrestrained Gazetracking
Copyright (c) 2016, Max Planck Institute for Biological Cybernetics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its 
      contributors may be used to endorse or promote products derived from 
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: Jonas Ditz (jonas.ditz@tuebingen.mpg.de)
"""


import sys
import time
import pygame
from pygame.locals import *
import pylsl


#--------------------------------------------------
# Constants
#--------------------------------------------------

SAMPLERATE = 100
FIXATIONTIME = 1000

# Set up colors.
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


#--------------------------------------------------
# Classes 
#--------------------------------------------------

class MovingDot:
    """ Class to represent the moving dot on the screen.
    """
    def __init__(self, x=0, y=0, dx=0, dy=0, size=10):
        """ Initialize a object of MovingDot.

        Keyword arguments:
        x ---- X coordinate of the dot in px (default 0).
        y ---- Y cooridnate of the dot in px (default 0).
        dx --- Movement of the dot in x direction
               (default 0)
        dy --- Movement of the dot in y direction
               (default 0)
        size - Size of the dot in px (default 10)
        """
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.size = size

    def move(self):
        """ Move the dot for one timestep
        """
        self.x += self.dx
        self.y += self.dy


#--------------------------------------------------
# Functions
#--------------------------------------------------

currentTime = lambda: int(round(time.time() * 1000))

def createLSLstream():
    """ Function to create a LSL StreamOutlet

    Returns the created StreamOutlet object.
    """
    # Define variables for the StreamInfo object.
    sName = "ConfigurationDotPositions"
    sType = "TXT"
    sChannelCount = 2
    sChannelFormat = pylsl.cf_int16
    sSourceId = "MUGS_Configuration_Script" 
     
    # Create the LSL StreamInfo object.
    info = pylsl.StreamInfo(sName, sType, sChannelCount, 
                      SAMPLERATE, sChannelFormat, sSourceId)

    # Use the StreamInfo object to create a StreamOutlet.
    streamOutlet = pylsl.StreamOutlet(info)
    return streamOutlet

def createSeq(startX, startY, width, height):
    """ Create a tour for the dot given a certain starting
    point and the width and height of the window.

    Keyword arguments:
    startX - Starting x coordinate of the dot.
    startY - Starting y coordinate of the dot.
    width -- Width of the current window.
    height - Height of the current window.
    """
    configSeq = []
    configSeq.append([(width-50, startY), 1, 0])
    configSeq.append([(width-50, height/4), 0, -1])
    configSeq.append([(width/4, height/4), -1, 0])
    configSeq.append([(width/4, 3*(height/4)), 0, 1])
    configSeq.append([(3*(width/4), 3*(height/4)), 1, 0])
    configSeq.append([(width/2, height/2), 0, 0])
    configSeq.append([(width/4, height/2), 0, 0])
    configSeq.append([(50, 50), 0, 0])
    configSeq.append([(width-50, height-50), 0, 0])
    configSeq.append([(width-50, 50), 0, 0])
    configSeq.append([(50, height-50), 0, 0])
    return configSeq

def drawAndRecordConfigSeq(dot, lslStream, sequence, pygameWindow):
    """ Draws the configuration sequence on the screen and 
        sends dot positions to LSL.

    Keywords arguments:
    dot ---------- MovingDot object, which is used to store
                   information about the current position of 
                   the dot.
    lslStream ---- LSL StreamOutlet object, which is used to 
                   stream positions of the dot to LSL.
    sequence ----- Array of tuples, which stores the tour
                   of the dot.
    pygameWindow - Pygame window object to visualize the 
                   Experiment.
    """
    for part in sequence:
        # check if you have fixation or smooth persued
        if (part[1]==0 and part[2]==0):
            dot.x = part[0][0]
            dot.y = part[0][1]
            dot.dx = 0
            dot.dy = 0
            startTime = currentTime()
            while (currentTime() < startTime+FIXATIONTIME):
                pygameWindow.fill(WHITE)
                pygame.draw.circle(pygameWindow, BLACK, (dot.x, dot.y), dot.size, 0)
                pygame.display.update()
                lslStream.push_sample([dot.x, dot.y], pylsl.local_clock())
        else:
            pygameWindow.fill(WHITE)
            pygame.draw.circle(pygameWindow, BLACK, (dot.x, dot.y), dot.size, 0)
            pygame.display.update()
            lslStream.push_sample([dot.x, dot.y], pylsl.local_clock())
            dot.dx = part[1]
            dot.dy = part[2]
            while True:
                dot.move()
                pygameWindow.fill(WHITE)
                pygame.draw.circle(pygameWindow, BLACK, (dot.x, dot.y), dot.size, 0)
                pygame.display.update()
                lslStream.push_sample([dot.x, dot.y], pylsl.local_clock())
                if (dot.x==part[0][0] and dot.y==part[0][1]):
                    break

def configure():
    """ This function generates the visual experiment, which can be
        used to collect configuration data for MUGS.
    """
    # Set up pygame
    pygame.init()

    # Get the current width and height of the screen.
    infoObject = pygame.display.Info()
    width = infoObject.current_w - 50
    height = infoObject.current_h - 50

    # Create a pygame window.
    window = pygame.display.set_mode((width, height), 0, 32)

    # Create MovingDot object located at the center of the screen.
    dot = MovingDot(width/2, height/2, 0, 0, 20)

    # Create the path sequence for the dot.
    configSeq = createSeq(dot.x, dot.y, width, height)

    # Create LSL StreamOutlet object
    lslStream = createLSLstream()

    drawAndRecordConfigSeq(dot, lslStream, configSeq, window)


if __name__ == '__main__':
    configure()
