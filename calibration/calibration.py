#!/usr/bin/env python

""" Configuration script for MUGS
This script provides a configuration procedure for MUGS. 
After starting the script it will draw a dot on the display, 
which movements has to be followed by the subject's eyes. Recorded 
data is streamed to LSL and can be used to  train the 
Gaussian process model of MUGS. This is JUST a suggestion. You do not 
have to use this calibration routine to train your model. Of cause, you
can use any calibration routine you want to in order to generate 
training data for MUGS.

Copyright (c) 2013, 2016 Max Planck Institute for Biological Cybernetics
All rights reserved.
 
This file is part of MUGS - Mobile and Unrestrained Gazetracking Software.

MUGS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MUGS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MUGS.  If not, see <http://www.gnu.org/licenses/>.

Author: Jonas Ditz (jonas.ditz@tuebingen.mpg.de)
"""


import sys
import time
import pygame
from pygame.locals import *
import pylsl
import argparse


#--------------------------------------------------
# Constants
#--------------------------------------------------

# Define the sample rate for the LSL outlet.
# Chose according to your monitor's refreshing rate
SAMPLERATE = 60

# Duration of a fixation (in msec).
FIXATIONTIME = 5000

# Movement speed of the dot.
SPEED = 6

# Set up colors.
BLACK = (0, 0, 0)
GRAY = (50, 50, 50)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Size of the moving dot
SIZE = 5

# Thickness of the rings outer line.
# Set it to zero for a filled circle
THICKNESS = 4

# true: The calibration sequence will contain smooth persuit
# false: The Calibration sequence will only consists of fixations
SMOOTHPERSUIT = True


#--------------------------------------------------
# Classes 
#--------------------------------------------------

class MovingDot:
    """ Class to represent the moving dot on the screen.
    """
    def __init__(self, x=0, y=0, dx=0, dy=0, size=SIZE):
        """ Initialize a object of MovingDot.

        Keyword arguments:
        x ---- X coordinate of the dot in px (default 0).
        y ---- Y cooridnate of the dot in px (default 0).
        dx --- Movement of the dot in x direction
               (default 0)
        dy --- Movement of the dot in y direction
               (default 0)
        size - Size of the dot in px (default SIZE)
        """
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.calcDirection()
        self.size = size

    def calcDirection(self):
        """ Calculates moving direction of the 
            dot.
        """
        if self.dx > 0 and self.dy == 0:
            self.direction = 0
        elif self.dx == 0 and self.dy > 0:
            self.direction = 1
        elif self.dx < 0 and self.dy == 0:
            self.direction = 2
        elif self.dx == 0 and self.dy < 0:
            self.direction = 3

    def move(self):
        """ Move the dot for one timestep
        """
        self.x += self.dx
        self.y += self.dy

    def reachedGoal(self, goal):
        """ Returns True if the dot reached its goal.

        Keyword arguments:
        goal - Array of size two that stores the final destination 
               of this moving dot
        """
        switcher = {
            0: lambda: self.x >= goal[0],
            1: lambda: self.y >= goal[1],
            2: lambda: self.x <= goal[0],
            3: lambda: self.y <= goal[1],
        }

        # get the function from the switcher
        case = switcher.get(self.direction, lambda: False)
        return case()



#--------------------------------------------------
# Checker for argparse
#--------------------------------------------------

def check_samplerate(rate):
    try:
        irate = int(rate)
        if irate <= 0:
            raise argparse.ArgumentTypeError("Samplerate needs to be a positive integer (%s was given)" % rate)
        return irate
    except ValueError as e:
        raise argparse.ArgumentTypeError("Samplerate needs to be a positive integer (%s was given)" % rate)


def check_speed(speed):
    try:
        ispeed = int(speed)
        if ispeed <= 0:
            raise argparse.ArgumentTypeError("Speed needs to be a positive integer (%s was given)" % speed)
        return ispeed
    except ValueError as e:
        raise argparse.ArgumentTypeError("Speed needs to be a positive integer (%s was given)" % speed)


def check_fixation(fixation):
    try:
        ifixation = int(fixation)
        if ifixation <= 0:
            raise argparse.ArgumentTypeError("Fixation duration needs to be a positive integer (%s was given)" % fixation)
        return ifixation
    except ValueError as e:
        raise argparse.ArgumentTypeError("Fixation duration needs to be a positive integer (%s was given)" % fixaton)



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
    gridWidth = width/5
    gridHeight = height/4
    widthCor = gridWidth/2
    heightCor = gridHeight/2

    # A two-dimensional list of the form [[(destination x, destination y), dx, dy]].
    # In order to generate fixation targets set dx=dy=0.
    configSeq = []

    if SMOOTHPERSUIT:
        # start smooth persuit part of the calibration sequence
        configSeq.append([(width-50, startY), SPEED, 0])
        configSeq.append([(width-50, height/4), 0, -SPEED])
        configSeq.append([(5*(width/8), height/4), -SPEED, 0])
        configSeq.append([(5*(width/8), height/8), 0, -SPEED])
        configSeq.append([(width/2, height/8), -SPEED, 0])
        configSeq.append([(width/2, height/4), 0, SPEED])
        configSeq.append([(width/4, height/4), -SPEED, 0])
        configSeq.append([(width/4, 3*(height/4)), 0, SPEED])
        configSeq.append([(3*(width/4), 3*(height/4)), SPEED, 0])
        configSeq.append([(3*(width/4), 5*(height/8)), 0, -SPEED])
        configSeq.append([(5*(width/8), 5*(height/8)), -SPEED, 0])
        configSeq.append([(5*(width/8), 4*(height/8)), 0, -SPEED])
        configSeq.append([(4*(width/8), 4*(height/8)), -SPEED, 0])
        configSeq.append([(4*(width/8), 3*(height/8)), 0, -SPEED])
        configSeq.append([(3*(width/8), 3*(height/8)), -SPEED, 0])
        configSeq.append([(3*(width/8), 2*(height/8)), 0, -SPEED])
        configSeq.append([(2*(width/8), 2*(height/8)), -SPEED, 0])
        configSeq.append([(2*(width/8), height/8), 0, -SPEED])
        configSeq.append([(width/8, height/8), -SPEED, 0])
        configSeq.append([(width/8, 11*(height/12)), 0, SPEED])
        configSeq.append([(11*(width/12), 11*(height/12)), SPEED, 0])
        configSeq.append([(11*(width/12), height/12), 0, -SPEED])
        configSeq.append([(width/12, height/12), -SPEED, 0])
        configSeq.append([(width/12, 11*(height/12)), 0, SPEED])
        configSeq.append([(3*(width/12), 11*(height/12)), SPEED, 0])
        configSeq.append([(3*(width/12), 9*(height/12)), 0, -SPEED])
        configSeq.append([(5*(width/12), 9*(height/12)), SPEED, 0])
        configSeq.append([(5*(width/12), 7*(height/12)), 0, -SPEED])
        configSeq.append([(7*(width/12), 7*(height/12)), SPEED, 0])
        configSeq.append([(7*(width/12), 5*(height/12)), 0, -SPEED])
        configSeq.append([(9*(width/12), 5*(height/12)), SPEED, 0])
        configSeq.append([(9*(width/12), 3*(height/12)), 0, -SPEED])
        configSeq.append([(11*(width/12), 3*(height/12)), SPEED, 0])
        configSeq.append([(11*(width/12), height/12), 0, -SPEED])

    # start fixation part of the calibration sequence
    configSeq.append([(gridWidth-widthCor, gridHeight-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])

    configSeq.append([((5*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([(gridWidth-widthCor, gridHeight-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((5*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
    configSeq.append([((3*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    configSeq.append([((2*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
    configSeq.append([((4*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
    
    # If the calibration sequences consists only of fixations, we need more fixations in
    # the sequence
    if not SMOOTHPERSUIT:
        configSeq.append([((2*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
        configSeq.append([(gridWidth-widthCor, gridHeight-heightCor), 0, 0])
        configSeq.append([((5*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
        configSeq.append([((4*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
        configSeq.append([((3*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
        configSeq.append([((2*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
        configSeq.append([((gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
        configSeq.append([((2*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
        configSeq.append([((3*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
        configSeq.append([((4*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
        configSeq.append([((5*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
        configSeq.append([((3*gridWidth)-widthCor, (4*gridHeight)-heightCor), 0, 0])
        configSeq.append([((2*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
        configSeq.append([((3*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
        configSeq.append([((4*gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
        configSeq.append([((5*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])
        configSeq.append([((gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
        configSeq.append([((gridWidth)-widthCor, (2*gridHeight)-heightCor), 0, 0])
        configSeq.append([((5*gridWidth)-widthCor, (3*gridHeight)-heightCor), 0, 0])
        configSeq.append([((4*gridWidth)-widthCor, (gridHeight)-heightCor), 0, 0])

    # return calibration sequence
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
    # Create a Clock object.
    clock = pygame.time.Clock()

    # Display dot at start position until 
    # subject press a button.
    pygameWindow.fill(GRAY)
    pygame.draw.circle(pygameWindow, WHITE, (dot.x, dot.y), dot.size, THICKNESS)
    pygame.display.update()
    eventOcc = False
    while True:
        if eventOcc:
            break
        for event in pygame.event.get():
            if event.type == KEYDOWN and event.key == K_RETURN:
                eventOcc = True

    # Start the calibration sequence.
    for part in sequence:
        # Check whether you have fixation or smooth persuit.
        if (part[1]==0 and part[2]==0): # fixation
            dot.x = part[0][0]
            dot.y = part[0][1]
            dot.dx = 0
            dot.dy = 0
            pygameWindow.fill(GRAY)
            pygame.draw.circle(pygameWindow, WHITE, (dot.x, dot.y), dot.size, THICKNESS)
            pygame.display.update()
            startTime = currentTime()
            while (currentTime() < startTime+FIXATIONTIME):
                pygameWindow.fill(GRAY)
                pygame.draw.circle(pygameWindow, WHITE, (dot.x, dot.y), dot.size, THICKNESS)
                clock.tick(SAMPLERATE)
                pygame.display.update()
                lslStream.push_sample([dot.x, dot.y], pylsl.local_clock())
        else: # smooth persuit
            pygameWindow.fill(GRAY)
            pygame.draw.circle(pygameWindow, WHITE, (dot.x, dot.y), dot.size, THICKNESS)
            clock.tick(SAMPLERATE)
            pygame.display.update()
            lslStream.push_sample([dot.x, dot.y], pylsl.local_clock())
            dot.dx = part[1]
            dot.dy = part[2]
            dot.calcDirection()
            while True:
                dot.move()
                pygameWindow.fill(GRAY)
                pygame.draw.circle(pygameWindow, WHITE, (dot.x, dot.y), dot.size, THICKNESS)
                clock.tick(SAMPLERATE)
                pygame.display.update()
                lslStream.push_sample([dot.x, dot.y], pylsl.local_clock())
                if dot.reachedGoal(part[0]):
                    break
 
    # Indicate that the calibration sequence is over.
    afterTime = currentTime()
    while (currentTime() < afterTime+500):
        lslStream.push_sample([-100, -100], pylsl.local_clock())


def configure():
    """ This function generates the visual experiment, which can be
        used to collect configuration data for MUGS.
    """
    # Set up pygame.
    pygame.init()

    # Get the current width and height of the screen.
    infoObject = pygame.display.Info()
    width = infoObject.current_w
    height = infoObject.current_h

    # Create a pygame window.
    window = pygame.display.set_mode((width, height), pygame.FULLSCREEN, 32)

    # Hide the mouse.
    pygame.mouse.set_visible(False)

    # Write a message to the window.
    window.fill(WHITE)
    font = pygame.font.SysFont("monospace", height/20)
    font_w, font_h = font.size("Waiting for consumers...")
    label = font.render("Waiting for consumers...", 1, BLACK)
    window.blit(label, (width/2-(font_w/2), height/2))
    pygame.display.update()

    # Create MovingDot object located at the center of the screen.
    dot = MovingDot(width/2, height/2, 0, 0, SIZE)

    # Create the path sequence for the dot.
    configSeq = createSeq(dot.x, dot.y, width, height)

    # Create LSL StreamOutlet object.
    lslStream = createLSLstream()

    # Send data only if consumers are presented.
    if (lslStream.wait_for_consumers(50)):
        drawAndRecordConfigSeq(dot, lslStream, configSeq, window)
    else:
        window.fill(WHITE)
        font_w, font_h = font.size("Did not find any consumer!")
        label = font.render("Did not find any consumer!", 1, BLACK)
        window.blit(label, (width/2-(font_w/2), height/2))
        pygame.display.update()
        time.sleep(1)

    # Quits the program
    pygame.quit()
    sys.exit()



#--------------------------------------------------
# Main
#--------------------------------------------------

if __name__ == '__main__':
    # Parse command line arguments.
    parser = argparse.ArgumentParser(description=
                        "Filter points of predicted gaze data.")
    parser.add_argument("-p", "--smoothpersuit", action="store_true",
                        help="If this flag is set, the calibration sequence contains smooth persuits.")
    parser.add_argument("-r", "--samplerate", type=check_samplerate, default=60,
                        help="Sets the frequency that is used by this script during generating the stimulus (in Hz).")
    parser.add_argument("-s", "--speed", type=check_speed, default=6,
                        help="Sets the movement speed of the stimulus during smooth persuit (in Pixel per frame).")
    parser.add_argument("-f", "--fixationtime", type=check_fixation, default=5000,
                        help="Sets the duration of a single fixation (in msec).")
    args = parser.parse_args()

    # Overwrite global variables by the user input.
    SMOOTHPERSUIT = args.smoothpersuit
    SAMPLERATE = args.samplerate
    SPEED = args.speed
    FIXATIONTIME = args.fixationtime
    
    configure()
