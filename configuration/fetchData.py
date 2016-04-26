#!/usr/bin/env python

""" Data fetching script for MUGS
This script provides a routine to fetch all LSL streams 
that provide information needed for the configuration of MUGS.
Fetched data is stored into specific files: dot_data.mugs, 
eye_data.mugs and head_data.mugs. One can use these files to 
create the calibration data file with the script buildCalibFile.py.

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
import pylsl
import time

# ----------------------------------------
# Constants
# ----------------------------------------

# Name of the streams used for calibrating 
# MUGS
DOT_STREAM_NAME = 'ConfigurationDotPositions'
EYE_STREAM_NAME = 'MyEyetrackerStream'
HEAD_STREAM_NAME = 'MyHeadtrackerStream'

# Name of the files where the raw calibration 
# data are stored
DOT_FILE_NAME = 'dot_data.mugs'
EYE_FILE_NAME = 'eye_data.mugs'
HEAD_FILE_NAME = 'head_data.mugs'

# ----------------------------------------
# Classes
# ----------------------------------------


# ----------------------------------------
# Functions
# ----------------------------------------

def fetchLSLstream(name):
    """ Try to fetch a LSL stream with a specific name.

        Keyword arguments:
        name - Value of the 'name' property of the stream.
    """
    results = pylsl.resolve_byprop('name', name)
    
    if (len(results) == 0):
        print 'Could not find any stream with the specified name: ',name
        sys.exit()

    lslInlet = pylsl.StreamInlet(results[0])
    return lslInlet

def dataToFile(data):
    """ Writes collected data to a file called 
        configuration.mugs
    """
    currentTime = int(round(time.time() * 1000))
    filename = 'calibData_'+str(currentTime)+'.mugs'
    f = open(filename, 'w')
    for line in data:
        s = (str(line[0])+' '+str(line[1])+' '+str(line[2])+' '+
             str(line[3])+' '+str(line[4])+' '+str(line[5])+' '+
             str(line[6])+' '+str(line[7])+' '+str(line[8])+' '+
             str(line[9])+' '+str(line[10])+' 0 0 0 0 '+
             str(line[11])+' '+str(line[12]))
        f.write(s+'\n')
    print 'Data written to file '+filename
    f.close()


if __name__ == '__main__':
    # Fetch the stream with eye tracker data.
    print 'fetching Eye tracker stream...'
    #eyeInlet = fetchLSLstream(EYE_STREAM_NAME)
    print 'DONE'

    # Fetch the stream with head tracker data.
    print 'fetching Head tracker stream...'
    #headInlet = fetchLSLstream(HEAD_STREAM_NAME)
    print 'DONE'
    
    # Fetch the stream with dot positions.
    print 'fetching dot position stream...'
    dotInlet = fetchLSLstream(DOT_STREAM_NAME)
    print 'DONE'

    # open storage files
    #dotFile = open(DOT_FILE_NAME, 'w')
    #eyeFile = open(EYE_FILE_NAME, 'w')
    #headFile = open(HEAD_FILE_NAME, 'w')
    data = []
    while (True):
        dotSample, dotTimestamp = dotInlet.pull_sample()
        #eyeSample, eyeTimestamp = eyeInlet.pull_sample()
        #headSample, headTimestamp = headInlet.pull_sample()
        eyeSample = [0,0,0,0]
        eyeTimestamp = dotTimestamp
        headSample = [0,0,0,0,0,0]
        headTimestamp = dotTimestamp

        # Check whether or not the calibration sequence is over
        if(dotSample[0] == -100):
            dotInlet.close_stream()
            #eyeInlet.close_stream()
            #headInlet.close_stream()
            break

        if (dotTimestamp == eyeTimestamp == headTimestamp):
            data.append([dotTimestamp, headSample[0], headSample[1], headSample[2], 
                         headSample[3], headSample[4], headSample[5], eyeSample[0], 
                         eyeSample[1], eyeSample[2], eyeSample[3], dotSample[0], dotSample[1]])
    dataToFile(data)
