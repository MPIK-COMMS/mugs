#!/usr/bin/env python

""" Data fetching script for MUGS
This script provides a routine to fetch all data, 
which are needed for the configuration of MUGS

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

# ----------------------------------------
# Constants
# ----------------------------------------

DOT_STREAM_NAME = 'ConfigurationDotPositions'
EYE_STREAM_NAME = ''
HEAD_STREAM_NAME = ''

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


    # Data is stored an 2-dimentional list with the following layout:
    # [[timestamp, posHead, orientHead, leftEye, rightEye, posTarget]]
    data = []
    while (True):
        dotSample, dotTimestamp = dotInlet.pull_sample()
        #eyeSample, eyeTimestamp = eyeInlet.pull_sample()
        #headSample, headTimestamp = headInlet.pull_sample()
        eyeSample = [0,0,0,0]
        headSample = [0,0,0,0,0,0]

        data.append([dotTimestamp, headSample[0], headSample[1], 
                     headSample[2], headSample[3], headSample[4], 
                     headSample[5], eyeSample[0], eyeSample[1], 
                     eyeSample[2], eyeSample[3], dotSample[0], dotSample[1]])
