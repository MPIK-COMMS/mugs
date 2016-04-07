#!/usr/bin/env python

"""
"""

import sys
import pylsl

# ----------------------------------------
# Constants
# ----------------------------------------


# ----------------------------------------
# Classes
# ----------------------------------------


# ----------------------------------------
# Functions
# ----------------------------------------

def fetchDotPositions(lslInlet):
    """
    """
    return False

def fetchHeadData(lslInlet):
    """
    """
    return False

def fetchEyeData(lslInlet):
    """
    """
    return False


if __name__ == '__main__':
    print 'resolving streams...'
    results = pylsl.resolve_byprop('name', 'ConfigurationDotPositions')
    #result = pylsl.resolve_byprop('type', 'TXT', 1, 20)
    print 'done'

    if (len(results) == 0):
        print 'Could not find any stream with the spezified properties'
        sys.exit()

    inlet = pylsl.StreamInlet(results[0])

    sample, timestamp = inlet.pull_sample()
    while (sample != None and timestamp != None):
        print timestamp,sample
        sample, timestamp = inlet.pull_sample()
