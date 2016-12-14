"""
  Defines the function convert_xdf_to_mugs, which converts multistream xdf files into mugs files.
  These mugs files can be read by MUGSEvaluator and MUGSFilter.
"""

import xdf
import argparse
import sys
import numpy as np
import scipy.spatial


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
  return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def writeFile(streams, indices_stream, indices_head, indices_eye, filename):
  # first case: stimulus was presented
  if indices_stream[2] != -1:
    f = open(filename, 'w')
    for i in range(0, len(streams[indices_stream[2]]["time_stamps"])):
      write_str = "%f %f %f %f %f %f %f %f %f %f %f 0 0 0 0 %i %i\n" % (streams[indices_stream[2]]["time_stamps"][i],
           streams[indices_stream[0]]["time_series"][indices_head[i]][0], streams[indices_stream[0]]["time_series"][indices_head[i]][1], streams[indices_stream[0]]["time_series"][indices_head[i]][2],
           streams[indices_stream[0]]["time_series"][indices_head[i]][3], streams[indices_stream[0]]["time_series"][indices_head[i]][4], streams[indices_stream[0]]["time_series"][indices_head[i]][5],  
           streams[indices_stream[1]]["time_series"][indices_eye[i]][0], streams[indices_stream[1]]["time_series"][indices_eye[i]][1], 
           streams[indices_stream[1]]["time_series"][indices_eye[i]][2], streams[indices_stream[1]]["time_series"][indices_eye[i]][3], 
           streams[indices_stream[2]]["time_series"][i][0], streams[indices_stream[2]]["time_series"][i][1])
      f.write(write_str)

  # second case: no stimulus was presented
  else:
    f = open(filename, 'w')
    for i in range(0, len(streams[indices_stream[2]]["time_stamps"])):
      write_str = "%f %f %f %f %f %f %f %f %f %f %f 0 0 0 0 -1 -1\n" % (streams[indices_stream[2]]["time_stamps"][i],
           streams[indices_stream[0]]["time_series"][indices_head[i]][0], streams[indices_stream[0]]["time_series"][indices_head[i]][1], streams[indices_stream[0]]["time_series"][indices_head[i]][2],
           streams[indices_stream[0]]["time_series"][indices_head[i]][3], streams[indices_stream[0]]["time_series"][indices_head[i]][4], streams[indices_stream[0]]["time_series"][indices_head[i]][5],  
           streams[indices_stream[1]]["time_series"][indices_eye[i]][0], streams[indices_stream[1]]["time_series"][indices_eye[i]][1], 
           streams[indices_stream[1]]["time_series"][indices_eye[i]][2], streams[indices_stream[1]]["time_series"][indices_eye[i]][3])
      f.write(write_str)


def convert_xdf_to_mugs(inFilename, outFilename, verbose):
  streams, headers = xdf.load_xdf(inFilename, verbose=verbose)

  # check which stream stores which data
  headStream = -1
  eyeStream = -1
  stimStream = -1
  i = 0
  for stream in streams:
    if stream["info"]["channel_count"][0] == '2':
      stimStream = i
      i = i+1
      continue
    elif stream["info"]["channel_count"][0] == '4':
      eyeStream = i
      i = i+1
      continue
    elif stream["info"]["channel_count"][0] == '6':
      headStream = i
      i = i+1
      continue
    else:
      print "Unexpected number of channels in stream", stream["info"]["name"][0], "(", stream["info"]["channel_count"][0], "channels detected )"
      sys.exit()

  if verbose:
    print "Start linking sample points..."
  # check whether a stimulus was presented
  if stimStream != -1:
    # sync the stimulus sample points to the head sample points
    stim = np.array(streams[stimStream]["time_stamps"])
    head = np.array(streams[headStream]["time_stamps"])
    
    # a tree optimized for nearest-neighbor lookup
    if verbose:
      print "Building KDTree for head and stimulus..."
    tree = scipy.spatial.cKDTree(head[..., np.newaxis])

    # get the distances and indices of all head sample points to their nearest neighbor 
    # stimulus sample point
    distances_head, indices_head = tree.query(stim[..., np.newaxis])

    # now do the same with the eye sample points
    eye = np.array(streams[eyeStream]["time_stamps"])
    if verbose:
      print "Building KDTree for eye and stimulus..."
    tree = scipy.spatial.cKDTree(eye[..., np.newaxis])
    distances_eye, indices_eye = tree.query(stim[..., np.newaxis])
  
    # write synchronized data to mugs file
    writeFile(streams, [headStream, eyeStream, stimStream], indices_head, indices_eye, outFilename)

  else:
    return


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=
                      "Converts a xdf file into a mugs file.")
  parser.add_argument("-v", "--verbose", action="store_true",
                      help="increase output verbosity")
  parser.add_argument("-i", "--input", default="samples.xdf",
                      help="name of the input file")
  parser.add_argument("-o", "--output", default="samples.mugs",
                      help="name of the output file")
  args = parser.parse_args()

  convert_xdf_to_mugs(args.input, args.output, args.verbose)