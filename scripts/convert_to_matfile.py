from ros import rosbag
import os, sys
import numpy as np
import scipy.io

HOME_PIXELS_REAL = np.array([620,538])
OBSERVATION_THRESHOLD_DIST=5.0

class OriFinder:
    def __init__(self,bag):
        self.by_stamp = {}
        for topic, msg, t in bag.read_messages(topics=['/flytrax_info']):
            self.by_stamp[msg.stamp] = msg.detections
    def find_ori( self, stamp, x, y ):
        detections = self.by_stamp[stamp]
        best_idx = None
        best_dist = np.inf
        for idx,detection in enumerate(detections):
            dist = np.sqrt((detection.x-x)**2 + (detection.y-y)**2)
            if dist<best_dist:
                best_dist = dist
                best_idx = idx

        if best_dist < OBSERVATION_THRESHOLD_DIST:
            theta = np.arctan( detection.slope )
            return theta, detection.eccentricity, detection.area
        else:
            return np.nan, np.nan, np.nan

if 1:
    if len(sys.argv) != 4:
        print >> sys.stderr, """Invalid arguments. Usage:

  python convert_to_matfile.py <OL_exp_infoDATE_TIME.txt> <OL_exp_rosDATE_TIME.bag> <outfile.mat>

  Where the following names have been replaced with appropriate filenames:

  <OL_exp_infoDATE_TIME.txt> Input file containing the text file saved by 'open-loop-exp.py'
  <OL_exp_rosDATE_TIME.bag>  Input file containing the ros bag file saved by 'rosbag record'
  <outfile.mat>              Output file containing the trajectories
  """
        sys.exit(1)

    text_fname = sys.argv[1]
    bag_fname = sys.argv[2]
    out_fname = sys.argv[3]
    bag = rosbag.Bag(bag_fname)

    array = np.array # put "array" in namespace (needed for eval statement below)
    obj_id = None
    magnet_obj_id = None
    home_pixels = None
    trials = []
    save_obj_ids = set()
    for text_line in open(text_fname).readlines():
        text_line=eval(text_line.strip())
        misc, tstamp, state = text_line[:3]
        assert misc=='misc'
        trigger = False
        if state == 'going_home':
            obj_id = None
            continue
        elif state == 'trigger_obj_id':
            trigger = True
            obj_id = text_line[3]
        elif state == 'waiting':
            obj_id = None
            continue
        elif state == 'magnet_obj_id':
            obj_id = None
            magnet_obj_id = text_line[3]
        elif state == 'home_pixels':
            obj_id = None
            home_pixels = text_line[3]
            dist = np.sqrt(np.sum((home_pixels-HOME_PIXELS_REAL)**2))
            if dist > 45.0:
                print >> sys.stderr, ('ERROR: Bad home localization at time '
                                      '%s, ignoring the rest of the data.'%
                                      (tstamp,))
                break

        elif state == 'lost_magnet':
            obj_id = None
            magnet_obj_id = None
        else:
            raise ValueError('unknown state %s'%state)

        if trigger:
            #print text_line
            save_obj_ids.add(obj_id)
            save_obj_ids.add(magnet_obj_id)
            trials.append( (tstamp, obj_id, magnet_obj_id, 0) )

    trials = np.array(trials)

    ori_finder = OriFinder(bag)

    trajectories = []
    for topic, msg, t in bag.read_messages(topics=['/tracker_info']):
        for tracked_object in msg.tracked_objects:
            if tracked_object.obj_id not in save_obj_ids:
                continue

            (orientation, eccentricity, area) = ori_finder.find_ori( msg.stamp, tracked_object.x, tracked_object.y )
            trajectories.append( (tracked_object.obj_id,
                                  msg.stamp.to_sec(),
                                  tracked_object.x,
                                  tracked_object.y,
                                  orientation,
                                  eccentricity,
                                  area) )

    trajectories = np.array(trajectories)
    scipy.io.savemat(out_fname,{'trials':trials,
                                'trajectories':trajectories}, oned_as='row')
    print 'Saved %d trials to %s'%(len(trials),out_fname)
