#!/usr/bin/env python
import roslib
roslib.load_manifest('flytrax_ros')

import rospy
from flytrax_ros.msg import FlytraxInfo, TrackedObject, TrackerInfo, Detection
import numpy as np
import time
import subprocess, sys, os
from virtual_motor import VirtualMotor
import threading
import ols
import pickle

TRIGGER_DISTANCE = 150 # in pixels
MOVE_DISTANCE = 250 # in pixels
CAL_FNAME = 'calibration.pkl'

class LostMagnetError(RuntimeError):
    pass

def list_to_MyObjectSet(element_list):
    elements = {}
    for el in element_list:
        elements[el.obj_id] = np.array( (el.x, el.y) )
    return MyObjectSet(elements)

class MyObjectSet(object):
    def __init__(self,elements,thresh_dist = 10.0):
        self.elements = elements
        self.keys = self.elements.keys()
        self.keys.sort()
        self.thresh_dist = thresh_dist
    def __getitem__(self,idx):
        key = self.keys[idx]
        return self.elements[key]
    def __len__(self):
        return len(self.elements)
    def __sub__(self,other):
        # return the elements in self that have moved in other
        result = {}
        for obj_id in self.keys:
            obj_loc = self.elements[obj_id]
            if obj_id in other.elements:
                other_loc = other.elements[obj_id]

                dist_2d = obj_loc - other_loc
                dist = np.sqrt(np.sum(dist_2d**2))
                if dist > self.thresh_dist:
                    result[obj_id] = self.elements[obj_id]
            else:
                # original object ID not present in other
                continue
        return MyObjectSet(result)
    def take_closest( self, xy ):
        assert len(self.keys)>0
        xy = np.array(xy)
        dists = []
        for obj_id in self.keys:
            obj_loc = self.elements[obj_id]
            d2 = (obj_loc-xy)**2
            np.sqrt(d2[0]+d2[1])
            dists.append( np.sqrt(d2[0]+d2[1]) )
        dists = np.array(dists)
        idx = np.argmin(dists)

        obj_id = self.keys[idx]
        result = { obj_id : self.elements[obj_id] }
        return MyObjectSet(result)

class App(object):
    def __init__(self):
        rospy.init_node('open_loop_experiment',
                        anonymous=True, # allow multiple instances to run
                        )
        self._tracker_info_subscriber = None
        self._input_topic_prefix = ''
        self._input_topic_prefix_changed()

        self._home_motor_coords = (120,115)
        self._not_home_motor_coords = (90,90)
        self.virtual_motor = VirtualMotor()
        self.current_objects_lock = threading.Lock()
        self.current_objects = None
        self.got_current_objects = threading.Event()
        self._home_pixels = None
        self._magnet_obj_id = None
        self.strftime = time.strftime('%Y%m%d_%H%M%S')
        fname = 'OL_exp_info'+self.strftime+'.txt'
        self.data_dump = open(fname,'w')

        self.find_home_pixels(load_cal=True)

    def _input_topic_prefix_changed(self):
        if self._tracker_info_subscriber is not None:
            # how to de-subscribe?
            raise NotImplementedError('')
        self._tracker_info_subscriber = rospy.Subscriber('%s/tracker_info'%self._input_topic_prefix,
                                                         TrackerInfo,
                                                         self.current_objects_callback)

    def current_objects_callback(self,value):
        with self.current_objects_lock:
            self.current_objects = value
        self.got_current_objects.set()

    def start_going_home(self):
        self.virtual_motor.goto_async( self._home_motor_coords[0],
                                       self._home_motor_coords[1],
                                       dur=1.0)#, callback=lambda : self.is_home.set())

    def get_current_2d_object_locations(self,max_age_sec=0.3):
        while 1:
            with self.current_objects_lock:
                value = self.current_objects
            if value is not None:
                now = time.time()
                data_time = value.stamp.to_seconds()
                age = now-data_time
                if age <= max_age_sec:
                    break
            self.got_current_objects.wait()

        objects = value.tracked_objects
        return objects

    def get_current_flies(self,**kwargs):
        objects = self.get_current_2d_object_locations(**kwargs)
        result = [obj for obj in objects if obj.obj_id != self._magnet_obj_id]
        return result

    def get_current_magnet(self,**kwargs):
        objects = self.get_current_2d_object_locations(**kwargs)
        result = [obj for obj in objects if obj.obj_id == self._magnet_obj_id]
        if len(result)==0:
            raise LostMagnetError('tracker lost the magnet')
        elif len(result) > 1:
            raise LostMagnetError('tracker found more than one magnet?')
        return result[0]

    def find_home_pixels(self,load_cal=False):
        possible_home_coords = []
        count = 0
        while 1:
            if 1:#len(possible_home_coords)==0:
                self.virtual_motor.goto_blocking( self._home_motor_coords[0],
                                                  self._home_motor_coords[1] )
                time.sleep(0.1) # allow any final residual movements to stop
                possible_home_coords = list_to_MyObjectSet(self.get_current_2d_object_locations())

            self.virtual_motor.goto_blocking( self._not_home_motor_coords[0],
                                              self._not_home_motor_coords[1] )
            time.sleep(0.1) # allow any final residual movements to stop
            not_home_coords = list_to_MyObjectSet(self.get_current_2d_object_locations())

            possible_home_coords = possible_home_coords - not_home_coords

            if len(possible_home_coords)==1:
                break
            count += 1

            if count > 3:
                print >> sys.stderr, 'WARNING: guessing closest home location'
                possible_home_coords = possible_home_coords.take_closest( (744,632) )
                break

        assert len(possible_home_coords)==1
        self._home_pixels = possible_home_coords[0]
        self._magnet_obj_id = possible_home_coords.keys[0]
        print 'self._magnet_obj_id',self._magnet_obj_id
        now_str = repr(time.time())
        self.data_dump.write( "['misc',%s,'magnet_obj_id',%d]\n"%(now_str,self._magnet_obj_id))

        print 'self._home_pixels',self._home_pixels
        self.data_dump.write( "['misc',%s,'home_pixels',%s]\n"%(now_str,repr(self._home_pixels)))
        self.go_home()

        if not load_cal:
            return

        if not os.path.exists(CAL_FNAME):
            self.cal_data = {'motor':[],'pixel':[]}

            self._register_coords( (0,0) )
            self._register_coords( (50,0) )
            self._register_coords( (0,50) )
            self._register_coords( (-50,0) )
            self._register_coords( (0,-50) )
        else:
            fd = open(CAL_FNAME,mode='rb')
            self.cal_data = pickle.load(fd)
            fd.close()

        self._calibrate()
        self.go_home()

    def _register_coords( self, home_offset ):
        motor_x = self._home_motor_coords[0] + home_offset[0]
        motor_y = self._home_motor_coords[1] + home_offset[1]
        self.virtual_motor.goto_blocking( motor_x, motor_y )
        time.sleep(0.1)  # allow any final residual movements to stop
        magnet = self.get_current_magnet()
        self.cal_data['motor'].append( (motor_x, motor_y) )
        self.cal_data['pixel'].append( (magnet.x, magnet.y) )

    def _calibrate(self):
        motor = np.array(self.cal_data['motor'])
        pixel = np.array(self.cal_data['pixel'])

        p2m_x = ols.ols( motor[:,0], pixel, 'motor_x', 'pixels' )
        print 'p2m_x.b',p2m_x.b
        self.p2m_x_b = p2m_x.b
        p2m_y = ols.ols( motor[:,1], pixel, 'motor_y', 'pixels' )
        print 'p2m_y.b',p2m_y.b
        self.p2m_y_b = p2m_y.b

        if 1:
            home_x_test, home_y_test = self.pixels2motor( self._home_pixels[0], self._home_pixels[1] )
            print 'home_x_test',home_x_test
            print 'home_y_test',home_y_test
            print 'self._home_motor_coords',self._home_motor_coords

        fd = open(CAL_FNAME,mode='wb')
        pickle.dump(self.cal_data,fd)
        fd.close()

    def go_home(self):
        self.virtual_motor.goto_blocking( self._home_motor_coords[0],
                                          self._home_motor_coords[1] )

    def run(self):
        rosbag_fname = 'OL_exp_ros'+self.strftime
        cmd = 'rosbag record -O %s /flytrax_info /tracker_info'%(rosbag_fname,)
        print >> sys.stderr, '*'*200
        print >> sys.stderr, 'type this to save data:'
        print >> sys.stderr, cmd
        print >> sys.stderr, '*'*200

        print 'waiting...'
        while 1:
            try:
                fly_objects = self.get_current_flies()

                for fly_obj in fly_objects:
                    distance_from_magnet,radial_velocity,fly_angle = self._calc_home_info(fly_obj)
                    #print 'distance_from_magnet,radial_velocity,fly_angle',distance_from_magnet,radial_velocity,fly_angle
                    if distance_from_magnet <= TRIGGER_DISTANCE:
                        if radial_velocity <= 0:
                            print 'triggering experiment against fly %d at %.0f, %.0f'%(fly_obj.obj_id,
                                                                                        fly_obj.x,
                                                                                        fly_obj.y)
                            now_str = repr(time.time())
                            self.data_dump.write( "['misc',%s,'trigger_obj_id',%d]\n"%(now_str,fly_obj.obj_id))
                            self.data_dump.flush()

                            angle_offset = 0
                            if 0:
                                target_x_pixels = MOVE_DISTANCE*np.cos(fly_angle + angle_offset ) + self._home_pixels[0]
                                target_y_pixels = MOVE_DISTANCE*np.sin(fly_angle + angle_offset ) + self._home_pixels[1]
                            else:
                                target_x_pixels = fly_obj.x
                                target_y_pixels = fly_obj.y

                            target_x, target_y = self.pixels2motor( target_x_pixels, target_y_pixels )

                            self.virtual_motor.goto_blocking( target_x, target_y, dur=1.0 )
                            magnet = self.get_current_magnet()
                            print >> sys.stderr, 'went to',magnet.x, magnet.y
                            time.sleep(4.0) # give some time for trial to end

                            now_str = repr(time.time())
                            self.data_dump.write( "['misc',%s,'going_home']\n"%(now_str))
                            self.data_dump.flush()

                            self.go_home()
                            print 'waiting...'
                            now_str = repr(time.time())
                            self.data_dump.write( "['misc',%s,'waiting']\n"%(now_str))
                            self.data_dump.flush()
            except LostMagnetError:
                print 'aborted - lost magnet'
                now_str = repr(time.time())
                self.data_dump.write( "['misc',%s,'lost_magnet']\n"%(now_str))
                self.find_home_pixels()

        self.virtual_motor.goto_async( 150, 150, dur=1.0 )
        time.sleep(1.1)

    def _calc_home_info(self, fly_obj):
        x0 = fly_obj.x - self._home_pixels[0]
        y0 = fly_obj.y - self._home_pixels[1]
        dist0 = np.sqrt(x0**2 + y0**2)

        angle = np.arctan2( y0, x0 )

        x1 = x0 + fly_obj.x_vel
        y1 = y0 + fly_obj.y_vel
        dist1 = np.sqrt(x1**2 + y1**2)

        rad_vel = (dist1-dist0) # poor man's radial velocity. at least the sign is right.
        return dist0, rad_vel, angle

    def pixels2motor( self, target_x_pixels, target_y_pixels ):
        xb = self.p2m_x_b
        yb = self.p2m_y_b
        motor_x = xb[0] + xb[1]*target_x_pixels + xb[2]*target_y_pixels
        motor_y = yb[0] + yb[1]*target_x_pixels + yb[2]*target_y_pixels
        return motor_x, motor_y


if __name__=='__main__':
    app = App()
    app.run()
