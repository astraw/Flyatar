import roslib
roslib.load_manifest('stage_message_interface')
import rospy
from stage.msg import StageCommands
import numpy as np
import time
from stage.srv import Stage_StateRequest, Stage_State
import subprocess, threading

# This is hardcoded in the USB device that sends commands.
VELOCITY_STEP_DUR = 0.016

class DoThread(threading.Thread):
    def doit(self,**kwargs):
        self.done_callback = kwargs.get('callback',None)
        self.motor = kwargs['vm']

        del kwargs['callback']
        del kwargs['vm']
        self.kwargs =kwargs

        self.start()

    def run(self):
        kwargs = self.kwargs
        x = kwargs['x']
        y = kwargs['y']
        del kwargs['x']
        del kwargs['y']
        self.motor.goto_blocking(x,y,**kwargs)
        if self.done_callback is not None:
            self.done_callback()

class VirtualMotor(object):
    def __init__(self):
        self._publisher = rospy.Publisher('/Stage/Commands',
                                          StageCommands,
                                          tcp_nodelay=True,
                                          latch=True,
                                          )
        rospy.wait_for_service('get_stage_state')
        self._get_stage_state_proxy = rospy.ServiceProxy('get_stage_state',
                                                         Stage_State)
        self._last_usb = -np.inf

    def goto_async(self,x,y,dur=1.0,xtol=1.0,ytol=1.0, callback=None):
        do_thread = DoThread()
        do_thread.setDaemon(True)
        do_thread.doit(x=x,y=y,dur=dur,xtol=xtol,ytol=ytol, callback=callback, vm=self)

    def goto_blocking(self,x,y,dur=1.0,xtol=1.0,ytol=1.0):
        """goto position x,y, in dur seconds"""
        if 1:
            response = self.get_stage_state()
            dx = x-response.x
            dy = y-response.y

            vx = dx/dur
            vy = dy/dur

            vel_mag = abs(np.sqrt( vx**2 + vy**2))

            n_steps = int(round(dur/ VELOCITY_STEP_DUR))
            n_steps = max(1,n_steps)

            if 1:
                # in position mode, we just do one step
                n_steps=1

            msg = StageCommands()

            msg.x_position = [x]*n_steps
            msg.y_position = [y]*n_steps
            msg.x_velocity = []
            msg.y_velocity = []
            msg.velocity_magnitude = [vel_mag]*n_steps
            if vel_mag != 0.0:
                self.publish(msg)
        while 1:
            response = self.get_stage_state()
            if ((abs(x-response.x) < xtol) and
                (abs(y-response.y) < ytol)):
                break
        time.sleep(0.5)

    # All this stuff is so we don't confuse the motor device by too
    # frequent commands.
    def _throttle_usb(self):
        self._last_usb = -np.inf
        now = time.time()
        dur = now-self._last_usb
        minimum_usb_cycle_duration = 0.05 # only hit ROS this often
        while 1:
            if dur > minimum_usb_cycle_duration:
                break
            target = (self._last_usb+minimum_usb_cycle_duration)-now
            time.sleep(target)
            now = time.time()
        self._last_usb = now

    def publish(self,msg):
        self._throttle_usb()
        self._publisher.publish(msg)

    def get_stage_state(self):
        self._throttle_usb()
        return self._get_stage_state_proxy()
