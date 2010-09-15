import roslib
roslib.load_manifest('stage_tf')
import rospy
from stage.msg import StageCommands
import numpy as np
import time
from stage.srv import Stage_StateRequest, Stage_State
import subprocess
from virtual_motor import VirtualMotor
import threading

class App(object):
    def __init__(self):
        rospy.init_node('open_loop_experiment',
                        anonymous=True, # allow multiple instances to run
                        )
        rospy.wait_for_service('get_stage_state')
        self._home = (120,115)
        self.is_home = threading.Event()
        self.virtual_motor = VirtualMotor()

    def start_going_home(self):
        self.virtual_motor.goto_async( self._home[0], self._home[1], dur=1.0, callback=lambda : self.is_home.set())

    def run(self):
        self.start_going_home()
        print 'going home'
        self.is_home.wait()
        print 'got home'

        self.virtual_motor.goto_async( 150, 150, dur=1.0 )
        time.sleep(1.1)

if __name__=='__main__':
    app = App()
    app.run()
