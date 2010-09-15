import roslib
roslib.load_manifest('stage_tf')
import rospy
from stage.msg import StageCommands
import numpy as np
import time
from stage.srv import Stage_StateRequest, Stage_State
import subprocess
from virtual_motor import VirtualMotor

class App(object):
    def __init__(self):
        rospy.init_node('open_loop_experiment',
                        anonymous=True, # allow multiple instances to run
                        )
        rospy.wait_for_service('get_stage_state')
        self._home = (120,115)
        self.virtual_motor = VirtualMotor()

    def goto_blocking(self,*args,**kws):
        self.virtual_motor.goto_blocking(*args,**kws)

    def go_home_blocking(self,dur=3.0):
        self.goto_blocking(self._home[0],self._home[1],dur)

    def circle(self,x0=120,y0=120,r=50,n_pts=20,dur=10.0):
        theta = np.linspace(0,2*np.pi,100)
        x = r*np.cos(theta)+x0
        y = r*np.sin(theta)+y0
        for (xi,yi) in zip(x,y):
            self.goto_blocking(xi,yi,dur=dur/n_pts)

    def run(self):
        #self.go_home_blocking()
        self.star()

    def star(self):
        n = 5
        theta = np.linspace(0,2*np.pi,n+1)[:n]
        r = 50
        x0,y0 = self._home
        xs = r*np.cos(theta)+x0
        ys = r*np.sin(theta)+y0

        self.go_home_blocking()
        for x,y in zip(xs,ys):
            self.goto_blocking(x,y)
            self.go_home_blocking()

if __name__=='__main__':
    app = App()
    app.run()
