import roslib
roslib.load_manifest('stage')
import rospy
from stage.srv import Stage_StateRequest, Stage_State
import numpy as np
import time

class App(object):
    def __init__(self):
        rospy.init_node('move_motor',
                        anonymous=True, # allow multiple instances to run
                        )
        self.set_stage_position = rospy.ServiceProxy('set_stage_position',
                                                     Stage_State)
    def goto_blocking(self,x,y,dur=1.0):
        """goto position x,y, in dur seconds"""
        stage_commands = Stage_StateRequest()
        stage_commands.x_position = [x]
        stage_commands.y_position = [y]
        stage_commands.x_velocity = [100]
        stage_commands.y_velocity = [100]
        response = self.set_stage_position(stage_commands)
        x = response.x
        y = response.y

    def circle(self):
        theta = np.linspace(0,2*np.pi,100)
        r = 50
        x = r*np.cos(theta)
        y = r*np.sin(theta)
        for (xi,yi) in zip(x,y):
            self.goto_blocking(xi,yi)

if __name__=='__main__':
    app = App()
    app.circle()
