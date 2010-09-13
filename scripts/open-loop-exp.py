import roslib
roslib.load_manifest('stage_tf')
import rospy
from stage.msg import StageCommands
import numpy as np
import time
from stage.srv import Stage_StateRequest, Stage_State
import subprocess

class VirtualMotor(object):
    def __init__(self):
        self._publisher = rospy.Publisher('/Stage/Commands',
                                          StageCommands,
                                          tcp_nodelay=True,
                                          latch=True,
                                          )
        self._get_stage_state_proxy = rospy.ServiceProxy('get_stage_state',
                                                         Stage_State)
        self.goto_blocking = self.goto_blocking_ros
        #self.goto_blocking = self.goto_blocking_subprocess
        self._last_usb = -np.inf

    def goto_blocking_subprocess(self,x,y,dur=3.0):
        response = self.get_stage_state()
        print response.x, response.y

        dx = x-response.x
        dy = y-response.y

        vx = dx/dur
        vy = dy/dur

        cmd = 'rostopic pub -1 /Stage/Commands stage/StageCommands -- True False False [%f] [%f] [%f] [%f]'%(x,y,vx,vy)
        tstart=time.time()
        subprocess.check_call(cmd,shell=True)
        while 1:
            now = time.time()
            if (now-tstart) >= dur:
                break
            sleep_dur = (tstart+dur)-now
            time.sleep(sleep_dur)
        response = self.get_stage_state()
        print response.x, response.y

    def goto_blocking_ros(self,x,y,dur=1.0,xtol=1.0,ytol=1.0):
        """goto position x,y, in dur seconds"""
        if 1:
            response = self.get_stage_state()
            dx = x-response.x
            dy = y-response.y

            vx = dx/dur
            vy = dy/dur

            n_steps = int(round(dur/ 0.060)) # 60 msec per step
            n_steps = max(1,n_steps)

            if 1:
                n_steps=1

            if 0:
                n_steps=1
                vx=50
                vy=50

            msg = StageCommands()
            
            msg.position_control = True
            msg.velocity_control = False
            msg.lookup_table_correct = False
            
            print 'response.x,response.y,x,y, vx,vy,n_steps',response.x,response.y,x,y, vx,vy,n_steps
            msg.x_position = [x]*n_steps
            msg.y_position = [y]*n_steps
            msg.x_velocity = [vx]*n_steps
            msg.y_velocity = [vy]*n_steps
            self.publish(msg)
        while 1:
            response = self.get_stage_state()
            #print 'response.header.stamp',response.header.stamp
            #print response.x
            #print response.y
            if ((abs(x-response.x) < xtol) and
                (abs(y-response.y) < ytol)):
                break
        time.sleep(0.5)

        #time.sleep(dur)
        #print 'dur',dur


    # All this stuff is so we don't confuse the motor device by too
    # frequent commands.

    def _throttle_usb(self):
        self._last_usb = -np.inf
        now = time.time()
        dur = now-self._last_usb
        minimum_usb_cycle_duration = 0.3
        while 1:
            if dur > minimum_usb_cycle_duration:
                break
            # only hit USB every 100 msec
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
