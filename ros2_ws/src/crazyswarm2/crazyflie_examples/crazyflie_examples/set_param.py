#!/usr/bin/env python

from crazyflie_py import Crazyswarm


def main():
    # In case of key errors, wait for all the crazyflies to be fully connected
    # before running the script.
    # Also 'query_all_values_on_connect' should be set to True in the server.yaml file.
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    thrust_mult = 1
    thrust_step = 500
    thrust = 10000

    # disable LED (one by one)
    for cf in allcfs.crazyflies:
        #cf.setParam('led.bitmask', 128)
        #cf.setParam('motorPowerSet.enable', 1)
        #print('param set')
        cf.cmdVel(0.0,0.0,0.0,0.0)
        print('command sent')
        while thrust >= 10000:
            cf.cmdVel(0.0,0.0,0.0,float(thrust))
            print('thrust: ', thrust)
            timeHelper.sleep(0.1)
            if thrust >= 45000:
                thrust_mult = -1
            thrust += thrust_step*thrust_mult
        for _ in range (30):
            cf.cmdVel(0.0,0.0,0.0,0.0)
            timeHelper.sleep(0.1)
        timeHelper.sleep(1.0)
        #if cf.getParam('led.bitmask') != 128:
            #print('LED is not disabled!')
        #if cf.getParam('motorPowerSet.enable') == 1:
            #cf.setParam('motorPowerSet.m1', 7000)
        #print('motor command set')

    timeHelper.sleep(2.0)

    # enable LED (broadcast)
    #allcfs.setParam('led.bitmask', 0)
    #timeHelper.sleep(5.0)


if __name__ == '__main__':
    main()