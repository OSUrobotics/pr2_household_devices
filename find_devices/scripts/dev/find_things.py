#!/usr/bin/env python

import roslib
roslib.load_manifest('find_devices')

import rospy

from find_devices.srv import *


if __name__ == '__main__':

    #rospy.init_node('find_things')

    

    # Import all scripts
    finder = __import__('find_devices.find_button')
    print dir()
    print dir(finder)


    """
    import os
    script_path = roslib.packages.get_pkg_dir('find_devices') + '/src/find_devices/'
    script_names = os.listdir(script_path)
    script_names = [script_name[:-3] for script_name in script_names if script_name.endswith('.py') and not script_name.startswith('_')]
    finders = dict()
    for script_name in script_names:
        finders[script_name] = eval('__import__("find_devices.' + script_name + '")')
    print finders
    print dir(finders['user_tools'])
    """

