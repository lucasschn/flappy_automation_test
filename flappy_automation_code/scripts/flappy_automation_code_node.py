#!/usr/bin/env python
#coding=UTF-8
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# topics are : 
# /flappy_acc for acceleration vector
# /flappy_vel for velocity vector
# /flappy_laser_scan for sensor data            

# constants
GOING_THROUGH_SAFE_DISTANCE = 0.25
SCAN_DISTANCE = 2
SCAN_VELOCITY = 0.3
APPROACH_VELOCITY = 0.8
GOING_THROUGH_VELOCITY = 0.4
MIN_DIST_TO_SCREEN_LIMITS = 0.5
KP_pos = .5 # proportional gain acceleration/distance_to_hole
KP_vel = 1 # proportional gain velocity/reference velocity

# global variables
state = 'approach'
distance_to_gate_y = 0
gate_width = None
upper_screen_limit_y = None
lower_screen_limit_y = None
rocks_x = None
vel_x = 0
vel_y = 0
pos_y = None
gate_probas = np.array([0.1]*10) # probability of gate being in one tenth fraction of screen height (from low to high)

def setLowerScreenLimit(pointcloud_y,angles,intensities):
    """ Returns True if a lower screen limit has been detected by the lasers, False elsewise.
        If the screen limit has been detected, also updates the value of the global variables lower_screen_limit_y with the distance of the screen limit with respect to the bird."""
        
    if intensities[0]: # there is either an lower screen limit or a rock
        if pointcloud_y[0] == pointcloud_y[1] and pointcloud_y[1] == pointcloud_y[2]: # lower screen limit detected
            global lower_screen_limit_y
            # discard the 1th element as well
            lower_screen_limit_y = pointcloud_y[0]
            return True
        else:
            return False
    
def setUpperScreenLimit(pointcloud_y,angles,intensities):
    """ Returns True if an upper screen limit has been detected by the lasers, False elsewise.
        If the screen limit has been detected, also updates the value of the global variables upper_screen_limit_y with the distance of the screen limit with respect to the bird."""
    if intensities[-1]: # there is either an upper screen limit or a rock
        if pointcloud_y[-1] == pointcloud_y[-2] and pointcloud_y[-2] == pointcloud_y[-3]: # upper screen limit detected
            global upper_screen_limit_y
            # discard the -2th element as well
            upper_screen_limit_y = pointcloud_y[-1]
            return True
        else:
            return False
    
def getRocksPosition(pointcloud_x, range_min, range_max):
    """ Returns the coordinates of the rock wall if there is one, returns None otherwise.
        Horizontal range of lasers is divided in bins of 0.3m. Defines the presence of a rock wall as at least 3 laser points falling in the same bin."""
        
    bin_edges = np.linspace(range_min,range_max,round((range_max-range_min)/0.3))
    distr,_ = np.histogram(pointcloud_x, bins = bin_edges)
    rocks_position = None
    possible_rocks_positions = []
    for bin_index in range(len(bin_edges)-1):  
        if distr[bin_index] >= 3: # if there are 3 rocks in the same bin
            possible_rocks_positions.append((bin_edges[bin_index] + bin_edges[bin_index+1])/2)
    if len(possible_rocks_positions) > 0:
        rocks_position = np.min(possible_rocks_positions) # if several rock walls detected, we are interested in the closest one    
    return rocks_position

def setGatePosition(pointcloud_x, angles, rocks_x):
    """ Returns True if the gate through the rock wall has been found, returns False elsewise.
        A gate is detected when there is one unique hole where several consecutive laser rays are going through."""
    is_going_through = (pointcloud_x - rocks_x) > GOING_THROUGH_SAFE_DISTANCE # laser ray going through the rock wall is defined as the laser point x-coordinate being significantly behind the rock wall.
    # we only compute the length of the hole when there are several rays going through
    if np.sum(is_going_through) > 1: # there might be one big or multiple holes
        holes_lengths, indices_holes_start = getHolesLength(is_going_through) # if hole_length >1, it's a gate!
        if np.max(holes_lengths) > 1: # if there is at least one big hole, we will detect a gate
        # now there are two cases, either there is only one big hole or there a multiple ones (big or small)
            if len(indices_holes_start) > 1: # if there are multiple holes
                index_gate_start = indices_holes_start[holes_lengths > 1]   
                if isinstance(index_gate_start, np.ndarray) and len(index_gate_start)!=1: # if there are multiple holes with length > 1
                        # Several holes have been gone through by more than one ray. The gate position cannot be determined
                        return False
                index_gate_stop = indices_holes_start[holes_lengths > 1] + holes_lengths[holes_lengths > 1] - 1 # a hole of length 1 starts and finishes at the same index
                global gate_width                
                gate_width = holes_lengths[holes_lengths > 1]
                
            else: # there is only one big hole
                index_gate_start = indices_holes_start
                print "Gate starts at laser ray {}".format(indices_holes_start)
                index_gate_stop = indices_holes_start + holes_lengths - 1 # a hole of length 1 starts and finishes at the same index
                global gate_width
                gate_width = holes_lengths
                
            avg_angle = (angles[index_gate_start] + angles[index_gate_stop])/2 # the gate position is the vertical distance between its middle point and the bird
            global distance_to_gate_y                
            distance_to_gate_y = rocks_x * math.tan(avg_angle) # distance_to_gate_x is equivalent to rocks_x (and not to pointcloud_x)
            return True
    else:
        return False

def getHolesLength(is_going_through):
    is_going_through_diff = np.diff(np.concatenate([[False], is_going_through]).astype('int')) # concatenating a False value at first to make appear a hole start at next line if first value of is_going_through is True
    #print "is_going_through_diff: {}".format(is_going_through_diff)    
    indices_holes_start = np.where(is_going_through_diff == 1)[0] # is_going_through_diff == 1 means it goes from False to True at that index in is_going_through: a hole starts at this index 
#    if len(indices_holes_start) < 2: # there is only one big hole
#        raise Warning('There are not multiple holes. Detected holes at laser index {}'.format(indices_holes_start))
    print "Indices holes start {}".format(indices_holes_start)       
    lengths_list = [] # is a list
    for index_hole_start in indices_holes_start:
        index_hole_stop = None
        next_index = index_hole_start + 1
        while not index_hole_stop:
            if next_index>=len(is_going_through_diff) or is_going_through_diff[next_index] == -1:
                index_hole_stop = next_index
                lengths_list.append(index_hole_stop - index_hole_start)
                print "Hole starts at laser ray {} and has length {}".format(index_hole_start, lengths_list[-1])
            # print "Still in the while loop ... next index = {}".format(next_index)
            next_index += 1
    length_holes = np.array(lengths_list)
    print "Length holes {}".format(length_holes)     
    return length_holes, indices_holes_start

def evaluateYPosition():
    global pos_y
    pos_y += vel_y/30
    if pos_y < 0:
        raise ValueError("Y-position is negative.")
    print "Y-position = {}m".format(round(pos_y,2))

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)


def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    # rospy.Subscriber(topic, message, callback)
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    """ The velocity callback is the controller. A cascade control scheme is used: 
        The velocity and the position are both regulated by a P controller."""
    # msg has the format of geometry_msgs::Vector3

    ### Position controller
    # reference is 0 distance to hole, so err_pos_y = distance_to_gate_y
    if state=='approach':
        ref_vel_x = APPROACH_VELOCITY
        ref_vel_y = 0.0
    elif state=='scan-down':
        ref_vel_x = 0.0
        ref_vel_y = -SCAN_VELOCITY
    elif state=='scan-up':
        ref_vel_x = 0.0
        ref_vel_y = SCAN_VELOCITY
    elif state=='go through': # then set a reference y-position
        if abs(distance_to_gate_y) <= 0.1 and abs(vel_y) < 0.01: 
            ref_vel_x = GOING_THROUGH_VELOCITY # literally go through
        else:
            ref_vel_x = 0.0 # wait for adjusting vertical position
        ref_pos_y = distance_to_gate_y
        # output of the position controller is the velocity reference
        ref_vel_y = KP_pos*ref_pos_y # if no hole is detected, y_distance is 0 and ref_vel_y = 0

    ### Velocity controller 
    # sensors signal
    global vel_x, vel_y
    vel_x = msg.x
    vel_y = msg.y

    # errors
    err_vel_x = ref_vel_x - vel_x
    err_vel_y = ref_vel_y - vel_y

    acc_x = KP_vel*err_vel_x
    acc_y = KP_vel*err_vel_y

    #print "Controller: pos. err. = y: {}".format(distance_to_gate_y)
    #print "Controller: vel. err. = x: {}, y: {}".format(err_vel_x, err_vel_y)
    #print "Controller: Acceleration = x: {}, y: {}".format(acc_x, acc_y)
    
    pub_acc_cmd.publish(Vector3(acc_x,acc_y,0))

def laserScanCallback(msg):
    """ laserScanCallback is the callback function executed when a message is received. It takes the message as an argument, so it can display and make use of information contained in it.
        Single scan from a planar laser range-finder

        float32 angle_min = -45°        # start angle of the scan [rad]
        float32 angle_max = +45°       # end angle of the scan [rad]
        float32 angle_increment = 90/8° # angular distance between measurements [rad]

        float32 time_increment   # time between measurements [seconds] - if your scanner
                                # is moving, this will be used in interpolating position
                                # of 3d points
        float32 scan_time        # time between scans [seconds]

        float32 range_min = 0       # minimum range value [m]
        float32 range_max = 3.55        # maximum range value [m]

        float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
        float32[] intensities    # Is 1.0 if the laser ray has encountered an obstacle, 0.0 otherwise.
        """

    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
    print "State = {}".format(state)
    angles = msg.angle_min + np.multiply(np.arange(start=0,stop=9,dtype=int),msg.angle_increment)
    
        # print "Laser range: {}, angle: {}".format(msg.ranges[k], angles[k]) # this works
    # compute points positions realtive to flappy bird:
    pointcloud_x = np.round(np.multiply(msg.ranges,np.cos(angles)),2) # precise to 1 pixel
    pointcloud_y = np.round(np.multiply(msg.ranges,np.sin(angles)),2) # precise to 1 pixel
    
#==============================================================================
#     for k in range(len(msg.ranges)-1,-1,-1): # there a 9 lasers
#         print "Point at {}°: x = {}, y = {}, intensity = {}".format(angles_deg[k], pointcloud_x[k], pointcloud_y[k], msg.intensities[k])
#==============================================================================
 
    if setUpperScreenLimit(pointcloud_y, angles, msg.intensities):
        pass
        #print "Upper screen limit detected at {}m".format(upper_screen_limit_y)
    elif upper_screen_limit_y: # update position based on velocity
        global upper_screen_limit_y
        upper_screen_limit_y -= vel_y/30 # frequency is 30Hz
        #print "Upper screen limit at {}m".format(upper_screen_limit_y)
        
    if setLowerScreenLimit(pointcloud_y, angles, msg.intensities):
        pass
        #print "Lower screen limit detected at {}m".format(lower_screen_limit_y)
    elif lower_screen_limit_y: # update position based on velocity
        global lower_screen_limit_y 
        lower_screen_limit_y -= vel_y/30
        #print "Lower screen limit at {}m".format(lower_screen_limit_y)

    if state =='approach':
        if np.any(msg.intensities[3:6]) and getRocksPosition(pointcloud_x, msg.range_min, msg.range_max): # rock wall detected (last index not included in a slice)
            global rocks_x            
            rocks_x = getRocksPosition(pointcloud_x, msg.range_min, msg.range_max)
            print "Rock wall detected at {}m".format(round(rocks_x,2))
            
        if rocks_x is not None and rocks_x <= SCAN_DISTANCE:
            global state            
            state = 'scan-down' # change state from approach to scanning down
        
    elif state == 'scan-down' or state == 'scan-up':
        global rocks_x
        rocks_x = getRocksPosition(pointcloud_x, msg.range_min, msg.range_max)
        if pos_y is not None:
             evaluateYPosition()
        
        if setGatePosition(pointcloud_x, angles, rocks_x) and lower_screen_limit_y is not None and upper_screen_limit_y is not None:
            print "Gate detected at {}m above (width {})".format(distance_to_gate_y, gate_width) 
            if pos_y is not None:
                likely_gate_pos_y = distance_to_gate_y + pos_y
                screen_height = upper_screen_limit_y + abs(lower_screen_limit_y)
                screen_fraction_to_reinforce = likely_gate_pos_y/screen_height # something between 0 and 1
                index_to_reinforce = int(round(screen_fraction_to_reinforce*9))
                print "Reinforcing index {}".format(index_to_reinforce)
                global gate_probas
                gate_probas[index_to_reinforce] += 0.02*gate_width**2 # better to have something depending on the hole length
                gate_probas = [proba/np.sum(gate_probas) for proba in gate_probas]   
                # print "Gate probas = {}".format(gate_probas)
        else:
            print "Gate not found."
        
        print "Max proba = {}".format(np.max(gate_probas))
        if np.max(gate_probas) >= 0.95:
            global state
            state = 'go through'
        
        if state=='scan-down':
            
            if lower_screen_limit_y is not None and abs(lower_screen_limit_y) <= MIN_DIST_TO_SCREEN_LIMITS:
                 print "Lowest point reached." 
                 global pos_y
                 pos_y = -lower_screen_limit_y # initialize pos_y
                 global state
                 state = 'scan-up'
             
        elif state == 'scan-up':
        
            if upper_screen_limit_y is not None and upper_screen_limit_y <= MIN_DIST_TO_SCREEN_LIMITS:
                print "Highest point reached"
                global state
                state = 'scan-down'
            
    elif state == 'go through':
        global rocks_x
        rocks_x -= vel_x/30
        global distance_to_gate_y
        distance_to_gate_y -= vel_y/30
        # corrections of gate position if needed
        if abs(pointcloud_y[0]) < 0.1:
            distance_to_gate_y += 0.1
        elif pointcloud_y[-1] < 0.1:
            distance_to_gate_y -= 0.1
        if rocks_x < -.5:
            global state, upper_screen_limit_y, lower_screen_limit_y, gate_probas
            state = 'approach'
            upper_screen_limit_y = None
            lower_screen_limit_y = None
            pos_y = None
            gate_probas = np.array([0.1]*9)
        
    else:
        raise ValueError('The state is undetermined (state {}).'.format(state))

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass