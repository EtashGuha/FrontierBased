#!/usr/bin/env python

import numpy as np
import dippykit as dip
from skimage import data,filters,segmentation,measure,morphology,color
import math
from skimage.measure import label, regionprops


def main():
    loop_num = 1
    while 1:
        print('counter: ', loop_num)
        loop_num = loop_num + 1
            # robot pose update
        odom = np.loadtxt('odom.txt', delimiter=' ', unpack=True, ndmin=0)
            # establish publisher
        pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs/PoseStamped, queue_size = 10, latch= False)
            # rotate 360'
            # rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 100.0, y: 1000.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
        pub.publish({header: {stamp: now, frame_id: "map"}, pose: {position: {x: 100.0, y: 1000.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}})
            # info and map
        info = np.loadtxt('info.txt', delimiter=' ', unpack=True, ndmin=0)
        grid = np.loadtxt('rawmap.txt')
            # process the map and get a goal point
        next_goal  = process(info, grid, odom)
        x = next_goal[0]
        y = next_goal[1]
        z = 0.0
            # go to this goal
        rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: %s, y: %s, z: %s}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' %(x, y, z)
#        Move base launch xml
# change map to odom

def process(info, grid, odom):

    resolution = info[3]
    origin_x = info[0]
    origin_y = info[1]
    # origin_z = info[2]
    size = grid.shape
    robot_pose_world = odom  # robot initial position in world
    robot_pose_pixel = []  # robot initial position in grid (pixel in image)
    robot_pose_pixel[0] = (robot_pose_world[0] - origin_x) / resolution
    robot_pose_pixel[1] = (robot_pose_world[0] - origin_y) / resolution

    #--------------------------------------------- open cells ---------------------

    thresh_low = 0
    thresh_high = 10
    dst =((grid <= thresh_high) & (grid >= thresh_low))*1.0   #threshold
    # "1" in dst are the open cells
    # "0" in dst are the unvisited cells and occupied cells

    # detect contours
    contours_open = measure.find_contours(dst, 0.5)
    contours_open_cell = list()
    for ele in contours_open:
        for cell in ele:
            contours_open_cell.append(cell.tolist())
    #print(contours_open_cell)

    #--------------------------------------------- unvisited cells ---------------------
    thresh_low = -1
    thresh_high = -1
    dst =((grid <= thresh_high) & (grid >= thresh_low))*1.0   #threshold
    # "1" in dst are the unvisited cells
    # "0" in dst are the open cells and occupied cells

    # find contours
    contours_unvisited = measure.find_contours(dst, 0.5)
    contours_unvisited_cell = list()
    for ele in contours_unvisited:
        for cell in ele:
            contours_unvisited_cell.append(cell.tolist())
    #print(contours_unvisited_cell)

    #----------------------------------------------------------------
    # frontier detection ! ! !
    frontier_cells = [x for x in contours_unvisited_cell if x in contours_open_cell]
    #print('frontier: ')
    #print(frontier_cells)  # to find the same elements in both lists

    grid_frontier = np.zeros(size)
    for ele in frontier_cells:
        grid_frontier[math.floor(ele[0]), math.floor(ele[1])] = 1

    # group them!
    grid_frontier_img = dip.float_to_im(grid_frontier)
    conected_frontier, label_num = measure.label(grid_frontier_img, return_num=True, connectivity=2)
    print("num of frontiers: %d" %label_num)
    conected_frontier = dip.float_to_im(conected_frontier/label_num)

    # delete small frontiers
    #image_label_overlay = label2rgb(conected_frontier, image=grid_frontier_img)
    #fig, ax = plt.subplots(figsize=(10, 6))
    #ax.imshow(image_label_overlay)

    manh_dist = []  # stores distances
    cents = []   # stores centers of frontiers

    for region in regionprops(conected_frontier):
        # take regions with large enough areas
        if region.area >= 10:                #  do not consider small frontier groups
            # print the centroid of each valid region
            cen_y = region.centroid[0]   # Centroid coordinate tuple (row, col)
            cen_x = region.centroid[1]   # Centroid coordinate tuple (row, col)
            cents.append([cen_x, cen_y])
            manh = abs(cen_x - robot_pose_pixel[0]) + abs(cen_y - robot_pose_pixel[1]) # Manhattan Distance from robot to each frontier center
            manh_dist.append(manh)
            #print(region.centroid)   # Centroid coordinate tuple (row, col)
            # draw rectangle around segmented coins
            #minr, minc, maxr, maxc = region.bbox
            #rect = mpatches.Rectangle((minc, minr), maxc - minc, maxr - minr,
             #                         fill=False, edgecolor='red', linewidth=1)
            #ax.add_patch(rect)
    #ax.set_axis_off()
    #plt.tight_layout()
    #plt.show()

    next_goal = cents[manh_dist.index(min(manh_dist))]
    # transform into real world
    next_goal[0] = next_goal[0]*resolution + origin_x
    next_goal[1] = next_goal[1]*resolution + origin_y
    print('next_goal: ', next_goal)

    return next_goal



main()






