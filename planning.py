#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Ammar on 3/6
"""

from math import sin, cos, pi, atan2
from collections import deque
from simulator.utilities import Utilities


# Utility functions
UTIL_SIGN = lambda x: x and (1, -1)[x < 0]

# Planning constants for the robot
R = .045
L = .1
D = .2427
A_BS = .056

MARGIN_CENTER_BIN = .2038
SAME_BIN_MARGIN_X = .0508
V_SCALE = 2
OMEGA_MAX_ALLOWED = 5

GOAL_X = .9398
BIN_XS = [-0.5461, -0.27305, 0.0, 0.27305, 0.5461]
BIN_YBASE = .2667

# Enum constants
DIRECTION_FORWARD = 0
DIRECTION_REVERSE = 1

SIDE_NEG = -1
SIDE_NON = 0
SIDE_POS = 1


# Should return the order the bins are to be picked in
# Will likely be hardcoded
def order_blocks(block_config):
    # TODO: Actually compute and return the ordering
    return block_config


# Functions for whether we have met our goal
# We define them here so we can explicitly write them in wp_queue
def cf_bounds(minx=float('-inf'), maxx=float('inf'), miny=float('-inf'), maxy=float('inf')):
    def ret(c):
        return (minx <= c[0] and c[0] <= maxx) and (miny <= c[1] and c[1] <= maxy)
    return ret
def cf_dist(x, y, dist=.05):
    def ret(c):
        return dist**2 >= (c[0]-x)**2 + (c[1]-y)**2
    return ret

# A queue for all the waypoints
# Format is (x, y, direction, doneFunc)
wp_queue = deque([])

# To tell the outside world when we are done with our current path
def wp_done():
    return len(wp_queue) == 0


# Utility function to convert between domains
def xydot_to_w(v, curT, dire):

    # Component extraction
    xDotTarg, yDotTarg = v

    # Compute matrix coefficients
    xDotC0 = R/2 * cos(curT) + R*L/D * sin(curT)
    xDotC1 = R/2 * cos(curT) - R*L/D * sin(curT)
    yDotC0 = R/2 * sin(curT) - R*L/D * cos(curT)
    yDotC1 = R/2 * sin(curT) + R*L/D * cos(curT)

    # Invert the matrix
    matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)
    wR = matDetInv * ( yDotC1 * xDotTarg - xDotC1 * yDotTarg)
    wL = matDetInv * (-yDotC0 * xDotTarg + xDotC0 * yDotTarg)

    # Normalize omegas
    wMax = max(abs(wR), abs(wL))
    wR *= OMEGA_MAX_ALLOWED / max(wMax, 1)
    wL *= OMEGA_MAX_ALLOWED / max(wMax, 1)

    return (wR, wL) if dire == DIRECTION_FORWARD else (-wL, -wR)

# Utility function just to turn us around
def match_pose_to_dir(v, dire):
    # If the direction doesn't need to be changed, don't
    if dire == DIRECTION_FORWARD:
        return v
    # Otherwise, move the control point and turn 180 degrees
    else:
        x, y, t = v
        return (x - 2*L*cos(t), y - 2*L*sin(t), (t + pi) % (2*pi))


# Does exactly what it says
# Depends on the next waypoint
def compute_wheel_velocities(cur):
    global wp_queue
    try:
        wp = wp_queue[0]
        cur = match_pose_to_dir(cur, wp[2])

        if wp[3](cur):
            wp_queue.popleft()
            return (0,0)
        else:
            d = (V_SCALE * (wp[0]-cur[0]), V_SCALE * (wp[1]-cur[1]))
            return xydot_to_w(d, cur[2], wp[2])
    except IndexError:
        return (0,0)


# Queues driving directly toward a given goal position
def queue_direct(goal):
    global wp_queue

    # We stop when we are a hardcoded distance away
    wp_queue.append((
        goal[0],  goal[1], DIRECTION_FORWARD,
        cf_dist(goal[0], goal[1], dist=.01)
    ))

# Queues backing out in a specified direction
def queue_back(cur, side):
    global wp_queue
    sy = UTIL_SIGN(cur[1])

    # Only back out, don't turn while doing so
    if side == SIDE_NON:
        wp_queue.append((
            cur[0], -.2*sy, DIRECTION_REVERSE,
            cf_bounds(maxy=-.1*sy) if sy > 0 else \
            cf_bounds(miny=-.1*sy)
        ))

    # If we need to turn in a direction
    # Use constants to determine how much to offset
    # TODO: Fine tune constants
    else:
        wp_queue.append((
            cur[0], -.2*sy, DIRECTION_REVERSE,
            cf_bounds(maxy=0) if sy > 0 else \
            cf_bounds(miny=0)
        ))
        wp_queue.append((
            cur[0] + side*.30, 0, DIRECTION_REVERSE,
            cf_bounds(minx=cur[0]+side*.20) if side > 0 else \
            cf_bounds(maxx=cur[0]+side*.20)
        ))

# Queues going into a bin from a certain side
def queue_turnin(p_bin, side):
    global wp_queue

    # If we are approaching from a side, go outside the bin first
    if side != SIDE_NON:
        wp_queue.append((
            p_bin[0] - side*.07, 0, DIRECTION_FORWARD,
            cf_bounds(maxx=p_bin[0]-side*.05) if side > 0 else \
            cf_bounds(minx=p_bin[0]-side*.05)
        ))

    # Always actually go into the bin
    queue_direct(p_bin)

# Queue going to the end position from our current
def queue_end(cur):
    global wp_queue

    # Only back out of the bin if we are in a bin
    if abs(cur[1]) >= MARGIN_CENTER_BIN:
        queue_back(cur, SIDE_NON)

    # Go to the goal
    queue_direct((GOAL_X + A_BS + L, 0))

# Queue going from our current position to an offset inside a bin
# Note that the bins are ONE INDEXED for consistency with the Colab Notebook
def queue_bin(cur, n, off):
    global wp_queue
    # The position we are going to inside the bin
    p_bin = (BIN_XS[(n-1)%5], UTIL_SIGN(n-5.5)*(BIN_YBASE + off))
    # The side we are approaching the bin from
    side_turnin = SIDE_NEG if cur[0] < p_bin[0] else SIDE_POS
    # Depends on whether we need to turn out
    side_back = side_turnin if abs(cur[0]-p_bin[0]) < SAME_BIN_MARGIN_X else SIDE_NON

    # Check if we are already in the correct bin
    # If we are in a bin on the right side and in the right lane
    if abs(cur[1]) > MARGIN_CENTER_BIN and UTIL_SIGN(cur[1]) == UTIL_SIGN(p_bin[1]) and abs(cur[0]-p_bin[0]) <= SAME_BIN_MARGIN_X:
        queue_direct(p_bin)
    else:
        # Only back up if we need to
        if abs(cur[1]) > MARGIN_CENTER_BIN:
            queue_back(cur, side_back)
        queue_turnin(p_bin, side_turnin)
