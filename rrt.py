
import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib import collections  as mc
import numpy as np
import math
import random

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        ab = polygon[v]
        verts.append((ab[0]/10., ab[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch

'''
Render the problem
'''

def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()


def find_closest_point(P0, P1, P2):

    P0 = np.array(P0)
    P1 = np.array(P1)
    P2 = np.array(P2)

    A = P2 - P0
    B = P1 - P0

    B_inverse = P0 - P1
    C = P2 - P1

    world_frame = np.array([1,0]) - np.array([0,0])

    ab_dot = np.dot(A,B)
    cb_dot = np.dot(B_inverse,C)

    if(ab_dot > 0 and cb_dot > 0):
        projection = ab_dot/np.linalg.norm(B)
        angle = np.degrees(math.acos(np.dot(B,world_frame)/(np.linalg.norm(B) * np.linalg.norm(world_frame))))
        distance = math.sqrt(math.pow(np.linalg.norm(A),2) - math.pow(projection,2))

        if (P1[1] - P0[1]) >= 0:
            new_point = [projection * math.cos(np.deg2rad(angle)), projection * math.sin(np.deg2rad(angle))] + P0
        else:
            new_point = P0 - [projection * math.cos(np.deg2rad(180 - angle)), projection * math.sin(np.deg2rad(180 - angle))]

        return new_point, True, distance


    else:
        distance_to_P0 = dist = np.linalg.norm(P2-P0)
        distance_to_P1 = dist = np.linalg.norm(P2-P1)

        if distance_to_P0 <= distance_to_P1:
            return P0, False, distance_to_P0
        else:
            return P1, False, distance_to_P1

'''
Grow a simple RRT
'''

def growSimpleRRT(points):

    newPoints = dict()
    adjListMap = dict()
    segment_list = []

    if len(points) < 2:
        if len(points) == 1:
            adjListMap[1] = list()
            return points, adjListMap
        else:
            return points, adjListMap

    #Connect first two points by default
    newPoints[1] = points[1]
    newPoints[2] = points[2]

    adjListMap[1] = [2]
    adjListMap[2] = []

    new_segment = dict()
    new_segment['point1'] = 1
    new_segment['point2'] = 2
    new_segment['line'] = [points[1],points[2]]

    segment_list.append(new_segment)

    for point_index in range(3,len(points) + 1):
        point = points[point_index]

        #Add it to the new_points_list
        index = len(newPoints)+1
        newPoints[index] = point

        adjListMap[index] = []

        closest_distance = float("inf")
        closest_point_index = -1
        final_closest_point = [-1,-1]
        final_is_new = False
        closest_point_1 = []
        closest_point_2 = []

        for segment in segment_list:

            point_1 = segment['point1']
            point_2 = segment['point2']
            line = segment['line']
            closest_point, is_new_point, distance = find_closest_point(line[0], line[1], point)

            closest_index = [-1,-1]

            if np.array_equal(closest_point,line[0]):
                closest_index = point_1
            elif np.array_equal(closest_point,line[1]):
                closest_index = point_2

            if distance < closest_distance:
                closest_distance = distance
                closest_point_index = closest_index
                final_closest_point = closest_point
                final_is_new = is_new_point
                closest_point_1 = point_1
                closest_point_2 = point_2

        if final_is_new:
            new_point_index = len(newPoints)+1
            newPoints[new_point_index] = final_closest_point

            adjListMap[new_point_index] = [index]

            new_segment = dict()
            new_segment['point1'] = new_point_index
            new_segment['point2'] = index
            new_segment['line'] = [newPoints[new_point_index], newPoints[index]]
            segment_list.append(new_segment)

            if closest_point_2 in adjListMap[closest_point_1]:
                adjListMap[closest_point_1].remove(closest_point_2)

                for index, segment in enumerate(segment_list):
                    if segment['point1'] == closest_point_1 and segment['point2'] == closest_point_2:
                        del(segment_list[index])

                adjListMap[closest_point_1].append(new_point_index)
                adjListMap[new_point_index].append(closest_point_2)

                new_segment = dict()
                new_segment['point1'] = closest_point_1
                new_segment['point2'] = new_point_index
                new_segment['line'] = [newPoints[closest_point_1],newPoints[new_point_index]]
                segment_list.append(new_segment)

                new_segment = dict()
                new_segment['point1'] = new_point_index
                new_segment['point2'] = closest_point_2
                new_segment['line'] = [newPoints[new_point_index],newPoints[closest_point_2]]
                segment_list.append(new_segment)

            elif closest_point_1 in adjListMap[closest_point_2]:
                adjListMap[closest_point_2].remove(closest_point_1)
                adjListMap[closest_point_2].append(new_point_index)
                adjListMap[new_point_index].append(closest_point_1)

                for index, segment in enumerate(segment_list):
                    if segment['point1'] == closest_point_2 and segment['point2'] == closest_point_1:
                        del(segment_list[index])

                new_segment = dict()
                new_segment['point1'] = closest_point_2
                new_segment['point2'] = new_point_index
                new_segment['line'] = [newPoints[closest_point_2],newPoints[new_point_index]]
                segment_list.append(new_segment)

                new_segment = dict()
                new_segment['point1'] = new_point_index
                new_segment['point2'] = closest_point_1
                new_segment['line'] = [newPoints[new_point_index],newPoints[closest_point_1]]
                segment_list.append(new_segment)

        else: #Not new point
            adjListMap[closest_point_index].append(index)
            new_segment = dict()
            new_segment['point1'] = closest_point_index
            new_segment['point2'] = index
            new_segment['line'] = [newPoints[closest_point_index],newPoints[index]]
            segment_list.append(new_segment)

    return newPoints, adjListMap

'''
Perform basic search
'''

def basicSearch(tree, start, goal):
    path = []
    queue = []
    closed_list = dict()

    queue.append((start,None))
    while(len(queue) != 0):
        potential_node, parent = queue.pop(0)

        if(potential_node not in closed_list):
            closed_list[potential_node] = parent
            if potential_node == goal:
                break;
            neighbors = tree[potential_node]
            for neighbor in neighbors:
                if neighbor not in closed_list:
                    queue.append((neighbor,potential_node))

    index = goal
    # print(closed_list)
    while True:
        path.insert(0, index)
        index = closed_list[index]
        if index == None:
            break


    return path

'''
Display the RRT and Path
'''

def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):

    print("Path: {}").format(path)

    lines = []
    path_lines = []

    for parent in tree:
        for kid in tree[parent]:

            p_1 = points[parent]
            p_2 = points[kid]
            lines.append([[p_1[0]/10.00, p_1[1]/10.00], [p_2[0]/10.00, p_2[1]/10.00]])

    for i in range(0,len(path)-1):

            p_1 = points[path[i]]
            p_2 = points[path[i+1]]
            path_lines.append([[p_1[0]/10.00, p_1[1]/10.00], [p_2[0]/10.00, p_2[1]/10.00]])

    lc = mc.LineCollection(lines, colors='#212121', linewidths=.5)
    pc = mc.LineCollection(path_lines, colors='#E64A19', linewidths=1)

    fig, ax = setupPlot()

    if polygons != None:
        patch = createPolygonPatch(robotStart, '#66BB6A')
        ax.add_patch(patch)
        patch = createPolygonPatch(robotGoal, '#c62828')
        ax.add_patch(patch)
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'grey')
            ax.add_patch(patch)

    ax.add_collection(lc)
    ax.add_collection(pc)

    plt.show()

    return

'''
Collision checking
'''

def checkIntersect(line1, line2):
    A = np.array([line1[0][0],line1[0][1]])
    B = np.array([line1[1][0],line1[1][1]])
    C = np.array([line2[0][0],line2[0][1]])
    D = np.array([line2[1][0],line2[1][1]])

    CA = A-C
    CD = D-C
    CB = B-C

    AC = C-A
    AD = D-A
    AB = B-A


    cross1 = np.cross(CA,CD).tolist()
    cross2 = np.cross(CB,CD).tolist()
    cross3 = np.cross(AC,AB).tolist()
    cross4 = np.cross(AD,AB).tolist()


    if cross1 == 0 and cross2 == 0 and cross3 == 0 and cross4 == 0:

        if A[0] - B[0] == 0:
            if min(C[1],D[1]) >= min(A[1],B[1]) and min(C[1],D[1]) <= max(A[1],B[1]):
                return True
        elif min(C[0],D[0]) >= min(A[0],B[0]) and min(C[0],D[0]) <= max(A[0],B[0]):
            return True

    if cross1 == 0 or cross2 == 0 or cross3 == 0 or cross4 == 0:
        if A[0] - B[0] == 0:
            m = (D[1] - C[1])/(D[0] - C[0])
            b = C[1] - m*C[0]
            yC = C[0] * m + b
            yD = D[0] * m + b
            if yC == C[1] and yC >= min(A[1],B[1]) and yC <= max(A[1],B[1]) or yD == D[1] and yD >= min(A[1],B[1]) and yD <= max(A[1],B[1]):
                return True
        elif C[0] - D[0] == 0:
            m = (B[1] - A[1])/(B[0] - A[0])
            b = A[1] - m*A[0]
            yA = A[0]*m + b
            yB = B[0]*m + b
            print(A[1])
            if yA == A[1]  and yA >= min(C[1],D[1]) and yA <= max(C[1],D[1]) or yB == B[1] >= min(C[1],D[1]) and yB <= max(C[1],D[1]):
                return True
        elif min(C[0],D[0]) >= min(A[0],B[0]) and min(C[0],D[0]) <= max(A[0],B[0]) or max(C[0],D[0]) >= min(A[0],B[0]) and max(C[0],D[0]) <= max(A[0],B[0]):
            return True


    if (np.cross(CA,CD).tolist() * np.cross(CB,CD).tolist()) <= 0:
        if (np.cross(AC,AB).tolist() * np.cross(AD,AB).tolist()) <= 0:
            return True

    return False

def does_robot_collide(segment, robot, obstacles):
    p1,p2 = segment['line']


    checkList = []
    for robot_point in robot:
        robot_offset = [(robot_point[0] - robot[0][0]),(robot_point[1] - robot[0][1])]
        checkList.append([[p1[0] + robot_offset[0], p1[1] + robot_offset[1]],[p2[0] + robot_offset[0], p2[1] + robot_offset[1]]])

    obstList = []
    for obstacle in obstacles:
        for index,pt1 in enumerate(obstacle):
            nextindex = (index+1)%len(obstacle)
            obstList.append([[pt1[0],pt1[1]],[obstacle[nextindex][0],obstacle[nextindex][1]]])
    obstList.append([[0,0],[0,10]])
    obstList.append([[0,10],[10,10]])
    obstList.append([[10,10],[10,0]])
    obstList.append([[0,0],[10,0]])


    for robot_line in checkList:
        for edge in obstList:

            if checkIntersect(robot_line,edge):
                return True

    return False


def isCollisionFree(robot, point, obstacles):
    obstList = []

    for rpoint in robot:
        if point_is_in_obstacle((point[0] + rpoint[0], point[1] + rpoint[1]),obstacles):
            return False

    for obstacle in obstacles:
        for index,pt1 in enumerate(obstacle):
            nextindex = (index+1)%len(obstacle)
            obstList.append([[pt1[0],pt1[1]],[obstacle[nextindex][0],obstacle[nextindex][1]]])
    obstList.append([[0,0],[0,10]])
    obstList.append([[0,10],[10,10]])
    obstList.append([[10,10],[10,0]])
    obstList.append([[0,0],[10,0]])


    for i,p in enumerate(robot):
        nexti = (i+1) % len(robot)
        roboedge = [[point[0] + p[0],point[1] + p[1]],[point[0] +robot[nexti][0],point[1] + robot[nexti][1]]]
        for edge in obstList:
            if checkIntersect(roboedge,edge):
                return False
    return True


def point_is_in_obstacle(point, obstacles):

    P0 = point
    P1 = [10,point[1]]
    ray = [[P0[0],P0[1]],[P1[0],P1[1]]]

    counter = 0

    for obstacle in obstacles:
        for index,pt1 in enumerate(obstacle):
            nextindex = (index+1)%len(obstacle)
            segment = [[pt1[0],pt1[1]],[obstacle[nextindex][0],obstacle[nextindex][1]]]
            if checkIntersect(segment, ray):
                print segment
                counter = counter + 1

    print counter
    if (counter % 2) != 0:
        return True
    else:
        return False

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    newPoints = dict()
    adjListMap = dict()
    segment_list = []
    path = []
    start_index = 1
    newPoints[start_index] = startPoint
    new_segment = dict()
    new_segment['point1'] = 1
    new_segment['point2'] = 2
    new_segment['line'] = [startPoint, goalPoint]

    if not does_robot_collide(new_segment, robot, obstacles):
        newPoints[2] = goalPoint
        adjListMap[1] = [2]
        return newPoints, adjListMap, [1,2]
    newPoints[2] = (random.uniform(0, 10), random.uniform(0, 10))

    adjListMap[1] = [2]
    adjListMap[2] = []

    new_segment = dict()
    new_segment['point1'] = 1
    new_segment['point2'] = 2
    new_segment['line'] = [newPoints[1], newPoints[2]]

    while does_robot_collide(new_segment, robot, obstacles):
        newPoints[2] = (random.uniform(0, 10),random.uniform(0, 10))
        new_segment['line'] = [newPoints[1],newPoints[2]]

    segment_list.append(new_segment)

    while True:

        point_x = random.uniform(0, 10)
        point_y = random.uniform(0, 10)

        if point_is_in_obstacle((point_x,point_y), obstacles):
            continue

        index = len(newPoints)+1
        point = (point_x, point_y)
        newPoints[index] = point
        adjListMap[index] = []

        print("Generated {} points").format(len(newPoints))

        closest_distance = float("inf")
        closest_point_index = -1
        final_closest_point = [-1,-1]
        final_is_new = False
        closest_point_1 = None
        closest_point_2 = None

        for segment in segment_list:

            point_1 = segment['point1']
            point_2 = segment['point2']
            line = segment['line']
            closest_point, is_new_point, distance = find_closest_point(line[0], line[1], point)

            closest_index = [-1,-1]

            if np.array_equal(closest_point,line[0]):
                closest_index = point_1
            elif np.array_equal(closest_point,line[1]):
                closest_index = point_2

            if distance < closest_distance:
                closest_distance = distance
                closest_point_index = closest_index
                final_closest_point = closest_point
                final_is_new = is_new_point
                closest_point_1 = point_1
                closest_point_2 = point_2

        if final_is_new:
            new_point_index = len(newPoints)+1
            newPoints[new_point_index] = final_closest_point
            adjListMap[new_point_index] = [index]

            new_segment = dict()
            new_segment['point1'] = new_point_index
            new_segment['point2'] = index
            new_segment['line'] = [newPoints[new_point_index], newPoints[index]]

            if does_robot_collide(new_segment, robot, obstacles):

                del newPoints[new_point_index]
                del adjListMap[new_point_index]
                continue
            segment_list.append(new_segment)

            if closest_point_1 == None:
                continue

            if closest_point_2 in adjListMap[closest_point_1]:
                adjListMap[closest_point_1].remove(closest_point_2)

                for index, segment in enumerate(segment_list):
                    if segment['point1'] == closest_point_1 and segment['point2'] == closest_point_2:
                        del(segment_list[index])

                adjListMap[closest_point_1].append(new_point_index)
                adjListMap[new_point_index].append(closest_point_2)

                new_segment = dict()
                new_segment['point1'] = closest_point_1
                new_segment['point2'] = new_point_index
                new_segment['line'] = [newPoints[closest_point_1],newPoints[new_point_index]]
                segment_list.append(new_segment)

                new_segment = dict()
                new_segment['point1'] = new_point_index
                new_segment['point2'] = closest_point_2
                new_segment['line'] = [newPoints[new_point_index],newPoints[closest_point_2]]
                segment_list.append(new_segment)

            elif closest_point_1 in adjListMap[closest_point_2]:
                adjListMap[closest_point_2].remove(closest_point_1)
                adjListMap[closest_point_2].append(new_point_index)
                adjListMap[new_point_index].append(closest_point_1)

                for index, segment in enumerate(segment_list):
                    if segment['point1'] == closest_point_2 and segment['point2'] == closest_point_1:
                        del(segment_list[index])

                new_segment = dict()
                new_segment['point1'] = closest_point_2
                new_segment['point2'] = new_point_index
                new_segment['line'] = [newPoints[closest_point_2],newPoints[new_point_index]]
                segment_list.append(new_segment)

                new_segment = dict()
                new_segment['point1'] = new_point_index
                new_segment['point2'] = closest_point_1
                new_segment['line'] = [newPoints[new_point_index],newPoints[closest_point_1]]
                segment_list.append(new_segment)

        else:
            adjListMap[closest_point_index].append(index)
            new_segment = dict()
            new_segment['point1'] = closest_point_index
            new_segment['point2'] = index
            new_segment['line'] = [newPoints[closest_point_index],newPoints[index]]

            if does_robot_collide(new_segment, robot, obstacles):
                adjListMap[closest_point_index].remove(index)
                continue

            segment_list.append(new_segment)
            #adjListMap[closest_point_index].append(index)

        connects, new_newPoints, new_adjListMap, new_segment_list = try_to_connect_goal(goalPoint, newPoints, adjListMap, segment_list)

        if connects:
            newPoints = new_newPoints
            adjListMap = new_adjListMap
            segment_list = new_segment_list
            break

    goal_index = -1
    for i in range(1,len(newPoints)):
        x = len(newPoints)+1 - i
        if newPoints[x][0] == goalPoint[0] and newPoints[x][1] == goalPoint[1]:
            goal_index = x
            break

    print("\n\n\n")
    print(adjListMap)
    print(start_index)
    print(goal_index)
    print(newPoints)

    path = basicSearch(adjListMap, start_index, goal_index)

    return newPoints, adjListMap, path


def try_to_connect_goal(goalPoint, newPoints, adjListMap, segment_list):
    closest_distance = float("inf")
    closest_point_index = -1
    final_closest_point = [-1,-1]
    final_is_new = False
    closest_point_1 = None
    closest_point_2 = None
    index = len(newPoints) + 1
    adjListMap[index] = []
    newPoints[index] = goalPoint

    for segment in segment_list:
        point_1 = segment['point1']
        point_2 = segment['point2']
        line = segment['line']
        closest_point, is_new_point, distance = find_closest_point(line[0], line[1], goalPoint)

        closest_index = [-1,-1]

        if np.array_equal(closest_point,line[0]):
            closest_index = point_1
        elif np.array_equal(closest_point,line[1]):
            closest_index = point_2

        if distance < closest_distance:
            closest_distance = distance
            closest_point_index = closest_index
            final_closest_point = closest_point
            final_is_new = is_new_point
            closest_point_1 = point_1
            closest_point_2 = point_2

    if final_is_new:
        new_point_index = len(newPoints)+1
        newPoints[new_point_index] = final_closest_point
        adjListMap[new_point_index] = [index]

        new_segment = dict()
        new_segment['point1'] = new_point_index
        new_segment['point2'] = index
        new_segment['line'] = [newPoints[new_point_index], newPoints[index]]

        if does_robot_collide(new_segment, robot, obstacles):
            del newPoints[new_point_index]
            del adjListMap[new_point_index]
            return False, None, None, None

        segment_list.append(new_segment)

        if closest_point_1 == None:
            return False, None, None, None

        if closest_point_2 in adjListMap[closest_point_1]:
            adjListMap[closest_point_1].remove(closest_point_2)

            for index, segment in enumerate(segment_list):
                if segment['point1'] == closest_point_1 and segment['point2'] == closest_point_2:
                    del(segment_list[index])

            adjListMap[closest_point_1].append(new_point_index)
            adjListMap[new_point_index].append(closest_point_2)

            new_segment = dict()
            new_segment['point1'] = closest_point_1
            new_segment['point2'] = new_point_index
            new_segment['line'] = [newPoints[closest_point_1],newPoints[new_point_index]]
            segment_list.append(new_segment)

            new_segment = dict()
            new_segment['point1'] = new_point_index
            new_segment['point2'] = closest_point_2
            new_segment['line'] = [newPoints[new_point_index],newPoints[closest_point_2]]
            segment_list.append(new_segment)

        elif closest_point_1 in adjListMap[closest_point_2]:
            adjListMap[closest_point_2].remove(closest_point_1)
            adjListMap[closest_point_2].append(new_point_index)
            adjListMap[new_point_index].append(closest_point_1)

            for index, segment in enumerate(segment_list):
                if segment['point1'] == closest_point_2 and segment['point2'] == closest_point_1:
                    del(segment_list[index])

            new_segment = dict()
            new_segment['point1'] = closest_point_2
            new_segment['point2'] = new_point_index
            new_segment['line'] = [newPoints[closest_point_2],newPoints[new_point_index]]
            segment_list.append(new_segment)

            new_segment = dict()
            new_segment['point1'] = new_point_index
            new_segment['point2'] = closest_point_1
            new_segment['line'] = [newPoints[new_point_index],newPoints[closest_point_1]]
            segment_list.append(new_segment)

    else: #Not new point
        adjListMap[closest_point_index].append(index)
        new_segment = dict()
        new_segment['point1'] = closest_point_index
        new_segment['point2'] = index
        new_segment['line'] = [newPoints[closest_point_index],newPoints[index]]

        if does_robot_collide(new_segment, robot, obstacles):
            #Remove from new points
            adjListMap[closest_point_index].remove(index)
            return False, None, None, None #Go to the next point, and don't do anything with this one

        segment_list.append(new_segment)

    return True, newPoints, adjListMap, segment_list

if __name__ == "__main__":

    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print ""
    print "The input points are:"
    print str(points)
    print ""

    points, adjListMap = growSimpleRRT(points)

    path = basicSearch(adjListMap, 1, len(points))

    displayRRTandPath(points, adjListMap, path)

    new_points, super_tree, the_path = RRT(robot, obstacles, (x1, y1), (x2, y2))

    displayRRTandPath(new_points, super_tree, the_path, robotStart, robotGoal, obstacles)
