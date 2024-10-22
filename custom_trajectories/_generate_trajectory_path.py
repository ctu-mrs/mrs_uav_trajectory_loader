import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys 
import os

def plot_waypoints(waypoints:np.array):
    # Extract coordinates for path
    x_coords = waypoints[:, 0]
    y_coords = waypoints[:, 1]
    z_coords = waypoints[:, 2]
    yaws = waypoints[:, 3]  # Yaw (optional, for additional visualization if needed)  

    # Create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the waypoints
    ax.scatter(x_coords[0], y_coords[0], z_coords[0], color="g", marker = "o")
    ax.scatter(x_coords[1:], y_coords[1:], z_coords[1:], c='r', marker='o')

    # Plot the path (lines connecting the waypoints)
    ax.plot(x_coords, y_coords, z_coords, label='Path')
    # Optionally, visualize yaw at each waypoint
    for i in range(len(waypoints)):
        ax.quiver(x_coords[i], y_coords[i], z_coords[i], 
                  np.cos(yaws[i]), np.sin(yaws[i]), 0, 
                  length=1, color='b', label='Yaw' if i == 0 else "")  # Add arrows to represent yaw

    # Labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Path Visualization through Waypoints')
    ax.set_xlim(np.min(x_coords-1),np.max(x_coords+1))
    ax.set_ylim(np.min(y_coords-1),np.max(y_coords+1))
    ax.set_zlim(0,max(z_coords)*3/2)
    # Show plot
    plt.legend()
    plt.show()


def generate_sinoid_trajectory(freq = 1,amplitude=1):
    num_points = int(Y_DISTANCE / (T * V_LIMIT) * freq)
    waypoints = list()
    for point in np.linspace(0,Y_DISTANCE,num_points):
        waypoints.append([amplitude*np.sin(point*freq),point,5,np.pi/2])
    return np.array(waypoints)


def generate_circular_trajectory(radius=4):
    # origin [0,0]
    num_points = int(2*np.pi*radius / (T*V_LIMIT))
    deg_per_point = T * V_LIMIT / radius
    flyier = list()
    observer = list()
    for point in range(num_points):
        angle = point*deg_per_point
        x = radius*np.cos(angle)
        y = radius*np.sin(angle)
        z = 5
        roll = np.pi/2 + angle
        flyier.append([x,y,z,roll])
        observer.append([0,0,z,angle-deg_per_point])
    flyier.append(flyier[0])
    observer.append(observer[0])

    return np.array([flyier,observer])



def interpolate_waypoints(waypoints, dynamic_interpolation=False, interpolate_roll=False, to_loop=False):
    """
    Function to interpolate waypoints with decreasing distance between points as you approach the next waypoint.
    The maximum distance between consecutive points is V_LIMIT * T.

    Args:
        waypoints (np.ndarray): Original waypoints as a 2D array [x, y, z, yaw].
        V_LIMIT (float): Velocity limit.
        T (float): Time interval.
        interpolate_yaw (bool): Whether to interpolate yaw or keep it constant between points.

    Returns:
        np.ndarray: New interpolated waypoints.
    """
    x_coords = waypoints[:, 0]
    y_coords = waypoints[:, 1]
    z_coords = waypoints[:, 2]
    rolls = waypoints[:, 3]

    interp_x = []
    interp_y = []
    interp_z = []
    interp_roll = []

    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dist = np.linalg.norm(p2[:3] - p1[:3])

        if dynamic_interpolation:
            power = 4
        else:
            power = 1

        num_points = int(np.ceil(dist / (V_LIMIT * T))) * power

        spacing = np.linspace(0, 1, num_points, endpoint=False)
        spacing = 1 - (1 - spacing) ** power
        
        interp_x.extend(p1[0] + (p2[0] - p1[0]) * spacing)
        interp_y.extend(p1[1] + (p2[1] - p1[1]) * spacing)
        interp_z.extend(p1[2] + (p2[2] - p1[2]) * spacing)

        if interpolate_roll:
            interp_roll.extend(p1[3] + (p2[3] - p1[3]) * spacing)
        else:
            interp_roll.extend(np.full(num_points, p1[3]))

    interp_x.append(x_coords[-1])
    interp_y.append(y_coords[-1])
    interp_z.append(z_coords[-1])
    interp_roll.append(rolls[-1])

    # Add start position to make looping possible
    if to_loop:
        interp_x.append(x_coords[0])
        interp_y.append(y_coords[0])
        interp_z.append(z_coords[0])
        interp_roll.append(rolls[0])


    return np.column_stack((interp_x, interp_y, interp_z, interp_roll))

def save_waypoints(trajectory_type:str,waypoints:np.array):
    with open(trajectory_type+'.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        for row in waypoints:
            writer.writerow(np.round(row,3))



def generate_waypoints(trajectory_type = "side_to_side") -> np.array:


    #########################################################################################
    # SIDE TO SIDE
    side_to_side = [[ X_DISTANCE * (2*(i%2)-1), 0, 5, np.pi/2 - ROTATE*np.pi*(i%2)] for i in range(NUM_WAYPOINTS)]
    side_to_side = np.array([[ 0, 0, 5, np.pi/2]] + side_to_side)
    
    #########################################################################################
    #########################################################################################
    # SQUARE

    square = np.array(
        [
            [ SQUARE_SIZE, SQUARE_SIZE,5, 5*np.pi/4],
            [ SQUARE_SIZE,-SQUARE_SIZE,5, 3*np.pi/4],
            [-SQUARE_SIZE,-SQUARE_SIZE,5, np.pi/4],
            [-SQUARE_SIZE, SQUARE_SIZE,5, -np.pi/4],
        ]
    )
    #########################################################################################
    #########################################################################################
    # SQUARE UP
    square_up = np.array(
        [
            [ SQUARE_SIZE, SQUARE_SIZE,            5,  5*np.pi/4],
            [ SQUARE_SIZE,           0, 5+Z_MOVEMENT,      np.pi],
            [ SQUARE_SIZE,-SQUARE_SIZE,            5,  3*np.pi/4],
            [           0,-SQUARE_SIZE, 5+Z_MOVEMENT,    np.pi/2],
            [-SQUARE_SIZE,-SQUARE_SIZE,            5,    np.pi/4],
            [-SQUARE_SIZE,           0, 5+Z_MOVEMENT,          0],
            [-SQUARE_SIZE, SQUARE_SIZE,            5,   -np.pi/4],
            [           0, SQUARE_SIZE, 5+Z_MOVEMENT,   -np.pi/2],
        ]
    )
    #########################################################################################
    #########################################################################################
    # SQUARE DOWN

    square_down = np.array(
        [
            [ SQUARE_SIZE, SQUARE_SIZE,            5,  5*np.pi/4],
            [ SQUARE_SIZE,           0, 5-Z_MOVEMENT,      np.pi],
            [ SQUARE_SIZE,-SQUARE_SIZE,            5,  3*np.pi/4],
            [           0,-SQUARE_SIZE, 5-Z_MOVEMENT,    np.pi/2],
            [-SQUARE_SIZE,-SQUARE_SIZE,            5,    np.pi/4],
            [-SQUARE_SIZE,           0, 5-Z_MOVEMENT,          0],
            [-SQUARE_SIZE, SQUARE_SIZE,            5,   -np.pi/4],
            [           0, SQUARE_SIZE, 5-Z_MOVEMENT,   -np.pi/2],
        ]
    )
    #########################################################################################
    #########################################################################################
    # LINE FOLLOW
    line_follow = np.array([
        [0, 0         , 5, np.pi/2],
        [0, Y_DISTANCE, 5, np.pi/2],
    ])

    #########################################################################################
    #########################################################################################
    # ZIG ZAG

    y_coords = np.linspace(0, Y_DISTANCE, NUM_WAYPOINTS)
    x_coords = [(2*(i % 2))-1 for i in range(len(y_coords)-1)]
    x_coords = np.array([0] + x_coords)*X_DISTANCE
    z_coords = [5]*len(y_coords)
    roll = [np.pi/2]*len(y_coords)

    zig_zag = np.column_stack((x_coords,y_coords,z_coords,roll))

    #########################################################################################
    #########################################################################################
    # RANDOM ZIG ZAG


    y_coords = np.linspace(0, Y_DISTANCE, NUM_WAYPOINTS)
    x_coords = [np.random.rand()*100 % X_DISTANCE*2 - X_DISTANCE for _ in range(len(y_coords)-1)]
    x_coords = [0] + x_coords
    z_coords = [5]*len(y_coords)
    roll = [np.pi/2]*len(y_coords)

    random_zig_zag = np.column_stack((x_coords,y_coords,z_coords,roll))

    #########################################################################################
    #########################################################################################
    # HEXAGON
    hexagon = np.array([
        [0,0,5,np.pi/2],
        [X_DISTANCE/2,X_DISTANCE,5,np.pi/2],
        [-X_DISTANCE/2,X_DISTANCE/2,5,np.pi/2],
        [0,3*X_DISTANCE/2,5,np.pi/2],
        [X_DISTANCE/2,X_DISTANCE/2,5,np.pi/2],
        [-X_DISTANCE/2,X_DISTANCE,5,np.pi/2],
        [0,0,5,np.pi/2],
    ])

    #########################################################################################
    #########################################################################################
    # OCTAGON
    

    octagon = np.array([
        [-X_DISTANCE/2,0,5,np.pi],
        [X_DISTANCE/2,X_DISTANCE,5,np.pi],
        [-X_DISTANCE,X_DISTANCE/4,5,np.pi],
        [X_DISTANCE,X_DISTANCE*3/4,5,np.pi],
        [X_DISTANCE/2,0,5,np.pi],
        [-X_DISTANCE,X_DISTANCE*3/4,5,np.pi],
        [X_DISTANCE,X_DISTANCE/4,5,np.pi],
        [-X_DISTANCE/2,X_DISTANCE,5,np.pi],
        [-X_DISTANCE/2,0,5,np.pi],
    ])



    #########################################################################################
    #########################################################################################
    # RETURN SWITCH
    if trajectory_type == "side_to_side":
        return side_to_side
    elif trajectory_type == "square":
        return square
    elif trajectory_type == "square_up":
        return square_up
    elif trajectory_type == "square_down":
        return square_down
    elif trajectory_type == "line_follow":
        return line_follow
    elif trajectory_type == "zig_zag":
        return zig_zag
    elif trajectory_type == "random_zig_zag":
        return random_zig_zag
    elif trajectory_type == "hexagon":
        return hexagon
    elif trajectory_type == "octagon":
        return octagon
    else:
        print("Unknown trajectory")
        exit()




if  __name__ == "__main__":
    types = {
        0 : ("side_to_side", True),
        1 : ("square", True),
        2 : ("square_up", True),
        3 : ("square_down", True),
        4 : ("line_follow", False),
        5 : ("zig_zag", False),
        6 : ("random_zig_zag", False),
        7 : ("sinus",False),
        8 : ("circle",True),
        9 : ("hexagon",True),
        10: ("octagon",True),
    }
    if len(sys.argv) != 2 or not sys.argv[1].isnumeric():
        print("Incorrect arguments")
        print("Usage: python <file_name>.py X")
        print("X is positive integer from 0 to",len(types)-1)
        exit()
    trajectory_type,loop = types[int(sys.argv[1])]

    T = 0.2
    V_LIMIT = 3.5 #5 for circle, 3.5 otherwise
    ROTATE = 0 # could be quite random 1=True, 0 = False

    X_DISTANCE = 5  # side movement 
    Y_DISTANCE = 20 # flight distance
    NUM_WAYPOINTS = 60 # if the path cannot be looped
    SQUARE_SIZE = 4 # square trajectory
    Z_MOVEMENT = 1  # up & down square

    if trajectory_type == "circle":
        outer, inner = generate_circular_trajectory(X_DISTANCE*2)
        plot_waypoints(inner)
        plot_waypoints(outer)
        save_waypoints("circle_observer",inner)
        save_waypoints("circle_flier",outer)

    elif trajectory_type == "sinus":
        path = generate_sinoid_trajectory(freq=1, amplitude=X_DISTANCE)
        plot_waypoints(path)
        save_waypoints("sinus",interpolate_waypoints(path))


    else:
        WAYPOINTS = generate_waypoints(trajectory_type)
        inter_waypoints = interpolate_waypoints(WAYPOINTS,dynamic_interpolation=False, interpolate_roll=True,to_loop=loop)
        plot_waypoints(inter_waypoints)
        save_waypoints(trajectory_type,inter_waypoints)
  
