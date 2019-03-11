## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README [Rubrics point : Writeup]

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code [Rubrics point : Explain starter code]

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

1. Get global home, global position, and local position
```
print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
```
2. Load obstavles location
```
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
```
3. Create grid based on altitude, safe margin and obstacles using planning_utils.py
```
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
```
4. Fing start and goal point
```
grid_start = (-north_offset, -east_offset)
grid_goal = (-north_offset + 10, -east_offset + 10)
```
5. From planning_utils.py use A* method to get the path
```
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
```
6. Convert path to intermediate waypoints
```
waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
self.waypoints = waypoints
```
7. Show in simulator
```
self.send_waypoints()
```


### Implementing Your Path Planning Algorithm [Rubrics point : Impliment code]

#### 1. Set your global home position [RP-IC: 01]
Read colliders.csv file using function file_data_input defined as 
```
def file_data_input(*args, **kwargs):
    if kwargs['method'] == 'csv':
        with open(args[0], 'r') as f:
            reader = csv.reader(f)
            data_as_list=list(reader)
            
    if kwargs['method'] == 'np':
        if 'dtype' in kwargs and 'skiprows' in kwargs:
            data_as_list=np.loadtxt(args[0], delimiter=',',dtype=kwargs['dtype'],skipwors=kwargs['skiprows'])
        else:
            data_as_list=np.loadtxt(args[0], delimiter=',')
    return data_as_list
```
Take first row first column as lat_zero and split to get lat0. similarly get lon0. Assign lat0,lon0 to global home variable
```
        # TODO: read lat0, lon0 from colliders into floating point values
        ##===Abu code start===##
        args=['colliders.csv']
        kwargs={'method':'csv'}
        start_lat_lon=file_data_input(*args, **kwargs)
        lat_zero=start_lat_lon[0][0].split()
        lat0=float(lat_zero[1])        
        lon_zero=start_lat_lon[0][1].split()
        lon0=float(lon_zero[1])
        #print(lat0,lon0)
        ##===Abu code end===##
        
        # TODO: set home position to (lon0, lat0, 0)
        ##===Abu code start===##
        self.global_home[0]=lon0
        self.global_home[1]=lat0
        self.global_home[2]=0.0
        ##===Abu code end===##
```
#### 2. Set your current local position [RP-IC: 02]
```
        # TODO: retrieve current global position
        ##===Abu code start===##
        global_position_lon=self.global_position[0]
        global_position_lat=self.global_position[1]
        global_position_alt=self.global_position[2]
        ##===Abu code end===##
 
        # TODO: convert to current local position using global_to_local()
        ##===Abu code start===##
        current_local_position=[]
        current_local_position=global_to_local(self.global_position,self.global_home)
        ##===Abu code end===##

```

#### 3. Set grid start position from local position [RP-IC: 03]
```
        # TODO: convert start position to current position rather than map center
        ##===Abu code start===##
        current_position_north=int(current_local_position[0])
        current_position_east=int(current_local_position[1])
        print('Abu current position',current_position_north,type(current_position_north))
        grid_start = (current_position_north-north_offset, current_position_east-east_offset)
        print('Abu Local Start: ', grid_start)
        ##===Abu code end===##
```

#### 4. Set grid goal position from geodetic coords [RP-IC: 04]
```
        # Set goal as some arbitrary position on the grid        
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        ##===Abu code start===##
        goal_position_global=[-122.4,37.8,0]
        goal_position_local=global_to_local(goal_position_global,self.global_home)
        goal_north=int(goal_position_local[0])
        goal_east=int(goal_position_local[1])
        grid_goal = (goal_north-north_offset, goal_east-east_offset)
        print('Abu Local Goal: ', grid_goal)
        ##===Abu code end===##
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether) [RP-IC: 05]
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

Diagonal motion
```
    ##===Abu code start===##
    NE = (-1, 1,np.sqrt(2))
    SE = ( 1, 1,np.sqrt(2))
    SW = ( 1,-1,np.sqrt(2))
    NW = (-1,-1,np.sqrt(2))
    ##===Abu code end===##
```
Check if diagonal motion in collition
```
    ##===Abu code start===##
    '''
    NE = (-1, 1,np.sqrt(2))
    SE = ( 1, 1,np.sqrt(2))
    SW = ( 1,-1,np.sqrt(2))
    NW = (-1,-1,np.sqrt(2))
    '''
    if (x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NE)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SE)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SW)
    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NW)
    ##===Abu code end===##
```

#### 6. Cull waypoints  [RP-IC: 06]
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.
```
def collinearity_check(point1,point2,point3,epsilon=1.0e-2):
    mat=np.concatenate((point1,point2,point3),0)
    det_mat=np.linalg.det(mat)
    return abs(det_mat)<epsilon
```
Using matrix determinant to check if points are collinear
```
def reshape_point(point):
    return np.array([point[0],point[1],1.0]).reshape(1,-1)
    
def prune_path(path):
    pruned_path=[points for points in path]
    i=0
    while i<len(pruned_path)-2:
```
    Get three consiquitive point to see if they are Collinear?
```
        point1=reshape_point(pruned_path[i])
        point2=reshape_point(pruned_path[i+1])
        point3=reshape_point(pruned_path[i+2])

        if collinearity_check(point1,point2,point3):
            pruned_path.remove(pruned_path[i+1])
```
            If collinear, remove them from Path
```
        else:
            i+=1
    return pruned_path
```



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


