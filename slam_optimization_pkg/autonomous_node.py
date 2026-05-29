import numpy as np
import rclpy
import math
import time
from random import randrange
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Autopilot(Node):

    def __init__(self):
        super().__init__('autopilot')
        """Initialize autopilot parameters:

        potential_pos (OccupancyGrid): Occupancy grid index associated with next potential position

        occupancy_grid (OccupancyGrid): Current occupancy grid information received from '/map'. References callback function 'next_waypoint'
        
        """
        # Allow callback functions to be called in parallel
        #self.parallel_callback_group = ReentrantCallbackGroup()

        #counter for new strategies when map is fully searched
        self.new_strat_counter = 0

        #Initializing variable for tracking if map is fully mapped
        self.fully_mapped = False

        # Initilizing the probablity at which we consider there to be an obstacle
        self.obstacle_probability = 75

        # Specifies how many incoming messages should be buffered
        self.queue_size = 10

        # Flag variable that indicates that the exploration is just started
        self.start=True

        # Initialize the number of waypoints published
        self.waypoint_counter = 0

        self.goal_updated_counter=0

        # Initialize the number of waypoints published with the new strategy
        self.new_strategy_counter = 0

        # Initiliazing distance to localise to ArUco marker
        self.desired_distance=3.0


        self.aruco_detected = False
        self.localisation_started = False

        # Initialize the current grid
        self.current_grid = OccupancyGrid()

        self.pose_to_aruco = PoseStamped()

        # Initializing x and y coordinates of Turtlebot in space, to be populated later
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0.0
        self.new_waypoint.pose.position.y = 0.0
        self.new_waypoint.pose.orientation.w = 1.0

        # Initializing potential_coordinate for debugging
        self.potential_coordinate = PointStamped()
        self.potential_coordinate.header.frame_id = 'map'
        self.potential_coordinate.point.x = 0.0
        self.potential_coordinate.point.y = 0.0

        #Initializing current position variable
        self.current_position = PoseStamped()
        self.current_position.header.frame_id = 'map'

        #Initializing number of iterations before the strategy is changed
        self.strategy_counter = 10

        #Subscribe to /behavior_tree_log to determine when Turtlebot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.readiness_check,
            self.queue_size,
            #callback_group=self.parallel_callback_group
        )

        #Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.store_grid,
            self.queue_size
        )

        #Subscribe to /pose to determine position of Turtlebot
        self.position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.current_position_callback,
            self.queue_size,
            #callback_group=self.parallel_callback_group
        )

        self.aruco_map_position_sub = self.create_subscription(
            PointStamped,
            'aruco_map_position',
            self.aruco_map_position_callback,
            1
        ) 

        #Create publisher to publish next waypoint parameters to
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            self.queue_size
        )

        #Publisher for publishing potential waypoints to for bug fixing
        self.potential_publisher = self.create_publisher(
            PointStamped,
            'potential_point',
            self.queue_size
        )

        #Publisher for publishing current coordinate for bug fixing
        self.current_publisher = self.create_publisher(
            PointStamped,
            'current_point',
            self.queue_size
        )





    
    
    def store_grid(self,grid:OccupancyGrid):
        """ 
            Callback function of /map topic.
            Everytime it receives the OccupacyGrid message it stores it.
            At the start it launches the next_point() method since no message is received from the behavior_tree_log.
        """
        self.current_grid=grid

        #Initiates looking for new waypoint if exploration has just started.
        #This is because readiness_check will not do this when exploration has just started
        if self.start:
            self.next_waypoint()
            self.start = False
            
            

    def next_waypoint(self):
        """
        Function to choose next waypoint when new occupancy grid is received, and old goal is either destroyed or achieved

        Args:
        self (Node): Autopilot node currently running and storing waypoint decisions 
        """

        self.width = self.current_grid.info.width
        isthisagoodwaypoint = False

        not_in_range_count=0
        points_checked = 0


        min_distance = 1
        max_distance = 3

        still_looking = False
        occupancy_data_np = np.array(self.current_grid.data)

        if self.strategy_counter > 0:

            while not isthisagoodwaypoint:
                
                points_checked += 1


                if self.new_strat_counter > 5:
                    self.get_logger().info('Could not find point after 5 new strategy searches, map is likely fully resolved, retracing...')
                    time.sleep(2)
                    self.fully_mapped = True
                    self.new_strat_counter = 0
                if points_checked > 10000:
                    self.get_logger().info('Maximum number of iterations exceeded, adopting new strategy...')
                    time.sleep(2)
                    self.new_strategy()
                    self.new_strat_counter += 1
                    isthisagoodwaypoint = True
                    break


                # Added this so that the terminal isn't filled with messages so it's easier to read
                if not still_looking:
                    self.get_logger().info('Searching for good point...')
                    still_looking = True

                # Taking a random cell and the corresponding cost value
                random_index = randrange(occupancy_data_np.size)
                self.potential_pos = occupancy_data_np[random_index]

                # Check that the cell is not unknown
                if self.potential_pos == -1:
                    continue

                # Compute correspondent row and column of the potential cell 
                [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(random_index)
                
                # Publish current position and potential position for visualization
                self.potential_publisher.publish(self.potential_coordinate)
        

                # Check that the cell is not an obstacle and is on the frontier
                if self.potential_pos>= self.obstacle_probability or not self.frontier_check(occupancy_data_np, random_index):
                    continue
                else:
                    self.get_logger().info('Found Good Point')
                    self.get_logger().info('Checking Point Distance')

                    distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.pose.position.x)**2 +
                        (self.potential_coordinate.point.y- self.current_position.pose.position.y)**2
                    )


                    if min_distance < distance2new < max_distance or self.start:
                        self.new_waypoint.pose.position.x = self.potential_coordinate.point.x
                        self.new_waypoint.pose.position.y = self.potential_coordinate.point.y
                        self.get_logger().info('Point Distance:' + str(distance2new))
                
                        isthisagoodwaypoint = True

                        self.strategy_counter -= 1
                        self.get_logger().info("Remaining points before new strategy:" + str(self.strategy_counter))

                    else:
                        self.get_logger().info('Point not in range, Finding New...')
                        isthisagoodwaypoint = False
                        still_looking = False

                        # If the point is not in range for 30 iterations, adopt a new strategy
                        not_in_range_count += 1
                        if not_in_range_count > 50:
                            self.get_logger().info('Could not find point in range, adopting new strategy...')
                            self.new_strategy()
                            isthisagoodwaypoint = True
                
                        
        #New strategy
        else:
            self.strategy_counter = 5
            self.new_strategy()
           
        #Publish the new waypoint
        self.get_logger().info('Publishing waypoint...')
        self.waypoint_publisher.publish(self.new_waypoint)
        self.waypoint_counter += 1

    def new_strategy(self):
        """Processes the occupancy grid and creates a sorted list of cells based on the number of uncertain cells around them."""

        self.get_logger().info('New Strategy: Processing occupancy grid ...')
        occupancy_data_np = np.array(self.current_grid.data)
        width = self.current_grid.info.width
        height = self.current_grid.info.height
        counts_list = []
        # Iterate over all cells in the occupancy grid
        for index in range(len(occupancy_data_np)) :
            
            if occupancy_data_np[index] == -1 or occupancy_data_np[index] >= self.obstacle_probability:
                continue

            [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(index)

            distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.pose.position.x)**2 +
                        (self.potential_coordinate.point.y- self.current_position.pose.position.y)**2
                    ) 

            # Check if the distance is greater than 9 meters and counter is not a multiple of four
            if distance2new > 5 and self.new_strategy_counter % 4 != 0:
                continue
            else: 
                # Count the number of uncertain cells around the current cell and append to the list
                uncertain_count = self.count_uncertain_cells_around(index, occupancy_data_np, width, height)
                counts_list.append((index, uncertain_count))

        # Sort the list by uncertain_count in descending order
        sorted_counts = sorted(counts_list, key=lambda x: x[1], reverse=True)
        
        try:
            [self.new_waypoint.pose.position.x,self.new_waypoint.pose.position.y]  = self.cell_coordinates(sorted_counts[0][0])
            [self.potential_coordinate.point.x, self.potential_coordinate.point.y] = self.cell_coordinates(sorted_counts[0][0])
        except IndexError as e:
            self.get_logger().error(f"List of points is empty: {e}")

        distance2new = math.sqrt(
                        (self.potential_coordinate.point.x - self.current_position.pose.position.x)**2 +
                        (self.potential_coordinate.point.y- self.current_position.pose.position.y)**2
                    )
        
        self.get_logger().info('New Strategy: Point Distance:' + str(distance2new))
        self.new_strategy_counter += 1
        self.potential_publisher.publish(self.potential_coordinate)
        


    def count_uncertain_cells_around(self, index, occupancy_data_np, width, height):
        """Counts the number of uncertain cells (-1) around a given cell within a defined box size."""
        uncertain_count = 0
        box_size = 5  # Define the size of the box around each cell

        # Loop over the box around the cell
        for x in range(-box_size, box_size + 1):
            for y in range(-box_size, box_size + 1):
                slider = x * self.width + y
                try:
                    if occupancy_data_np[index + slider] == -1:
                        uncertain_count += 1
                #the index of a point next to the index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    continue
        return uncertain_count
           

    def frontier_check(self, occupancy_data_np, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes  = 0

    
        #Inspects the nature of points in a grid around the selected point 
        for x in range(-4,3):
            for y in range(-4,3):
                slider = x * self.width + y
                try:
                    if occupancy_data_np[random_index + slider] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data_np[random_index + slider] > self.obstacle_probability:
                        obstacle_indexes += 1
                #the index of a point next to the random_index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    continue
         
        if uncertain_indexes > 20:
            return True
        elif self.fully_mapped and obstacle_indexes > 5:
            return True
        else:
            return False
        

    def cell_coordinates(self,index):
        """
        Given an index of a cell in the occupancy grid, it returns the coordinates of the cell in the map frame.
        """
        resolution = 0.05
        origin_x = self.current_grid.info.origin.position.x
        origin_y = self.current_grid.info.origin.position.y
        width = self.current_grid.info.width

        # Compute correspondent row and column of the potential cell
        row_index_float = index / width
        row_index = math.ceil(row_index_float)
        col_index = index % width

        # Compute position with respect map frame of the potential cell
        x_coord = (col_index*resolution) + origin_x
        y_coord = (row_index*resolution) + origin_y

        return x_coord, y_coord

    def current_position_callback(self, msg:PoseWithCovarianceStamped):
        #Return current robot pose, unless searching_for_waypoint
        self.current_position.pose.position.x = msg.pose.pose.position.x
        self.current_position.pose.position.y = msg.pose.pose.position.y
        self.current_position.pose.orientation = msg.pose.pose.orientation
        self.current_position.header.frame_id = msg.header.frame_id


    def aruco_map_position_callback(self, msg:PointStamped):
        
        self.aruco_detected = True
        if not self.localisation_started:
            aruco_position = PointStamped()
            aruco_position = msg
            # Calculate distance to ArUco marker
            dx = aruco_position.point.x - self.current_position.pose.position.x
            dy = aruco_position.point.y - self.current_position.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > 5:
                # If further than 6 meters, move to the midpoint
                self.get_logger().info('ArUco marker is far. Moving to midpoint.')
                ratio = 0.5  # Midpoint
                self.new_waypoint.pose.position.x = self.current_position.pose.position.x + dx * ratio
                self.new_waypoint.pose.position.y = self.current_position.pose.position.y + dy * ratio

                # Calculate orientation towards the ArUco marker
                angle = math.atan2(dy, dx)
                self.new_waypoint.pose.orientation.z = math.sin(angle / 2)
                self.new_waypoint.pose.orientation.w = math.cos(angle / 2)

                self.waypoint_publisher.publish(self.new_waypoint)
                self.localisation_started = True

            elif distance > self.desired_distance:
                # If further than 1 meters, move towards the ArUco marker
                self.new_waypoint = PoseStamped()
                self.new_waypoint.header.frame_id = 'map'
                self.new_waypoint.header.stamp = self.get_clock().now().to_msg()
                
                # Calculate position 1 meters away from the ArUco marker
                ratio = 1 - (self.desired_distance/ distance)
                self.new_waypoint.pose.position.x = self.current_position.pose.position.x + dx * ratio
                self.new_waypoint.pose.position.y = self.current_position.pose.position.y + dy * ratio
                
                # Calculate orientation towards the ArUco marker
                angle = math.atan2(dy, dx)
                self.new_waypoint.pose.orientation.z = math.sin(angle / 2)
                self.new_waypoint.pose.orientation.w = math.cos(angle / 2)

                self.waypoint_publisher.publish(self.new_waypoint)
                self.get_logger().info('Moving towards the ArUco Marker')
                self.localisation_started = True
            else:
                # If within 1 meters, stop and wait
                self.get_logger().info('Within 1.5 meters of ArUco Marker. Stopping for 15 seconds.')
                
                # Publish current position as waypoint to make the robot stop
                stop_waypoint = PoseStamped()
                stop_waypoint.header.frame_id = 'map'
                stop_waypoint.header.stamp = self.get_clock().now().to_msg()
                stop_waypoint.pose.position.x = self.current_position.pose.position.x
                stop_waypoint.pose.position.y = self.current_position.pose.position.y
                stop_waypoint.pose.orientation = self.current_position.pose.orientation
                self.waypoint_publisher.publish(stop_waypoint)
                self.localisation_started = True

       

    def readiness_check(self, msg:BehaviorTreeLog):
        """
        Call the next_waypoint() when specific conditions of the BehaviorTreeLog message are satisfied
        """

        for event in msg.event_log:
            

            if event.node_name == 'NavigateRecovery' and event.current_status =='IDLE':
                self.get_logger().info('NavigateRecovery--IDLE received')

                if not self.aruco_detected:
                    self.next_waypoint()
                    self.goal_updated_counter = 0

                if self.localisation_started:
                    time.sleep(15)
                    self.next_waypoint()
                    self.localisation_started = False
                    self.aruco_detected = False

            if event.node_name == 'GoalUpdated' and event.current_status =='FAILURE':
                self.goal_updated_counter += 1
                if self.goal_updated_counter > 500:
                    if not self.aruco_detected:
                        self.next_waypoint()
                        self.goal_updated_counter = 0


def main():
    rclpy.init()
    autopilot_node = Autopilot()
    autopilot_node.get_logger().info('Running autopilot node')
    rclpy.spin(autopilot_node)

if __name__=='__main__':
    main()