import rclpy
from rclpy.node import Node
from transitions import Machine
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from abra_interfaces.msg import AbraPath, AbraState, AbraControllerState
import lanelet2
from lanelet2.core import (AllWayStop, AttributeMap, BasicPoint2d,
                           BoundingBox2d, Lanelet, LaneletMap,
                           LaneletWithStopLine, LineString3d, Point2d, Point3d,
                           RightOfWay, TrafficLight, getId)
from lanelet2.projection import (UtmProjector, MercatorProjector,
                                 LocalCartesianProjector, GeocentricProjector)
import cubic_spline_planner.cubic_spline_planner as cubic_spline_planner
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String, Header
from nav_msgs.msg import Path
import tf_transformations
import random
import numpy as np

#Settings:

LANELET_MAP_PATH = "/home/mirac/Desktop/abra-autonomous/src/mapping/lanelet2_maps/map8.osm"
POINT_DOWNSAMPLING_RATE = 0.3
NODE_FREQUENCY = 1
PARK_ENTERANCE_LANELET_ID = 4674
STATION_LANELET_IDS = [392, 533, 5344]
STATION_FRONT_LANELET_IDS = [2122, 633, 4368]
PARK_LANELETS = [4813, 4799]
STATION_TOLERANCE = 1.0
STATION_WAIT_TIME = 5.0

class AbraFSMPlannerNode(Node):
    #States
    states = ['idle', 'drive', 'red_light_stop', 'station_enterance', 'station', 'park', 'obstacle_avoidance']

    def __init__(self):
        super().__init__("abra_fsm_planner_node")

        self.timer_period = 1/NODE_FREQUENCY
        self.timer = self.create_timer(self.timer_period, self.main_callback)
        self.clock = Clock()

        #Publishers
        self.current_path_publisher = self.create_publisher(AbraPath, 'abra/current_path', 1)
        self.controller_state_publisher = self.create_publisher(AbraControllerState, 'abra/controller_state', 1)
        self.path_visualize_publisher = self.create_publisher(Path, "abra/path_v", 1)

        #Subscribers
        self.localization_subscription = self.create_subscription(AbraState, 'abra/localization_state', self.localization_state_callback, 10)
        self.goal_pose_subscription = self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 1)
        self.test_subscription = self.create_subscription(Int32, "/planner_test", self.test_sub_callback, 10)
        self.controller_feedback_subscription = self.create_subscription(String, "abra/controller_feedback", self.controller_feedback_callback, 10)

        #FSM variables
        self.machine = Machine(model=self, states=AbraFSMPlannerNode.states, initial='idle')

        #Transitions
        self.machine.add_transition(trigger='start_driving', source='idle', dest='drive', before='plan_initial_path')
        self.machine.add_transition(trigger='red_light', source='drive', dest='red_light_stop')
        self.machine.add_transition(trigger='green_light', source='red_light_stop', dest='drive')
        self.machine.add_transition(trigger='station_sign_detected', source='drive', dest='station_enterance', before="plan_station_enterance_path")
        self.machine.add_transition(trigger='entered_station', source='station_enterance', dest='station')
        self.machine.add_transition(trigger='exit_station', source='station', dest='drive', before='plan_path_after_station')
        self.machine.add_transition(trigger='obstacle_detected', source='drive', dest='obstacle_avoidance', before='plan_obstacle_avoidance_path')
        self.machine.add_transition(trigger='obstacle_avoided', source='obstacle_avoidance', dest='drive', before='plan_path_after_avoidance')
        self.machine.add_transition(trigger="goal_received", source="*", dest="drive", before='plan_goal_path')
        self.machine.add_transition(trigger="turn_left_sign_detected", source="drive", dest="drive", before="plan_turn_left_path")
        self.machine.add_transition(trigger="turn_right_sign_detected", source="drive", dest="drive", before="plan_turn_right_path")
        #self.machine.add_transition(trigger="turned_left", source="turn_left", dest="drive", before="plan_path_after_left_turn")
        #self.machine.add_transition(trigger="turned_right", source="turn_right", dest="drive", before="plan_path_after_right_turn")
        self.machine.add_transition(trigger="no_left_turn_sign_detected", source="drive", dest="drive", before="plan_no_left_turn_path")
        self.machine.add_transition(trigger="red_light_detected", source="drive", dest="red_light_stop")
        self.machine.add_transition(trigger="green_light_detected", source="red_light_stop", dest="drive", conditions="is_in_red_light_stop")
        self.machine.add_transition(trigger="reached_park_zone", source="drive", dest="park", before="plan_park_path")
        
        #State variables
        self.current_localization_state= AbraState()
        self.current_controller_state = "stop"
        self.controller_feedback = ""
        self.station_count = 0

        #Map variables
        self.map = None
        self.x_offset = None
        self.y_offset = None

        #Routing variables
        self.routing_graph = None
        self.current_goal_lanelet = None
        self.current_path = None
        self.current_path_id = 0
        self.goal_pose_x = None
        self.goal_pose_y = None
        self.right_turn_lanelets = []
        self.left_turn_lanelets = []

        #Station task related variables
        self.station_enter_time = 0
        self.goal_station_lanelet = None

        #Obstacle avoidance related variables
        self.current_lane_type = "up"

        #Initial operations
        self.initialize_map()
        self.initialize_routing_graph()

    #FSM functions:

    def on_enter_stop(self):
        print("Vehicle has stopped")

    def on_enter_park(self):
        self.run_controller_and_publish_path()

    def on_enter_idle(self):
        print("Vehicle is idle")
    
    def on_enter_station(self):
        self.station_count += 1
        self.station_enter_time = self.clock.now()
        self.current_controller_state = "stop"
    
    def on_enter_red_light_stop(self):
        self.current_controller_state = "stop"

    def on_enter_drive(self):
        self.run_controller_and_publish_path()

    def on_enter_station_enterance(self):
        self.run_controller_and_publish_path()
    
    def on_enter_obstacle_avoidance(self):
        self.run_controller_and_publish_path()
    
    def on_enter_turn_left(self):
        self.run_controller_and_publish_path()
    
    def on_enter_turn_right(self):
        self.run_controller_and_publish_path()
    
    def is_in_red_light_stop(self):
        return self.state == "red_light_stop"
    
    def plan_initial_path(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        via_lanelets = [self.map.laneletLayer[STATION_FRONT_LANELET_IDS[i]] for i in range(len(STATION_FRONT_LANELET_IDS))]
        goal_lanelet = self.map.laneletLayer[PARK_ENTERANCE_LANELET_ID]
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet, via_lanelets)

    def plan_goal_path(self):
        point = self.create_point(self.goal_pose_x, self.goal_pose_y)
        goal_lanelet = self.get_nearest_lanelet(point)
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet)
        self.visualize_path()
    
    def plan_station_enterance_path(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        goal_lanelet = self.get_nearest_station_lanelet(vehicle_position_point)
        self.goal_station_lanelet = goal_lanelet
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet)
        
    
    def plan_path_after_station(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        goal_lanelet = self.map.laneletLayer[PARK_ENTERANCE_LANELET_ID]
        nearest_station_front_lanelet = self.get_nearest_station_front_lanelet(vehicle_position_point)
        other_station_front_lanelets = [self.map.laneletLayer[i] for i in STATION_FRONT_LANELET_IDS if i != nearest_station_front_lanelet.id]
        if self.station_count != 2:
            self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet, via_lanelets=other_station_front_lanelets)
        else:
            self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet)
    
    def plan_obstacle_avoidance_path(self):
        ax = []
        ay = []
        current_pos_x = self.current_localization_state.x
        current_pos_y = self.current_localization_state.y
        current_yaw = self.current_localization_state.yaw
    
        possible_angles = [0.0, np.pi/2, np.pi, -np.pi/2]
        nearest_angle = self.find_nearest_angle(possible_angles, current_yaw)
        
        if nearest_angle == 0.0:
            for i in range(10):
                ay.append(current_pos_y + 2)
                ax.append(current_pos_x + i)
            
            ay.append(current_pos_y)
            ax.append(current_pos_x + 12)
        
        elif nearest_angle == np.pi/2:
            for i in range(10):
                ax.append(current_pos_x - 2)
                ay.append(current_pos_y + i)
            
            ax.append(current_pos_x)
            ay.append(current_pos_y + 12)
        
        elif nearest_angle == np.pi:
            for i in range(10):
                ay.append(current_pos_y - 2)
                ax.append(current_pos_x - i)
            
            ay.append(current_pos_y)
            ax.append(current_pos_x - 12)
        
        elif nearest_angle == -np.pi/2:
            for i in range(10):
                ay.append(current_pos_y - i)
                ax.append(current_pos_x + 2)
            
            ax.append(current_pos_x)
            ay.append(current_pos_y - 12)
            
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.3)

        self.current_path_id += 1
        path = AbraPath()
        path.cx, path.cy, path.cyaw = cx, cy, cyaw
        path.id = self.current_path_id

        self.current_path = path
    
    
    def plan_path_after_avoidance(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        via_lanelets = [self.map.laneletLayer[STATION_FRONT_LANELET_IDS[i]] for i in range(len(STATION_FRONT_LANELET_IDS))]
        goal_lanelet = self.map.laneletLayer[PARK_ENTERANCE_LANELET_ID]
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet, via_lanelets)

    def plan_turn_left_path(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        goal_lanelet = self.map.laneletLayer[PARK_ENTERANCE_LANELET_ID]
        nearest_left_lanelet = self.get_nearest_left_turn_lanelet(vehicle_position_point)
        via_lanelets = [self.map.laneletLayer[STATION_FRONT_LANELET_IDS[i]] for i in range(len(STATION_FRONT_LANELET_IDS))]
        via_lanelets.append(nearest_left_lanelet)
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet, via_lanelets)
    
    def plan_no_left_turn_path(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        goal_lanelet = self.map.laneletLayer[PARK_ENTERANCE_LANELET_ID]
        following_lanelets = self.routing_graph.following(from_lanelet)
        
        right_turn_possible_lanelet = None
        straight_possible_lanelet = None
        
        via_lanelets = []
        
        for lanelet in following_lanelets:
            if lanelet in self.right_turn_lanelets:
                right_turn_possible_lanelet = lanelet
            else:
                if lanelet not in self.left_turn_lanelets:
                    straight_possible_lanelet = lanelet
        
        if straight_possible_lanelet != None and right_turn_possible_lanelet == None:
            via_lanelets.append(straight_possible_lanelet)
            if self.station_count != 2:
                station_front_lanelets = [self.map.laneletLayer[STATION_FRONT_LANELET_IDS[i]] for i in range(len(STATION_FRONT_LANELET_IDS))]
                via_lanelets = via_lanelets + station_front_lanelets
            
            self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet, via_lanelets)
            return
        
        elif straight_possible_lanelet == None and right_turn_possible_lanelet != None:
            via_lanelets.append(right_turn_possible_lanelet)
            if self.station_count != 2:
                station_front_lanelets = [self.map.laneletLayer[STATION_FRONT_LANELET_IDS[i]] for i in range(len(STATION_FRONT_LANELET_IDS))]
                via_lanelets = via_lanelets + station_front_lanelets
            
            self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet, via_lanelets)
            return
        
        elif straight_possible_lanelet != None and right_turn_possible_lanelet != None:
            right_turn_via_lanelets = [right_turn_possible_lanelet]
            straight_via_lanelets = [straight_possible_lanelet]
            
            if self.station_count != 2:
                station_front_lanelets = [self.map.laneletLayer[STATION_FRONT_LANELET_IDS[i]] for i in range(len(STATION_FRONT_LANELET_IDS))]
                right_turn_via_lanelets = right_turn_via_lanelets + station_front_lanelets
                straight_via_lanelets = straight_via_lanelets + station_front_lanelets
            
            right_turn_path = self.routing_graph.getRouteVia(from_lanelet, right_turn_via_lanelets, goal_lanelet)
            straight_turn_path = self.routing_graph.getRouteVia(from_lanelet, straight_via_lanelets, goal_lanelet)
            
            right_turn_cost = right_turn_path.length2d()
            straight_turn_cost = straight_turn_path.length2d()
            
            if right_turn_cost > straight_turn_cost:
                self.current_path = self.create_cubic_spline_path(from_lanelet, straight_via_lanelets, goal_lanelet)
            else:
                self.current_path = self.create_cubic_spline_path(from_lanelet,right_turn_cost, goal_lanelet)

    def plan_turn_right_path(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        goal_lanelet = self.map.laneletLayer[PARK_ENTERANCE_LANELET_ID]
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet)
        
    def plan_park_path(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        from_lanelet = self.get_nearest_lanelet(vehicle_position_point)
        goal_lanelet_id = random.choice(PARK_LANELETS)
        goal_lanelet = self.map.laneletLayer[goal_lanelet_id]
        self.current_path = self.create_cubic_spline_path(from_lanelet, goal_lanelet)

    #Subscription callbacks:


    def localization_state_callback(self, msg):
        # Process the incoming state
        self.current_localization_state = msg
    
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y
        self.goal_received()
    
    def controller_feedback_callback(self, msg):
        self.controller_feedback = msg.data
    
    def test_sub_callback(self, msg):
        if msg.data == 1:
            if self.state != "drive":
                self.start_driving()
        elif msg.data == 2:
            if self.state != "station_enterance":
                self.station_sign_detected()
        elif msg.data == 3:
            if self.state != "obstacle_avoidance":
                self.obstacle_detected()
        elif msg.data == 4:
            if self.state != "turn_left":
                self.turn_left_sign_detected()
        
        elif msg.data == 5:
            if self.state != "turn_right":
                self.turn_right_sign_detected()
        
        elif msg.data == 6:
            if self.state != "red_light_stop":
                self.red_light_detected()
        
        elif msg.data == 7:
            if self.state != "drive":
                self.green_light_detected()
        
        elif msg.data == 8:
            if self.state != "park":
                self.reached_park_zone()
        
        elif msg.data == 9:
            self.no_left_turn_sign_detected()
    
    #Main callback:

    def main_callback(self):
        print(self.state)
        self.handle_station_conditions()
        self.handle_obstacle_conditions()
        #self.handle_turn_conditions()
        controller_state_msg = AbraControllerState()
        controller_state_msg.controller_state = self.current_controller_state
        self.controller_state_publisher.publish(controller_state_msg)
    
    #Helper functions:

    def initialize_map(self):
        self.map = lanelet2.io.load(LANELET_MAP_PATH, lanelet2.io.Origin(0,0,0))
        lanelets = [i for i in self.map.laneletLayer]
        point = lanelets[0].centerline[0]
        self.x_offset = point.x - float(point.attributes["local_x"])
        self.y_offset = point.y - float(point.attributes["local_y"])
        
        for lanelet in self.map.laneletLayer:
            try:    
                if lanelet.attributes["turn_direction"] == "right":
                    self.right_turn_lanelets.append(lanelet)
                elif lanelet.attributes["turn_direction"] == "left":
                    self.left_turn_lanelets.append(lanelet)
            except KeyError:
                pass
                
    
    def initialize_routing_graph(self):
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        self.routing_graph = lanelet2.routing.RoutingGraph(self.map, traffic_rules)
    
    def get_nearest_lanelet(self, point):
        return lanelet2.geometry.findNearest(self.map.laneletLayer, point, 1)[0][1]

    def create_cubic_spline_path(self, fromLanelet, toLanelet, via_lanelets=None):
        if via_lanelets == None:
            route = self.routing_graph.getRoute(fromLanelet, toLanelet)
        else:
            route = self.routing_graph.getRouteVia(fromLanelet, via_lanelets, toLanelet)
        
        path = route.shortestPath()

        ax = []
        ay = []
        for lanelet in path:
            print(lanelet.id)
            for point in lanelet.centerline:
                ax.append(float(point.attributes["local_x"]))
                ay.append(float(point.attributes["local_y"]))

        ax = self.select_evenly(ax, int(len(ax)*POINT_DOWNSAMPLING_RATE))
        ay = self.select_evenly(ay, int(len(ay)*POINT_DOWNSAMPLING_RATE))

        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.3)

        self.current_path_id += 1
        path = AbraPath()
        path.id = self.current_path_id
        path.cx = cx
        path.cy = cy
        path.cyaw = cyaw
        

        return path
    
    def find_nearest_angle(self, angle_list, target_angle):
        """
        Find the nearest angle in a list to the target angle.

        Args:
        - angle_list (list of float): List of angles in radians.
        - target_angle (float): The target angle in radians.

        Returns:
        - float: The angle from the list closest to the target angle.
        """
        def angle_distance(a1, a2):
            # Calculate the shortest distance between two angles
            diff = abs(a1 - a2) % (2 * np.pi)
            return min(diff, 2 * np.pi - diff)
        
        # Find the angle in the list with the smallest distance to the target angle
        nearest_angle = min(angle_list, key=lambda angle: angle_distance(angle, target_angle))
    
        return nearest_angle
    
    
    def select_evenly(self, my_list, N):
        length = len(my_list)
        step = length / N
        selected_elements = [my_list[int(i * step)] for i in range(N)]
        
        return selected_elements
    
    def create_point(self, x, y):
        point = BasicPoint2d()
        point.x = x + self.x_offset
        point.y = y + self.y_offset
        return point
    
    def get_nearest_station_lanelet(self, point):
        min_distance = float('inf')
        min_distance_id = None
        for lanelet_id in STATION_LANELET_IDS:
            distance = lanelet2.geometry.distanceToCenterline2d(self.map.laneletLayer[lanelet_id], point)
            if distance < min_distance:
                min_distance = distance
                min_distance_id = lanelet_id
        
        return self.map.laneletLayer[min_distance_id]
    
    def get_nearest_station_front_lanelet(self, point):
        min_distance = float('inf')
        min_distance_id = None
        for lanelet_id in STATION_FRONT_LANELET_IDS:
            distance = lanelet2.geometry.distanceToCenterline2d(self.map.laneletLayer[lanelet_id], point)
            if distance < min_distance:
                min_distance = distance
                min_distance_id = lanelet_id
        
        return self.map.laneletLayer[min_distance_id]
    
    def get_nearest_right_turn_lanelet(self, point):
        min_distance = float('inf')
        min_distance_lanelet = None
        #right_turn_lanelets = [lanelet for lanelet in self.map.laneletLayer if lanelet.attributes["turn_direction"] == "right"]
        for lanelet in self.right_turn_lanelets:
            distance = lanelet2.geometry.distanceToCenterline2d(lanelet, point)
            if distance < min_distance:
                min_distance = distance
                min_distance_lanelet = lanelet
        
        return min_distance_lanelet
    
    def get_nearest_left_turn_lanelet(self, point):
        min_distance = float('inf')
        min_distance_lanelet = None
        #left_turn_lanelets = [lanelet for lanelet in self.map.laneletLayer if lanelet.attributes["turn_direction"] == "left"]
        for lanelet in self.left_turn_lanelets:
            distance = lanelet2.geometry.distanceToCenterline2d(lanelet, point)
            if distance < min_distance:
                min_distance = distance
                min_distance_lanelet = lanelet
                
        return min_distance_lanelet
    
    
    def station_position_checker(self):
        vehicle_position_point = self.create_point(self.current_localization_state.x, self.current_localization_state.y)
        distance_to_station = lanelet2.geometry.distanceToCenterline2d(self.goal_station_lanelet, vehicle_position_point)
        if distance_to_station <= STATION_TOLERANCE:
            return True
        return False
    
    def station_time_checker(self):
        delta_time = self.clock.now() - self.station_enter_time
        print(delta_time.nanoseconds / 1e9)
        if delta_time.nanoseconds / 1e9 >= STATION_WAIT_TIME:
            return True
        else:
            return False
    
    def handle_station_conditions(self):
        if self.state == "station_enterance":
            if self.station_position_checker() == True:
                self.entered_station()
        elif self.state == "station":
            if self.station_time_checker() == True:
                self.exit_station()
    
    def handle_obstacle_conditions(self):
        if self.state == "obstacle_avoidance":
            if self.controller_feedback == "goal_reached":
                self.obstacle_avoided()
    
    def handle_turn_conditions(self):
        if self.state == "turn_left":
            if self.controller_feedback == "goal_reached":
                self.turned_left()
        
        
        elif self.state == "turn_right":
            if self.controller_feedback == "goal_reached":
                self.turned_right()
    
    def visualize_path(self):
        path = Path()
        path_header = Header()
        path_header.stamp = self.get_clock().now().to_msg()
        path_header.frame_id = 'map'
        path.header = path_header

        poses = []
        for i in range(len(self.current_path.cx)):
            pose = PoseStamped()
            pose_header = Header()
            pose_header.stamp = self.get_clock().now().to_msg()
            pose_header.frame_id = 'map'
            pose.header = pose_header
            pose.pose.position.x = self.current_path.cx[i]
            pose.pose.position.y = self.current_path.cy[i]
            quaternion = tf_transformations.quaternion_from_euler(0, 0, self.current_path.cyaw[i])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            poses.append(pose)
        path.poses = poses
        self.path_visualize_publisher.publish(path)
    

    def run_controller_and_publish_path(self):
        self.current_controller_state = "run"
        self.current_path_publisher.publish(self.current_path)
        self.visualize_path()

# class AbraFSMPlannerNode(Node):
#     def __init__(self):
#         super().__init__('vehicle_node')
#         self.fsm = VehicleFSM()

#         self.create_subscription(String, 'start_driving', self.start_driving_callback, 10)
#         self.create_subscription(String, 'stop_driving', self.stop_driving_callback, 10)
#         self.create_subscription(String, 'park_vehicle', self.park_vehicle_callback, 10)
#         self.create_subscription(String, 'restart', self.restart_callback, 10)

#     def start_driving_callback(self, msg):
#         self.fsm.start_driving()

#     def stop_driving_callback(self, msg):
#         self.fsm.stop_driving()

#     def park_vehicle_callback(self, msg):
#         self.fsm.park_vehicle()

#     def restart_callback(self, msg):
#         self.fsm.restart()

def main(args=None):
    rclpy.init(args=args)
    node = AbraFSMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
