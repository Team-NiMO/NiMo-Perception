import numpy as np
import open3d as o3d
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import ColorRGBA, Header
from termcolor import colored
from visualization_msgs.msg import Marker, MarkerArray

# Unfortunate hack to fix a bug in ROS Noetic
np.float = np.float64

from open3d_ros_helper import open3d_ros_helper as orh


class Visualizer:
    '''
    Publish a variety of visualizations for the pipeline
    '''
    @classmethod
    def __init__(cls):
        # Map the topic name to the (Publisher, topic_type) tuple
        cls.publishers: dict[str, tuple[rospy.Publisher, type]] = {}

        # Current marker counter
        cls.marker_counter = 0

        # Map the data type to the ROS message type and conversion function
        cls.data_to_ros_type = {
            np.ndarray: (Image, lambda img: CvBridge().cv2_to_imgmsg(img, encoding='bgr8')),
            Point: (Marker, lambda pt: cls._point_to_marker(pt, (255, 0, 0), cls.marker_counter, 0.01)),
            o3d.geometry.PointCloud: (PointCloud2, lambda pcl: orh.o3dpc_to_rospc(pcl, frame_id='world', stamp=rospy.Time.now())),
            list: (MarkerArray, lambda lst: cls.list_to_marker_array(lst))
        }
        cls.output_types = [i[0] for i in list(cls.data_to_ros_type.values())]

        cls.marker_color = [255, 0, 0]
        cls.marker_size = 0.01

    @classmethod
    def list_to_marker_array(cls, points: list) -> MarkerArray:
        if isinstance(points[0], np.ndarray):
            # Combine all the markers across multiple stalks into one MarkerArray
            markers = []
            for sub_array in points:
                for j in sub_array:
                    markers.append(cls._point_to_marker(j, cls.marker_color, cls.marker_counter, size=cls.marker_size))
                    cls.marker_counter += 1

            return MarkerArray(markers=markers)

        elif isinstance(points[0], Point):
            return MarkerArray(markers=[cls._point_to_marker(
                i, cls.marker_color, cls.marker_counter + j, size=cls.marker_size) for j, i in enumerate(points)])

    @classmethod
    def _point_to_marker(cls, point: Point, color, id: int, size: float) -> Marker:
        '''
        Convert a point to a marker

        Parameters
            point (geometry_msgs.msg.Point): the point to convert

        Returns
            visualization_msgs.msg.Marker: the marker
        '''
        header = Header(frame_id='world', stamp=rospy.Time.now())

        return Marker(header=header, id=id, pose=Pose(position=point, orientation=Quaternion(1, 0, 0, 0)),
                      lifetime=rospy.Duration(0), type=Marker.SPHERE, scale=Point(x=size, y=size, z=size),
                      color=ColorRGBA(r=color[0] / 255., g=color[1] / 255., b=color[2] / 255., a=1.), action=Marker.ADD)

    @classmethod
    def new_frame(cls):
        cls.marker_counter = 0
        cls.marker_size = 0.01
        cls.marker_color = [255, 0, 0]

    @classmethod
    def publish_item(cls, topic, item, **kwargs):
        '''
        Publish an item to the visualization topic

        Parameters
            topic (str): the topic of the item to publish
            item (np.ndarray): the item to publish
        '''
        if topic not in cls.publishers.keys():
            # Check to ensure the item is a valid type (not in the keys or values of data_to_ros_type)
            if type(item) not in list(cls.data_to_ros_type.keys()) + cls.output_types:
                print(colored('Invalid visualization type {} when trying to publish with id {}'.format(
                    type(item), topic), 'red'))
                print(colored('Valid types are: {}'.format(
                    list(cls.data_to_ros_type.keys()) + list(cls.data_to_ros_type.values())), 'yellow'))
                return
            else:
                topic_type = cls.data_to_ros_type[type(item)][0] if type(item) in cls.data_to_ros_type.keys() else type(item)
                cls.publishers[topic] = (rospy.Publisher('stalk_detect/viz/{}'.format(topic), topic_type, queue_size=10), topic_type)

        if type(item) in cls.data_to_ros_type.keys():
            # Validate the additional arguments for Marker and MarkerArray types
            if 'marker_size' in kwargs.keys():
                if cls.publishers[topic][1] in [Marker, MarkerArray]:
                    cls.marker_size = kwargs['marker_size']
                else:
                    print(colored('marker_size argument should only be set for Marker, MarkerArray types', 'red'))

            if 'marker_color' in kwargs.keys():
                if cls.publishers[topic][1] in [Marker, MarkerArray]:
                    cls.marker_color = kwargs['marker_color']
                else:
                    print(colored('marker_color argument should only be set for Marker, MarkerArray types', 'red'))

            # Convert the data to a ROS message via the conversion function
            try:
                msg = cls.data_to_ros_type[type(item)][1](item)
            except Exception as e:
                print(colored('Error converting data to ROS message: {}'.format(e), 'red'))
                return
        else:
            msg = item
            msg.header = Header(frame_id='world', stamp=rospy.Time.now())

        # Delete all old markers on this topic
        if cls.publishers[topic][1] == Marker:
            cls.publishers[topic][0].publish(Marker(action=Marker.DELETEALL))
        elif cls.publishers[topic][1] == MarkerArray:
            cls.publishers[topic][0].publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

        cls.publishers[topic][0].publish(msg)
