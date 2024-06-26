import numpy as np
import pyransac3d as pyrsc
from geometry_msgs.msg import Point
import warnings
from message_filters import Subscriber
from skspatial.objects import Line as Line3D

from skimage.measure import LineModelND, ransac

from stalk_detect.config import INLIER_THRESHOLD, MAX_LINE_RANSAC_ITERATIONS, MAX_X, MIN_X, MAX_Y, MIN_Y, MAX_Z, MIN_Z, OPTIMAL_STALK_HEIGHT

# Line object, with a slope and intercept
class Line:
    def __init__(self, slope, intercept, points=[]):
        if len(slope) != 3 or len(intercept) != 3:
            raise ValueError('Slope and intercept must be of length 3')
        self.slope = slope / np.linalg.norm(slope)
        self.intercept = intercept
        self.points = points

def ransac_3d(points):
    '''
    Perform RANSAC line detection on a set of 3D points

    Parameters
        points (list[Point]): The points to perform RANSAC on

    Returns
        best_line (np.ndarray[Point]): The best line found
    '''
    # Convert np.ndarray[Point] to np.array (N, 3)
    points = np.array([[p.x, p.y, p.z] for p in points])

    # Catch the RuntimeWarning that pyransac3d throws when it fails to find a line
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        line = pyrsc.Line().fit(points, thresh=INLIER_THRESHOLD, maxIteration=MAX_LINE_RANSAC_ITERATIONS)
        if len(w) > 0 and not issubclass(w[-1].category, RuntimeWarning):
            warnings.warn(w[-1].message, w[-1].category)

    return line

def ransac_2d(points):
    '''
    Perform RANSAC line detection on a set of 2D points

    Parameters
        points (list[Point]): The points to perform RANSAC on

    Returns
        Line: The best line found
    '''

    points = np.array([[p.x, p.y] for p in points])
    
    model = LineModelND()
    model.estimate(points)

    model_robust, inliers = ransac(points, LineModelND, min_samples=2,
                               residual_threshold=1, max_trials=MAX_LINE_RANSAC_ITERATIONS)

    x = np.array([0, 1])
    y = model_robust.predict_y([0, 1])

    slope = (y[1] - y[0]) / (x[1] - x[0])
    intercept = model_robust.predict_x([0])
    inlier_points = points[inliers]

    return slope, intercept, inlier_points


def fit_line(points):
    '''
    Perform RANSAC line detection, then refine the line using least squares

    Parameters
        points (list[Point]): The points to perform RANSAC on

    Returns
        Line: The best line found
    '''
    line: Line = ransac_3d(points)

    if len(line.points) == 0:
        raise ValueError("No line found")

    # # Refine using least squares
    line_fit = Line3D.best_fit(line.points)

    line.slope = line_fit.direction
    line.intercept = line_fit.point

    return line


def find_xy_from_z(line, z):
    '''
    Find the x and y coordinates on a line given a z coordinate

    Parameters
        line (np.ndarray[Point]): The line to find the point on
        z (float): The z coordinate of the point

    Returns
        x (float): The x coordinate of the point
        y (float): The y coordinate of the point
    '''
    normalized_direction = line[0] / np.linalg.norm(line[0])
    t = (z - line[1][2]) / normalized_direction[2]

    return line[1][0] + t * normalized_direction[0], line[1][1] + t * normalized_direction[1]

class Stalk:
    '''
    Helper class for storing a 3D stalk

    A Stalk is always in world frame
    '''
    def __init__(self, points: 'list[Point]', score: float, mask: np.ndarray, width: float):
        self.points = points
        self.line = ransac_3d(points)
        self.score = score
        self.mask = mask
        self.width = width

    def set_grasp_point(self, min_height=0):
        '''
        Get the point on the stalk to grasp

        Parameters
            min_height (float): The minimum height the stalk can touch
        '''
        if 'grasp_point' in self.__dict__:
            return

        # Retrieve the point above the lowest point
        goal_height = min_height - OPTIMAL_STALK_HEIGHT

        # Find the point on the line at this height
        x, y = find_xy_from_z(self.line, goal_height)

        self.grasp_point = Point(x=x, y=y, z=goal_height)

    def is_valid(self):
        return len(self.line[0]) > 0 and self.width > 0

    def is_within_bounds(self) -> bool:
        return self.cam_grasp_point.x <= MAX_X and self.cam_grasp_point.x >= MIN_X and \
            self.cam_grasp_point.y <= MAX_Y and self.cam_grasp_point.y >= MIN_Y and \
            self.cam_grasp_point.z <= MAX_Z and self.cam_grasp_point.z >= MIN_Z

    def set_cam_grasp_point(self, cam_grasp_point):
        self.cam_grasp_point = cam_grasp_point


class KillableSubscriber(Subscriber):
    '''
    A message filter subscriber that can unregister to the topic
    '''
    def unregister(self):
        self.sub.unregister()
