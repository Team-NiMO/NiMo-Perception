from enum import Enum

# Currently, the only option for stalk detection is Mask R-CNN

GraspPointFindingOptions = Enum(
    'GraspPointFindingOptions',
    ['mask_only', 'mask_projection'])

BestStalkOptions = Enum(
    'BestStalkOptions', ['largest', 'largest_favorable'])

RUN_REALSENSE_ON_REQUEST = False
DRIVER_COMMAND = ['roslaunch', 'realsense2_camera', 'rs_camera_pcloud.launch']

# Camera parameters
DEPTH_SCALE = 1000.0
DEPTH_TRUNC = 10
INLIER_THRESHOLD = 0.025
MAX_RANSAC_ITERATIONS = 1000

# Used for RANSAC line detection for the stalks
MAX_STALK_THICKNESS = 0.005  # Used for RANSAC Inlier threshold
MAX_LINE_RANSAC_ITERATIONS = 1000
GRIPPER_WIDTH = 0.15  # 6 inches
GRIPPER_LENGTH_PAST_STALK = 0.075  # 3 inches

OPTIMAL_STALK_DISTANCE = 0.35
OPTIMAL_STALK_HEIGHT = 0.11  # 4 inches
MINIMUM_MASK_AREA = 30

# Distance in pixels between each feature point in a stalk
# Measured in pixels (and not meters) since feature point accuracy relies on image and depth image resolution
FEATURE_POINT_PIXEL_OFFSET = 10

# Time between when the last possible frame can be received and the request needs to finish
SERVICE_REQUEST_END_BUFFER_TIME = 0.1

MIN_X = 0.1
MAX_X = 0.6
MIN_Y = -0.07
MAX_Y = 0.25

# Model parameters
MODEL_PATH = '/home/thomasdetlefsen/NiMo/src/NiMo-Perception/model_field_day1.pth'
SCORE_THRESHOLD = 0.6
CUDA_DEVICE_NO = -1

VISUALIZE = True
VERBOSE = False
SAVE_IMAGES = False

IMAGE_TOPIC = '/camera/color/image_raw'
DEPTH_TOPIC = '/camera/depth/image_rect_raw'
CAMERA_INFO = '/camera/color/camera_info'
DEPTH_CAMERA_INFO = '/camera/depth/camera_info'
CAMERA_FRAME = 'camera_link'
BASE_FRAME = 'link_base'
WORLD_FRAME = 'camera_link' # TEMPORARY FOR TESTING SHOULD BE 'world'
CAMERA_COLOR_FRAME = 'camera_color_frame'
POINTCLOUD_TOPIC = '/camera/pointcloud/points'

# Select the algorithms to use
GRASP_POINT_ALGO = GraspPointFindingOptions.mask_projection
BEST_STALK_ALGO = BestStalkOptions.largest_favorable