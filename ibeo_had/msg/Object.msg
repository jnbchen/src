uint8 UNCLASSIFIED=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 PEDESTRIAN=3
uint8 BIKE=4
uint8 CAR=5
uint8 TRUCK=6

uint16 id
uint32 age
uint32 flag

uint8 classification
uint8 classification_certainty
uint32 classification_age

geometry_msgs/Pose2D object_pose
geometry_msgs/Polygon contour

std_msgs/ColorRGBA color
string text

float32 object_box_x
float32 object_box_y

float32 absolute_velocity_x
float32 absolute_velocity_y

float32 relative_velocity_x
float32 relative_velocity_y
