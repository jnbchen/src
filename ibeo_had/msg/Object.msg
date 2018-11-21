uint8 UNCLASSIFIED=0
uint8 UNKNOWN_SMALL=1
uint8 UNKNOWN_BIG=2
uint8 PEDESTRIAN=3
uint8 BIKE=4
uint8 CAR=5
uint8 TRUCK=6

uint16 id
uint32 age

uint8 classification
uint8 classification_certainty
uint32 classification_age

Point2D object_box_center
Point2D object_box_size

Color color
string text

float32 yaw_angle

Point2D[] contour

Point2D absolute_velocity
Point2D relative_velocity


