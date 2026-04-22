from label_detect import decodeShelf
from label_detect_nav import decodeShelfNav
from shelf_distance import getShelfDistance
from shelf_distance_nav import getShelfDistanceNav()

ret = decodeShelf()
yaw = ret[1]
ret_nav = decodeShelfNav()
yaw_nav = ret_nav[1]
distance = getShelfDistance(yaw)
distance_nav = getShelfDistanceNav()


