std_msgs/Header header

# All arrays are parallel, so the index of a marker in one array corresponds to the same index in the other arrays

uint8[] ids
# Confidence from [0, 100]. Only relevant for objects
float32[] confs
# Translations from the camera frame to the marker frame in meters
geometry_msgs/Vector3[] translations

sensor_msgs/RegionOfInterest[] rois