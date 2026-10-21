#WhyCode marker pose with ID and tracking info 

int32 whycode_id # WhyCode decoded ID 
int32 tracking_id # Persistent tracking ID for the marker
bool id_valid # True if ID is reliable
float32 orientation_confidence # Probability that the orientation is the right one of the two mirror solutions; 0.5 = undecided

geometry_msgs/Pose pose # 3D pose (position + orientation)
