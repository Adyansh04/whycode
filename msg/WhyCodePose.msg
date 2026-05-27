#WhyCode marker pose with ID and tracking info 

int32 whycode_id # WhyCode decoded ID 
int32 tracking_id # Persistent tracking ID for the marker
bool id_valid # True if ID is reliable

geometry_msgs/Pose pose # 3D pose (position + orientation)
