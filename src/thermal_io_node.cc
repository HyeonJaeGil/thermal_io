#include "thermal_io/thermal_io.h"
#include "thermal_io/thermal_8bit_transformer.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "thermal_io_node");
    Thermal8bitTransformer tf_left
        ("/home/hj/Workspace/thermal_ws/src/thermal_io/config/thermal_14bit_left.yaml");
    Thermal8bitTransformer tf_right
        ("/home/hj/Workspace/thermal_ws/src/thermal_io/config/thermal_14bit_right.yaml");
    ROS_INFO("Starting thermal_io_node ...");
    ros::spin();
    return 0;

}