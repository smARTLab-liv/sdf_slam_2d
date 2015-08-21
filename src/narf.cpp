
#include "graph/narf.h"

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

    ROS_INFO("narf started");

    ros::init(argc, argv, "narf");
    ros::NodeHandle nh;
    pcd_sub_ = nh.subscribe("local_pcd_map", 10, pcdCb);

    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage (argv[0]);
        return 0;
    }
    if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
        cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
    int tmp_coordinate_frame;
    if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
        cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
    }
    if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
        cout << "Setting support size to "<<support_size<<".\n";
    if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
        cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
    angular_resolution = pcl::deg2rad (angular_resolution);


    //--------------------
    // -----Main loop-----
    //--------------------
    while (ros::ok())
    {
        ros::spinOnce();
        pcl_sleep(0.01);
    }
}