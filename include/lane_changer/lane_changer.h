#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Point.h"

#include "waypoint_maker/Waypoint.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"

#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose2D.h"

#include "cmath"
#include "math.h"
#include "vector"
#include "tuple"
#include "std_msgs/Bool.h"
#include "clustering.cpp"

#define _USE_MATH_DEFINES
#define DIST_0 7.0
#define DIST_1 4.0

typedef pcl::PointXYZI PointType;

class LaneChanger{

private:
	//ros
    ros::NodeHandle nh_;

	//publisher
    ros::Publisher pub_;
    ros::Publisher marker_pub_;

	//subscriber
    ros::Subscriber sub_, lane_sub_;
	ros::Subscriber acker_sub_;

	//vector
    vector<geometry_msgs::Point> local_obs_;

	//various
	double detectAngle_;
	int isObsDetected_;
	int cur_state_;
	int lane_number_;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &input);
    void laneNumCallback(const waypoint_maker::WaypointConstPtr &lane_num);
	void ackermannCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &acker);
	void visualize(vector<geometry_msgs::Point> obs_points);

    double getDist(geometry_msgs::Point p);
    double calcSteer(geometry_msgs::Point p);

    LaneChanger() {
        initSetup();
    }
};

