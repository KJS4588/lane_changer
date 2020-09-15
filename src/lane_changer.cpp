#include "lane_changer/lane_changer.h"

void LaneChanger::initSetup() {
	pub_ = nh_.advertise<std_msgs::Bool>("/static_obs", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/output_points", 10);
    sub_ = nh_.subscribe("/velodyne_points", 10, &LaneChanger::pointCallback, this);
    lane_sub_ = nh_.subscribe("/target_state", 10, &LaneChanger::laneNumCallback, this);
	acker_sub_ = nh_.subscribe("/ctrl_cmd", 10, &LaneChanger::ackermannCallback, this);
	
	isObsDetected_ = false;
	detectAngle_ = 0;
	cur_state_ = 0;
	lane_number_ = 0;

}


//	0 : lane change when first obstacle detected
//  1 : ignore
//

void LaneChanger::laneNumCallback(const waypoint_maker::WaypointConstPtr &lane_num) {
	lane_number_ = lane_num->lane_number;
}

void LaneChanger::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input) {
	nh_.getParam("kuuve_state", cur_state_);

	if (cur_state_ == 2){
		if(detectAngle_ > 10 && detectAngle_ < -10) {
			local_obs_ = Cluster().cluster(input, 0, 0, 0, 0);
			visualize(local_obs_);
		}else {
			local_obs_ = Cluster().cluster(input, 1, 13, -1, 1);

			if (local_obs_.size() != 0){
				visualize(local_obs_);

				double dist = (lane_number_ == 0) ? DIST_0 : DIST_1;
					
				cout << "dist = " << dist << endl;
				cout << "obs_dist = " << getDist(local_obs_.at(0)) << endl;
				if(getDist(local_obs_.at(0)) < dist) {
					isObsDetected_ = true;	
					std_msgs::Bool obs;
					obs.data = isObsDetected_;
					pub_.publish(obs);

					isObsDetected_ = false;
				} else{
					std_msgs::Bool obs;
					obs.data = isObsDetected_;
					pub_.publish(obs);
				}
			}	
		}
	}
}


void LaneChanger::ackermannCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &acker){
	detectAngle_ = acker->drive.steering_angle;
}

double LaneChanger::getDist(geometry_msgs::Point p){
	return sqrt(pow(p.x - 0, 2) + pow(p.y - 0, 2) + pow(p.z - 0, 2));
}

void LaneChanger::visualize(vector<geometry_msgs::Point> input_points) {

	visualization_msgs::Marker points;

	points.header.frame_id = "velodyne";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 1;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;
	points.color.g = 1.0f;

	geometry_msgs::Point p;

	for (auto point : input_points) {
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		points.points.push_back(p);
	}

	marker_pub_.publish(points);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane_changer_node");
    LaneChanger lc;
	ros::spin();
}
