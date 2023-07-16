#include <D-Map/D-Map.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <livox_ros_driver/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define BUFFER_SIZE 100

std::vector<double> EXT_T, EXT_Q;

double FOV_DEPTH = 30.0;

bool VIS_EN = true;
bool LOG_MAP = false;
string point_frame, lidar_topic, depth_topic, odom_topic, fixed_frame;
string odom_topic_type;
string log_name;

DMap::dmap *dmap_;

ros::Publisher dmap_octree_pub, bbx_pub, depthmap_pub, dmap_gridmap_pub, odom_pub;

OdomType odom_q, ext;

ros::Time last_updated;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyCloudPose;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
SynchronizerCloudPose sync_cloud_pose_;
SynchronizerCloudOdom sync_cloud_odom_;

void Visualization() {
	// Publish odom
	nav_msgs::Odometry dmap_odom;
	dmap_odom.header.stamp = ros::Time::now();
	dmap_odom.header.frame_id = fixed_frame.c_str();
	dmap_odom.pose.pose.position.x = odom_q.pos(0);
	dmap_odom.pose.pose.position.y = odom_q.pos(1);
	dmap_odom.pose.pose.position.z = odom_q.pos(2);
	dmap_odom.pose.pose.orientation.w = odom_q.q.w();
	dmap_odom.pose.pose.orientation.x = odom_q.q.x();
	dmap_odom.pose.pose.orientation.y = odom_q.q.y();
	dmap_odom.pose.pose.orientation.z = odom_q.q.z();
	odom_pub.publish(dmap_odom);

	// Publish octree
	PointVector vis_ptclouds;
	PointClouds cloud_vis;
	sensor_msgs::PointCloud2 output_pointcloud;
	dmap_->unknown_pointcloud_visualize(vis_ptclouds);
	cloud_vis.clear();
	cloud_vis.points.assign(vis_ptclouds.begin(), vis_ptclouds.end());
	pcl::toROSMsg(cloud_vis, output_pointcloud);
	output_pointcloud.header.stamp = ros::Time::now();
	output_pointcloud.header.frame_id = fixed_frame.c_str();
	dmap_octree_pub.publish(output_pointcloud);

	// Publish grid map
	PointVector vis_occupied_points;
	dmap_->occupied_pointcloud_visualize(vis_occupied_points);
	PointClouds occupied_vis;
	sensor_msgs::PointCloud2 output_occupied;
	occupied_vis.clear();
	occupied_vis.points.assign(vis_occupied_points.begin(), vis_occupied_points.end());
	pcl::toROSMsg(occupied_vis, output_occupied);
	output_occupied.header.stamp = ros::Time::now();
	output_occupied.header.frame_id = fixed_frame.c_str();
	dmap_gridmap_pub.publish(output_occupied);

	// Depthmap Visualization
	int min_phi = dmap_->cur_depthmap.min_phi;
	int max_phi = dmap_->cur_depthmap.max_phi;
	int min_theta = dmap_->cur_depthmap.min_theta;
	int max_theta = dmap_->cur_depthmap.max_theta;
	if (max_phi >= min_phi && max_theta >= min_theta) {
		int idx = 0, len_phi = 0, len_theta = 0;
		len_phi = max_phi - min_phi + 1;
		len_theta = max_theta - min_theta + 1;
		sensor_msgs::Image output_depthmap;
		output_depthmap.height = len_phi;
		output_depthmap.width = len_theta;
		output_depthmap.header.stamp = ros::Time::now();
		output_depthmap.encoding = "mono8";
		output_depthmap.data.resize(len_phi * len_theta);
		double value = 0.0f;
		for (int i = min_phi; i <= max_phi; i++) {
			for (int j = min_theta; j <= max_theta; j++) {
				idx = (len_phi - (i - min_phi) - 1) * len_theta + (len_theta - (j - min_theta) - 1);
				value = dmap_->cur_depthmap.DepthMap(i, j);
				if (value < 0) {
					value = FOV_DEPTH - 0.1;
				} else if (value > FOV_DEPTH) {
					value = 0.0f;
				}
				// printf("ori value: %0.3f, fov_depth: %0.3f, floor value: %f\n",dmap_->DepthMap(i,j), FOV_DEPTH, floor(value/FOV_DEPTH * 256));
				output_depthmap.data[idx] = floor(value / FOV_DEPTH * 256);
			}
		}
		depthmap_pub.publish(output_depthmap);
	}

	// Publish Bounding box
	BoxPointType bbx = dmap_->GetBBX();
	std_msgs::ColorRGBA color;
	// Write orange RGB to color
	color.r = 1.0;
	color.g = 0.5;
	color.b = 0.0;
	color.a = 1.0;
	Vector3f size, vis_pos_world;
	for (int i = 0; i < 3; i++) {
		size(i) = (bbx.vertex_max[i] - bbx.vertex_min[i]) / 2.0;
		vis_pos_world(i) = (bbx.vertex_max[i] + bbx.vertex_min[i]) / 2.0;
	}
	float width = size.x();
	float length = size.y();
	float hight = size.z();
	visualization_msgs::MarkerArray mkrarr;
	static int id = 0;
	visualization_msgs::Marker line_strip;
	line_strip.header.stamp = ros::Time::now();
	line_strip.header.frame_id = fixed_frame.c_str();
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	// line_strip.id = id++;									  // unique id, useful when multiple markers exist.
	line_strip.type = visualization_msgs::Marker::LINE_STRIP; // marker type
	line_strip.scale.x = 0.1;

	line_strip.color = color;
	geometry_msgs::Point p[8];

	// vis_pos_world是目标物的坐标
	p[0].x = vis_pos_world(0) - width;
	p[0].y = vis_pos_world(1) + length;
	p[0].z = vis_pos_world(2) + hight;
	p[1].x = vis_pos_world(0) - width;
	p[1].y = vis_pos_world(1) - length;
	p[1].z = vis_pos_world(2) + hight;
	p[2].x = vis_pos_world(0) - width;
	p[2].y = vis_pos_world(1) - length;
	p[2].z = vis_pos_world(2) - hight;
	p[3].x = vis_pos_world(0) - width;
	p[3].y = vis_pos_world(1) + length;
	p[3].z = vis_pos_world(2) - hight;
	p[4].x = vis_pos_world(0) + width;
	p[4].y = vis_pos_world(1) + length;
	p[4].z = vis_pos_world(2) - hight;
	p[5].x = vis_pos_world(0) + width;
	p[5].y = vis_pos_world(1) - length;
	p[5].z = vis_pos_world(2) - hight;
	p[6].x = vis_pos_world(0) + width;
	p[6].y = vis_pos_world(1) - length;
	p[6].z = vis_pos_world(2) + hight;
	p[7].x = vis_pos_world(0) + width;
	p[7].y = vis_pos_world(1) + length;
	p[7].z = vis_pos_world(2) + hight;
	// LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
	for (int i = 0; i < 8; i++) {
		line_strip.points.push_back(p[i]);
	}
	// 为了保证矩形框的八条边都存在：
	line_strip.points.push_back(p[0]);
	line_strip.points.push_back(p[3]);
	line_strip.points.push_back(p[2]);
	line_strip.points.push_back(p[5]);
	line_strip.points.push_back(p[6]);
	line_strip.points.push_back(p[1]);
	line_strip.points.push_back(p[0]);
	line_strip.points.push_back(p[7]);
	line_strip.points.push_back(p[4]);
	mkrarr.markers.push_back(line_strip);
	bbx_pub.publish(mkrarr);
}

void CloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const geometry_msgs::PoseStampedConstPtr &pose) {
	last_updated = ros::Time::now();

	odom_q.pos(0) = pose->pose.position.x;
	odom_q.pos(1) = pose->pose.position.y;
	odom_q.pos(2) = pose->pose.position.z;
	odom_q.q.w() = pose->pose.orientation.w;
	odom_q.q.x() = pose->pose.orientation.x;
	odom_q.q.y() = pose->pose.orientation.y;
	odom_q.q.z() = pose->pose.orientation.z;
	odom_q.R = odom_q.q.matrix();

	odom_q.pos = odom_q.pos;

	pcl::PointCloud<pcl::PointXYZI> CloudsInput;
	pcl::fromROSMsg(*msg, CloudsInput);

	if (point_frame == "body") {
		for (int i = 0; i < CloudsInput.size(); i++) {
			Vector3f tmp_p(CloudsInput.points[i].x, CloudsInput.points[i].y, CloudsInput.points[i].z);
			Vector3f world_p = odom_q.pos + odom_q.R * (ext.R * tmp_p + ext.pos);
			CloudsInput.points[i].x = world_p(0);
			CloudsInput.points[i].y = world_p(1);
			CloudsInput.points[i].z = world_p(2);
		}
	}
	dmap_->UpdateMap(odom_q, CloudsInput);
	ros::Time start = ros::Time::now();
	if (VIS_EN) {
		Visualization();
		printf("Visualization Time :%0.3fs\n", (ros::Time::now() - start).toSec());
	}
}

void CloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::OdometryConstPtr &odom_input) {
	last_updated = ros::Time::now();

	odom_q.pos(0) = odom_input->pose.pose.position.x;
	odom_q.pos(1) = odom_input->pose.pose.position.y;
	odom_q.pos(2) = odom_input->pose.pose.position.z;
	Quaternionf q1(odom_input->pose.pose.orientation.w, odom_input->pose.pose.orientation.x, odom_input->pose.pose.orientation.y, odom_input->pose.pose.orientation.z);
	odom_q.q = q1;
	odom_q.R = odom_q.q.matrix();

	pcl::PointCloud<pcl::PointXYZI> CloudsInput;
	pcl::fromROSMsg(*msg, CloudsInput);

	if (point_frame == "body") {
		for (int i = 0; i < CloudsInput.size(); i++) {
			Vector3f tmp_p(CloudsInput.points[i].x, CloudsInput.points[i].y, CloudsInput.points[i].z);
			// Vector3f world_p = odom.pos + odom.R * (-imu_T + imu_R.transpose() * tmp_p);
			Vector3f world_p = odom_q.pos + odom_q.R * (ext.R * tmp_p + ext.pos);
			CloudsInput.points[i].x = world_p(0);
			CloudsInput.points[i].y = world_p(1);
			CloudsInput.points[i].z = world_p(2);
		}
	}

	odom_q.R = odom_q.R * ext.R;
	dmap_->UpdateMap(odom_q, CloudsInput);
	ros::Time start = ros::Time::now();
	if (VIS_EN) {
		Visualization();
		printf("Visualization Time :%0.3fs\n", (ros::Time::now() - start).toSec());
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_dmap");

	ros::NodeHandle nh("");

	std::string NameSpace = "DMap";
	nh.param<string>(NameSpace + "/point_frame", point_frame, "world");
	nh.param<string>(NameSpace + "/lidar_topic", lidar_topic, "");
	nh.param<string>(NameSpace + "/depth_topic", depth_topic, "");
	nh.param<string>(NameSpace + "/odom_topic", odom_topic, "");
	nh.param<string>(NameSpace + "/fixed_frame", fixed_frame, "world");
	nh.param<string>(NameSpace + "/odom_topic_type", odom_topic_type, "odometry");

	nh.param<std::vector<double>>(NameSpace + "/extrinsic_T", EXT_T, std::vector<double>({0.0, 0.0, 0.0}));
	nh.param<std::vector<double>>(NameSpace + "/extrinsic_q", EXT_Q, std::vector<double>({0.0, 0.0, 0.0, 1.0}));
	nh.param<double>(NameSpace + "/FOV_depth", FOV_DEPTH, 30.0);
	nh.param<bool>(NameSpace + "/vis_en", VIS_EN, true);
	nh.param<bool>(NameSpace + "/log_map", LOG_MAP, false);

	ext.pos = Eigen::Vector3f(EXT_T[0], EXT_T[1], EXT_T[2]);
	ext.q = Eigen::Quaternionf(EXT_Q[3], EXT_Q[0], EXT_Q[1], EXT_Q[2]);
	ext.R = ext.q.toRotationMatrix();

	ros::AsyncSpinner spinner(0);
	spinner.start();

	std::string root_dir;
	root_dir = ROOT_DIR;

	DMap::DMapConfig demap_cfg(nh);
	dmap_ = new DMap::dmap(demap_cfg);

	cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidar_topic.c_str(), 50));

	if (odom_topic_type == "odometry") {
		// Odom_sub = nh.subscribe(odom_topic.c_str(), 50, OdomCallback);
		odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic.c_str(), 50));
		sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
		sync_cloud_odom_->registerCallback(boost::bind(&CloudOdomCallback, _1, _2));
	} else if (odom_topic_type == "pose") {
		// Odom_sub = nh.subscribe(odom_topic.c_str(), 50, OdomCallback_Pose);
		pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, odom_topic.c_str(), 50));
		sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
		sync_cloud_pose_->registerCallback(boost::bind(&CloudPoseCallback, _1, _2));
	} else {
		assert(false && "ERROR in Odom Topic Type");
	}

	dmap_octree_pub = nh.advertise<sensor_msgs::PointCloud2>("/dmap_octree", 100);
	depthmap_pub = nh.advertise<sensor_msgs::Image>("/dmap_depthmap", 100);
	dmap_gridmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/demap_gridmap", 100);
	bbx_pub = nh.advertise<visualization_msgs::MarkerArray>("/bbx", 100);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/dmap_odom", 100);

	last_updated = ros::Time::now();
	ros::Rate rate(100);
	bool status = ros::ok();
	while (status) {
		status = ros::ok();
		ros::Time cur_time = ros::Time::now();
		if (LOG_MAP && (cur_time - last_updated).toSec() > 100.0) {
			break;
		}
		rate.sleep();
	}
	if (LOG_MAP) {
		string map_log, occupied_log;
		printf("[Node] Saving Map ...\n");
		map_log = root_dir + "/Log/demap_octree_" + log_name + ".txt";
		occupied_log = root_dir + "/Log/demap_gridmap_" + log_name + ".txt";

		dmap_->OutputMap(map_log, occupied_log);

		printf("[Node] Map Saved\n");
	}

	delete dmap_;
	ros::shutdown();
	return 0;
}