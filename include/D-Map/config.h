#include <D-Map/lib_common.h>
#include <fmt/color.h>
#include <ros/ros.h>

namespace DMap {
	class DMapConfig {
	public:
		DMapConfig();
		DMapConfig(const ros::NodeHandle nh, string name_space = "DMap");
		~DMapConfig();

		/* LiDAR params*/
		std::vector<double> FOV_THETA_RANGE;
		std::vector<double> FOV_PHI_RANGE;
		std::vector<double> SENSOR_RES_HOR;
		std::vector<double> SENSOR_RES_VERT;
		double FOV_DEPTH = 30.0;
		int FOV_shape = 0;

		/* Map Settings */
		double MAP_RES_MIN = 0.1;
		double MAP_RES_INIT = 10.0;
		double SLIDING_THRES = 10.0;
		std::vector<double> ENV;
		BoxPointType env_box;

		/* Decremental Map Settings */
		double FULL_RATIO = 0.8;
		double DEPTHMAP_ACCURACY = 0.9;
		bool PRINT_EN = false;
		bool SLIDING_EN = false;

		/* Log Settings */
		string log_name;

	private:
		ros::NodeHandle nh_;

		template <class T>
		bool LoadParam(string param_name, T &param_value, T default_value);
		template <class T>
		bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);
	};

} // namespace DMap