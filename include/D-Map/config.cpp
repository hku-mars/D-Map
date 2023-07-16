#include <D-Map/config.h>

namespace DMap {

	DMapConfig::DMapConfig() {}

	DMapConfig::DMapConfig(const ros::NodeHandle nh, string name_space) {
		nh_ = nh;
		LoadParam<int>(name_space + "/FOV_shape", FOV_shape, 0);
		LoadParam<double>(name_space + "/FOV_theta_range", FOV_THETA_RANGE, std::vector<double>());
		LoadParam<double>(name_space + "/FOV_phi_range", FOV_PHI_RANGE, std::vector<double>());
		LoadParam<double>(name_space + "/sensor_res_hor", SENSOR_RES_HOR, std::vector<double>());
		LoadParam<double>(name_space + "/sensor_res_vert", SENSOR_RES_VERT, std::vector<double>());
		LoadParam<double>(name_space + "/FOV_depth", FOV_DEPTH, 30.0);

		LoadParam<double>(name_space + "/map_res_min", MAP_RES_MIN, 0.1);
		LoadParam<double>(name_space + "/map_res_init", MAP_RES_INIT, 10.0);
		LoadParam<double>(name_space + "/full_ratio", FULL_RATIO, 0.8);
		LoadParam<double>(name_space + "/depthmap_accuracy", DEPTHMAP_ACCURACY, 0.9);
		LoadParam<double>(name_space + "/environment", ENV, std::vector<double>());

		LoadParam<bool>(name_space + "/print_en", PRINT_EN, false);
		LoadParam<string>(name_space + "/log_name", log_name, "test");
		LoadParam<bool>(name_space + "/sliding_en", SLIDING_EN, false);
		LoadParam<double>(name_space + "/sliding_thres", SLIDING_THRES, 10.0);

		for (int axis = 0; axis < 3; axis++) {
			env_box.vertex_min[axis] = ENV[axis];
			env_box.vertex_max[axis] = ENV[3 + axis];
		}
	}

	DMapConfig::~DMapConfig() {}

	template <class T>
	bool DMapConfig::LoadParam(string param_name, T &param_value, T default_value) {
		if (nh_.getParam(param_name, param_value)) {
			fmt::print(fg(fmt::color::green), "[DemapConfig] Load param: {} = {}\n", param_name, param_value);
			return true;
		} else {
			param_value = default_value;
			fmt::print(fg(fmt::color::red), "[DemapConfig] Load failed, use default param: {} = {}\n", param_name, param_value);
			return false;
		}
	}

	template <class T>
	bool DMapConfig::LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
		if (nh_.getParam(param_name, param_value)) {
			fmt::print(fg(fmt::color::green), "[DemapConfig] Load param: {} = [", param_name);
			for (size_t i = 0; i < param_value.size(); i++) {
				if (i == param_value.size() - 1)
					fmt::print(fg(fmt::color::green), "{}]\n", param_value[i]);
				else
					fmt::print(fg(fmt::color::green), "{}, ", param_value[i]);
			}
			return true;
		} else {
			param_value = default_value;
			fmt::print(fg(fmt::color::red), "[DemapConfig] Load failed, use default param: {} = [", param_name);
			for (size_t i = 0; i < param_value.size(); i++) {
				if (i == param_value.size() - 1)
					fmt::print(fg(fmt::color::red), "{}]\n", param_value[i]);
				else
					fmt::print(fg(fmt::color::red), "{}, ", param_value[i]);
			}
			return false;
		}
	}
} // namespace DMap
