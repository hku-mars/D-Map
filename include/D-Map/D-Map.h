#include <chrono>
#include <pthread.h>
#include <string.h>

#include <D-Map/GridMap.h>
#include <D-Map/config.h>
#include <D-Map/lib_common.h>
#include <D-Map/octree_map.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace DMap {
	class dmap {
	private:
		DMap::octree_map<PointType> octree;
		DMap::SegmentTree segtree;
		DMap::gridmap<PointType> occupied_map;

		// Demap Params
		float full_ratio = 0.8;
		float depthmap_accuracy = 0.8;
		int polar_height;
		int polar_width;

		bool SENSOR_PARAMS_SET = false;
		bool INITIALIZE = false;
		bool LOG_EN = false;
		bool PRINT_EN = false;

		uint8_t FOV_shape = 0;

		// Map Resolution
		float init_res = 5.0;
		float map_res = 0.1;

		// Map Sliding
		bool center_initialized = false;
		bool SLIDING_EN = false;
		float sliding_thres = 10.0;
		void MapSliding();
		Vector3f map_center;

		// Sensor Resolution
		std::vector<float> sensor_res_hor, sensor_res_vert;
		std::vector<float> FOV_theta_range, FOV_phi_range;
		std::vector<float> polar_res_hor, polar_res_vert;
		std::vector<int> polar_N, polar_M;
		float FOV_depth;
		float FOV_phi_min, FOV_phi_max;
		float FOV_theta_min, FOV_theta_max;
		bool THETA_360 = false, PHI_360 = false;

		pthread_mutex_t map_mutex_lock;
		MatrixXf newMat;
		MatrixXi newMati;

		// Time log
		int time_log_cnt = 0;
		double average_time = 0.0;
		FILE *fp;
		string root_dir = ROOT_DIR;
		string log_dir = root_dir + "/Log/dmap_log_test.csv";

		static void *depthmap_thread_ptr(void *arg);
		void Calc_DepthMap_Size();
		void Calc_DepthMap_Resolution();
		void build_depthmap();
		void GenerateDepthMap(PointClouds &new_clouds, OdomType odom, DepthmapType &map); // Modified to PointClouds
		void SlideMap(OdomType odom);
		bool insideEnvBox(Vector3d p);

		int Rad2Idx(float rad, bool type);
		float Idx2Rad(int idx, bool type);
		size_t memUsage();

	public:
		BoxPointType env_box;
		DepthmapType cur_depthmap;
		OdomType init_odom;

		dmap(float resolution, float accuracy, float f_ratio);
		dmap(DMapConfig &demap_cfg);
		~dmap();
		void UpdateMap(OdomType odom, PointClouds &pointclouds);
		void Initialize(BoxPointType env_box, float len);
		void OutputMap(string octree_log, string gridmap_log);
		void unknown_pointcloud_visualize(PointVector &pointclouds);
		void occupied_pointcloud_visualize(PointVector &pointclouds);
		void Set_Sensor_Params(uint8_t shape, std::vector<double> &fov_theta_range, std::vector<double> &fov_phi_range,
							   float fov_depth, std::vector<double> &res_hor, std::vector<double> res_vert);
		void Set_Resolution(float resolution);
		void Set_Ratio(float f_ratio);
		void Set_DepthMap_Resolution(std::vector<double> &res_hor, std::vector<double> &res_vert);
		void Set_DepthMap_Accuracy(float accuracy);
		void Set_Log_Name(string log_name);
		void Set_Print(bool en);
		void Set_Sliding(bool en, double thres = 10.0);
		void euc2polar(Vector3f &euc_pt, polar3D *polar_pt);
		void polar2euc(polar3D *polar_pt, Vector3f &euc_pt);
		void Get_Depthmap_Resolution(std::vector<float> &res_hor, std::vector<float> &res_vert);
		BoxPointType GetBBX();
		string getCurrentTimestr();
		double getMemoryUsage_KB();
		double getMemoryUsage_MB();
		double getMemoryUsage_GB();

		double getResolution();
		bool CheckUnknown(Vector3d pos);
		bool CheckOccupied(Vector3d pos);
	};
} // namespace DMap