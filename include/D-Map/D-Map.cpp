#include <D-Map/D-Map.h>

namespace DMap {
	dmap::dmap(float resolution, float accuracy, float f_ratio) {
		/* Initilize Params Check */
		INITIALIZE = false;
		SENSOR_PARAMS_SET = false;

		/* Initialize params */
		Set_Resolution(resolution);
		Set_Ratio(f_ratio);
		Set_DepthMap_Accuracy(accuracy);

		/* Initialize Mutex Lock for Octree Map Updating & Visualization */
		pthread_mutex_init(&map_mutex_lock, NULL);

		/* counter for time log */
		time_log_cnt = 0;
	}

	dmap::dmap(DMapConfig &demap_cfg) {
		/* Initilize Params Check */
		INITIALIZE = false;
		SENSOR_PARAMS_SET = false;

		/* Initialize params */
		Set_Resolution(demap_cfg.MAP_RES_MIN);
		Set_Ratio(demap_cfg.FULL_RATIO);
		Set_DepthMap_Accuracy(demap_cfg.DEPTHMAP_ACCURACY);

		/* Initialize Mutex Lock for Octree Map Updating & Visualization */
		pthread_mutex_init(&map_mutex_lock, NULL);

		/* Init Sensor Params */
		Set_Sensor_Params(demap_cfg.FOV_shape, demap_cfg.FOV_THETA_RANGE, demap_cfg.FOV_PHI_RANGE,
						  demap_cfg.FOV_DEPTH, demap_cfg.SENSOR_RES_HOR, demap_cfg.SENSOR_RES_VERT);

		/* File name for logs */
		Set_Log_Name(demap_cfg.log_name);

		/* Enable printing detailed time consumption */
		Set_Print(demap_cfg.PRINT_EN);

		/* Enable Map Sliding function */
		Set_Sliding(demap_cfg.SLIDING_EN, demap_cfg.SLIDING_THRES);

		/* Init Map */
		Initialize(demap_cfg.env_box, demap_cfg.MAP_RES_INIT);

		/* counter for time log */
		time_log_cnt = 0;
	}

	dmap::~dmap() {
		segtree.clear();
		octree.clear();
		occupied_map.clear();
		pthread_mutex_destroy(&map_mutex_lock);
	}

	void dmap::Initialize(BoxPointType init_env, float L) {
		float len;
		int k = ceil(log2(L / map_res));
		len = map_res * pow(2, k);
		init_res = len;
		env_box = init_env;
		octree.SetBoundingBox(init_env);
		printf("Bounding Box: (%0.3f,%0.3f), (%0.3f,%0.3f), (%0.3f,%0.3f)\n", init_env.vertex_min[0], init_env.vertex_max[0],
			   init_env.vertex_min[1], init_env.vertex_max[1], init_env.vertex_min[2], init_env.vertex_max[2]);
		octree.InitByResolution(len);
		printf("Initial Len: %0.3f, Octree size: %d\n", len, octree.size());
		occupied_map.SetMapParams(map_res, init_env, FOV_depth);
		INITIALIZE = true;
	}

	float dmap::Idx2Rad(int idx, bool type) {
		float rad = 0.0;
		if (type) {
			for (size_t i = 0; i < polar_M.size(); i++) {
				if (idx >= polar_M[i])
					idx -= polar_M[i];
				else {
					rad = FOV_theta_range[i] + idx * polar_res_hor[i];
					break;
				}
			}
		} else {
			for (size_t i = 0; i < polar_N.size(); i++) {
				if (idx >= polar_N[i])
					idx -= polar_N[i];
				else {
					rad = FOV_phi_range[i] + idx * polar_res_vert[i];
					break;
				}
			}
		}
		return rad;
	}

	int dmap::Rad2Idx(float rad, bool type) {
		/*
			Type false: phi;
			Type true: theta;
		*/
		int idx = 0;
		if (type) {
			if (fabs(rad - FOV_theta_max) < polar_res_hor[polar_res_hor.size() - 1]) {
				idx = polar_width - 1;
			} else {
				for (size_t i = 1; i < FOV_theta_range.size(); i++) {
					if (rad + EPSS > FOV_theta_range[i]) {
						idx += polar_M[i - 1];
					} else {
						idx += floor((rad - FOV_theta_range[i - 1] - EPSS) / polar_res_hor[i - 1]);
						break;
					}
				}
			}
		} else {
			if (fabs(rad - FOV_phi_max) < polar_res_vert[polar_res_vert.size() - 1]) {
				idx = polar_height - 1;
			} else {
				for (size_t i = 1; i < FOV_phi_range.size(); i++) {
					if (rad + EPSS > FOV_phi_range[i]) {
						idx += polar_N[i - 1];
					} else {
						idx += floor((rad - FOV_phi_range[i - 1] - EPSS) / polar_res_vert[i - 1]);
						break;
					}
				}
			}
		}
		return idx;
	}

	void dmap::euc2polar(Vector3f &euc_pt, polar3D *polar_pt) {
		polar_pt->theta_rad = atan2(euc_pt[1], euc_pt[0]);
		polar_pt->phi_rad = atan2(euc_pt[2], euc_pt.head<2>().norm());
		polar_pt->r = euc_pt.norm();
		size_t i = 0;
		if (polar_pt->theta_rad > FOV_theta_max) {
			polar_pt->theta = polar_width;
		} else if (polar_pt->theta_rad < FOV_theta_min) {
			polar_pt->theta = -1;
		} else {
			polar_pt->theta = Rad2Idx(polar_pt->theta_rad, true);
		}
		if (polar_pt->phi_rad > FOV_phi_max) {
			polar_pt->phi = polar_height;
		} else if (polar_pt->phi_rad < FOV_phi_min) {
			polar_pt->phi = -1;
		} else {
			polar_pt->phi = Rad2Idx(polar_pt->phi_rad, false);
		}
		return;
	}

	void dmap::polar2euc(polar3D *polar_pt, Vector3f &euc_pt) {
		// trans from polar coordinate to euclidean coordinate
		float theta_rad = 0.0f;
		float phi_rad = 0.0f;
		size_t i;
		int cur_N = polar_pt->phi;
		int cur_M = polar_pt->theta;
		if (cur_M > polar_width) {
			theta_rad = FOV_theta_max + (cur_M - polar_width) * polar_res_hor[polar_res_hor.size() - 1];
		} else if (cur_M < 0) {
			theta_rad = FOV_theta_min + cur_M * polar_res_hor[0];
		} else {
			theta_rad = Idx2Rad(polar_pt->theta, true);
		}
		if (cur_N > polar_height) {
			phi_rad = FOV_phi_max + (cur_N - polar_height) * polar_res_vert[polar_res_vert.size() - 1];
		} else if (cur_N < 0) {
			phi_rad = FOV_phi_min + cur_N * polar_res_hor[0];
		} else {
			phi_rad = Idx2Rad(polar_pt->phi, false);
		}
		euc_pt[2] = polar_pt->r * sin(phi_rad);
		euc_pt[1] = polar_pt->r * cos(phi_rad) * sin(theta_rad);
		euc_pt[0] = polar_pt->r * cos(phi_rad) * cos(theta_rad);
	}

	string dmap::getCurrentTimestr() {
		time_t t = time(NULL);
		char ch[64] = {0};
		char result[100] = {0};
		strftime(ch, sizeof(ch) - 1, "%Y%m%d_%H%M%S", localtime(&t));
		sprintf(result, "%s", ch);
		return std::string(result);
	}

	void dmap::Set_Log_Name(string log_name) {
		LOG_EN = true;
		log_dir = log_name + ".csv";
		/* Data Log */
		fp = fopen(log_dir.c_str(), "w");
		fprintf(fp, "total time, depthmap, update, segtree, occupied octree\n");
		fclose(fp);
	}

	void dmap::Set_Print(bool en) {
		PRINT_EN = en;
	}

	void dmap::Set_Sliding(bool en, double thres) {
		SLIDING_EN = en;
		sliding_thres = thres;
	}

	void dmap::Set_Sensor_Params(uint8_t shape, std::vector<double> &fov_theta_range, std::vector<double> &fov_phi_range,
								 float fov_depth, std::vector<double> &res_hor, std::vector<double> res_vert) {
		/* Sensor Params */
		FOV_theta_range.clear();
		FOV_phi_range.clear();
		sensor_res_hor.clear();
		sensor_res_vert.clear();
		FOV_shape = shape;
		size_t i;
		for (i = 0; i < fov_theta_range.size(); i++)
			FOV_theta_range.push_back(DegToRad(fov_theta_range[i]));
		for (i = 0; i < fov_phi_range.size(); i++)
			FOV_phi_range.push_back(DegToRad(fov_phi_range[i]));
		for (i = 0; i < res_hor.size(); i++)
			sensor_res_hor.push_back(DegToRad(res_hor[i]));
		for (i = 0; i < res_vert.size(); i++)
			sensor_res_vert.push_back(DegToRad(res_vert[i]));
		FOV_theta_min = FOV_theta_range[0];
		FOV_theta_max = FOV_theta_range[FOV_theta_range.size() - 1];
		FOV_phi_min = FOV_phi_range[0];
		FOV_phi_max = FOV_phi_range[FOV_phi_range.size() - 1];
		if (FOV_theta_max - FOV_theta_min > 2 * M_PI - EPSS)
			THETA_360 = true;
		if (FOV_phi_max - FOV_phi_min > 2 * M_PI - EPSS)
			PHI_360 = true;

		FOV_depth = fov_depth;
		SENSOR_PARAMS_SET = true;

		/* Depthmap params */
		Calc_DepthMap_Resolution();
		Calc_DepthMap_Size();
		segtree.SetSize(polar_height, polar_width);

		/* Octree params */
		octree.SetResolution(map_res);
		octree.Set_Sensor_Params(FOV_theta_range, FOV_phi_range, FOV_depth, polar_res_hor, polar_res_vert);

		/* Empty Matrix for initialization */
		newMat = MatrixXf::Zero(polar_height, polar_width);
		newMat.setConstant(-1.0);
		newMati = MatrixXi::Zero(polar_height, polar_width);
		newMati.setConstant(0);
	}

	void dmap::Set_Resolution(float resolution) {
		map_res = resolution;
	}

	void dmap::Set_Ratio(float f_ratio) {
		full_ratio = f_ratio;
		octree.SetFullRatio(full_ratio);
	}

	void dmap::Set_DepthMap_Resolution(std::vector<double> &res_hor, std::vector<double> &res_vert) {
		/* Depthmap params */
		size_t i;
		polar_res_hor.clear();
		polar_res_vert.clear();
		for (i = 0; i < res_hor.size(); i++)
			polar_res_hor.push_back(DegToRad(res_hor[i]));
		for (i = 0; i < res_vert.size(); i++)
			polar_res_vert.push_back(DegToRad(res_vert[i]));
		if (polar_res_hor.size() < FOV_theta_range.size() - 1) {
			float res_ext = polar_res_hor[polar_res_hor.size() - 1];
			for (i = polar_res_hor.size(); i < FOV_theta_range.size() - 1; i++)
				polar_res_hor.push_back(res_ext);
		}
		if (polar_res_vert.size() < FOV_phi_range.size() - 1) {
			float res_ext = polar_res_vert[polar_res_vert.size() - 1];
			for (i = polar_res_vert.size(); i < FOV_phi_range.size() - 1; i++)
				polar_res_vert.push_back(res_ext);
		}
		printf("[Depth Map] Resolution: [");
		for (i = 0; i < polar_res_vert.size(); i++) {
			if (i == 0)
				printf("%0.3f", polar_res_vert[i] / M_PI * 180);
			else
				printf(",%0.3f", polar_res_vert[i] / M_PI * 180);
		}
		printf("](deg) x [");
		for (i = 0; i < polar_res_hor.size(); i++) {
			if (i == 0)
				printf("%0.3f", polar_res_hor[i] / M_PI * 180);
			else
				printf(",%0.3f", polar_res_hor[i] / M_PI * 180);
		}
		printf("](deg)\n");
		Calc_DepthMap_Size();
		segtree.SetSize(polar_height, polar_width);
		/* Octree params */
		octree.Set_Sensor_Params(FOV_theta_range, FOV_phi_range, FOV_depth, polar_res_hor, polar_res_vert);
		/* Empty Matrix for initialization */
		newMat = MatrixXf::Zero(polar_height, polar_width);
		newMati = MatrixXi::Zero(polar_height, polar_width);
		newMat.setConstant(-1.0);
		newMati.setConstant(0);
	}

	void dmap::Get_Depthmap_Resolution(std::vector<float> &res_hor, std::vector<float> &res_vert) {
		res_hor.clear();
		res_vert.clear();
		for (size_t i = 0; i < polar_res_hor.size(); i++)
			res_hor.push_back(polar_res_hor[i]);
		for (size_t i = 0; i < polar_res_vert.size(); i++)
			res_vert.push_back(polar_res_vert[i]);
		return;
	}

	void dmap::Set_DepthMap_Accuracy(float Accuracy) {
		if (Accuracy > 1.0f)
			Accuracy = 1.0f;
		if (Accuracy < 0.0f)
			Accuracy = 0.0f;
		depthmap_accuracy = Accuracy;
		if (SENSOR_PARAMS_SET) {
			printf("Accuracy Reset. Depth Map Params are updated as follows:");
			Calc_DepthMap_Resolution();
			Calc_DepthMap_Size();
		}
	}

	void dmap::Calc_DepthMap_Resolution() {
		std::vector<float> res_hor_std, res_vert_std;
		float res_hor_from_map, res_vert_from_map;
		float max_phi, A, L;
		float a, b, c, d, p, q, r, theta, r_3;
		float x[3], k = 1;

		max_phi = max(fabs(FOV_phi_min), fabs(FOV_phi_max));
		if (depthmap_accuracy < 1 - EPSS) {
			A = (3.0f * (1.0f - cos(max_phi)) + 12.0f / M_PI * sin(max_phi)) / (1.0f - 0.5f * sin(max_phi) * pow(cos(max_phi), 2) - 0.5f * pow(1 - sin(max_phi), 2) * (2 + sin(max_phi)));
			L = pow(3.0f, 0.5f) * sin(min(atan(1 / pow(2.0f, 0.5f)) + max_phi, float(M_PI_2)));
			a = depthmap_accuracy;
			b = 0;
			c = -A;
			d = A * L - pow(L, 3);
			p = (3 * a * c - b * b) / (3 * a * a);
			q = (27 * a * a * d - 9 * a * b * c + 2 * b * b * b) / (27 * a * a * a);
			r = pow(-p * p * p / 27, 0.5);
			r_3 = pow(r, 1.0f / 3);
			theta = 1.0 / 3 * acos(-q / 2 / r);
			x[0] = 2 * r_3 * cos(theta);
			x[1] = 2 * r_3 * cos(theta + 2 * M_PI / 3.0f);
			x[2] = 2 * r_3 * cos(theta + 4 * M_PI / 3.0f);
			for (int i = 0; i < 3; i++) {
				if (x[i] >= L - EPSS) {
					k = x[i];
					break;
				}
			}
		} else {
			k = 1.0;
		}

		printf("Relaxed Factor: %0.3f\n", k);
		polar_res_hor.clear();
		polar_res_hor.resize(sensor_res_hor.size());
		polar_res_vert.clear();
		polar_res_vert.resize(sensor_res_vert.size());
		res_hor_from_map = 2 * atan(map_res / 2.0f / FOV_depth);
		res_vert_from_map = 2 * atan(map_res / 2.0f / FOV_depth);
		size_t i;
		for (i = 0; i < polar_res_hor.size(); i++)
			polar_res_hor[i] = k * max(res_hor_from_map, sensor_res_hor[i]);
		for (i = 0; i < polar_res_vert.size(); i++)
			polar_res_vert[i] = k * max(res_vert_from_map, sensor_res_vert[i]);
		if (polar_res_hor.size() < FOV_theta_range.size() - 1) {
			float res_ext = polar_res_hor[polar_res_hor.size() - 1];
			for (i = polar_res_hor.size(); i < FOV_theta_range.size() - 1; i++)
				polar_res_hor.push_back(res_ext);
		}
		if (polar_res_vert.size() < FOV_phi_range.size() - 1) {
			float res_ext = polar_res_vert[polar_res_vert.size() - 1];
			for (i = polar_res_vert.size(); i < FOV_phi_range.size() - 1; i++)
				polar_res_vert.push_back(res_ext);
		}
		printf("[Depth Map] Resolution: [");
		for (i = 0; i < polar_res_vert.size(); i++) {
			if (i == 0)
				printf("%0.3f", polar_res_vert[i] / M_PI * 180);
			else
				printf(",%0.3f", polar_res_vert[i] / M_PI * 180);
		}
		printf("](deg) x [");
		for (i = 0; i < polar_res_hor.size(); i++) {
			if (i == 0)
				printf("%0.3f", polar_res_hor[i] / M_PI * 180);
			else
				printf(",%0.3f", polar_res_hor[i] / M_PI * 180);
		}
		printf("](deg)\n");
		return;
	}

	void dmap::Calc_DepthMap_Size() {
		polar_width = 0;
		polar_height = 0;
		size_t i = 0;
		int len;
		polar_N.clear();
		polar_M.clear();
		for (i = 1; i < FOV_theta_range.size(); i++) {
			len = ceil((FOV_theta_range[i] - FOV_theta_range[i - 1] - EPSS) / polar_res_hor[i - 1]);
			polar_M.push_back(len);
			polar_width += len;
		}
		for (i = 1; i < FOV_phi_range.size(); i++) {
			len = ceil((FOV_phi_range[i] - FOV_phi_range[i - 1] - EPSS) / polar_res_vert[i - 1]);
			polar_N.push_back(len);
			polar_height += len;
		}
		printf("[Depth Map] Size: %d x %d\n", polar_height, polar_width);
	}

	void dmap::GenerateDepthMap(PointClouds &new_clouds, OdomType odom, DepthmapType &map) {
		// Common Variables
		Vector3f dir_vec, world_p;
		polar3D polar_pt;
		float theta_rad, phi_rad;

		// Initialize the depthmap and project points on it.
		map.DepthMap = newMat;
		map.odom = odom;
		for (int i = 0; i < 2; i++)
			map.Polars[i] = newMat;

		for (size_t i = 0; i < new_clouds.size(); i++) {
			world_p.x() = new_clouds[i].x;
			world_p.y() = new_clouds[i].y;
			world_p.z() = new_clouds[i].z;
			dir_vec = odom.R.transpose() * (world_p - odom.pos);
			if (dir_vec.norm() > 0) {
				euc2polar(dir_vec, &polar_pt);
				if (polar_pt.r < 0 || polar_pt.theta > (polar_width - 1) || polar_pt.theta < 0 || polar_pt.phi > (polar_height - 1) || polar_pt.phi < 0) { // || polar_pt.r > FOV_depth
					continue;
				}
				if (map.DepthMap(polar_pt.phi, polar_pt.theta) < 0 || map.DepthMap(polar_pt.phi, polar_pt.theta) > polar_pt.r) {
					map.DepthMap(polar_pt.phi, polar_pt.theta) = polar_pt.r;
					map.Polars[0](polar_pt.phi, polar_pt.theta) = polar_pt.phi_rad;
					map.Polars[1](polar_pt.phi, polar_pt.theta) = polar_pt.theta_rad;
				}
				map.max_theta = max(map.max_theta, polar_pt.theta);
				map.min_theta = min(map.min_theta, polar_pt.theta);
				map.max_phi = max(map.max_phi, polar_pt.phi);
				map.min_phi = min(map.min_phi, polar_pt.phi);
			}
		}
	}

	BoxPointType dmap::GetBBX() {
		return env_box;
	}

	void dmap::SlideMap(OdomType odom) {
		if (!center_initialized) {
			map_center = odom.pos;
			center_initialized = true;
		} else {
			Vector3f diff = odom.pos - map_center;
			for (int i = 0; i < 3; i++) {
				if (fabs(diff(i)) <= sliding_thres) {
					diff(i) = 0;
				} else {
					diff(i) = ceil(diff(i) / map_res + EPSS) * map_res;
				}
			}
			if (fabs(diff(0)) <= sliding_thres) {
				diff(0) = 0.0;
			}
			if (fabs(diff(1)) <= sliding_thres) {
				diff(1) = 0.0;
			}
			if (fabs(diff(2)) <= sliding_thres) {
				diff(2) = 0.0;
			}

			if (diff.norm() > EPSS) {
				BoxPointType new_bbx = env_box;
				for (int i = 0; i < 3; i++) {
					new_bbx.vertex_min[i] += diff(i);
					new_bbx.vertex_max[i] += diff(i);
				}

				map_center = map_center + diff;
				octree.SlideMap(new_bbx);
				occupied_map.SlideMap(new_bbx);
				env_box = new_bbx;
			}
		}
	}

	void dmap::UpdateMap(OdomType odom, PointClouds &pointclouds) {
		assert(SENSOR_PARAMS_SET && "MAP PARAMS NOT SET");
		assert(INITIALIZE && "MAP NOT INITIALIZED");
		// printf("----------------------------\n");
		// std::cout << "pos: " << odom.pos.transpose() << std::endl;
		// std::cout << "Rot: " << odom.R << std::endl;
		if (pointclouds.size() == 0)
			return;

		/* Time logger*/
		TimePoint all_start, all_end, depthmap_start, depthmap_end, box_start, box_end;
		TimePoint update_start, update_end, segtree_start, segtree_end, occ_start, occ_end;
		TimePoint sliding_start, sliding_end;
		nanoseconds duration_update, duration_depthmap, duration_sliding;
		nanoseconds duration_box, duration_seg, duration_all, duration_occ;
		double update_ms, depthmap_ms, box_ms, seg_ms, all_ms, occ_ms, sliding_ms;
		duration_depthmap.zero();
		duration_box.zero();
		duration_all.zero();

		all_start = high_resolution_clock::now();

		/* Map Sliding */
		sliding_start = high_resolution_clock::now();
		if (SLIDING_EN)
			SlideMap(odom);

		sliding_end = high_resolution_clock::now();
		duration_sliding = duration_cast<nanoseconds>(sliding_end - sliding_start);
		sliding_ms = double(duration_sliding.count()) * 1e-6;

		/* Point Cloud to Depth Map*/
		depthmap_start = high_resolution_clock::now();

		GenerateDepthMap(pointclouds, odom, cur_depthmap);

		depthmap_end = high_resolution_clock::now();
		duration_depthmap = duration_cast<nanoseconds>(depthmap_end - depthmap_start);
		init_odom = cur_depthmap.odom;

		/* Initialize segment tree */
		box_start = high_resolution_clock::now();
		segtree_start = high_resolution_clock::now();
		segtree.init(cur_depthmap.DepthMap);
		segtree_end = high_resolution_clock::now();
		duration_seg = duration_cast<nanoseconds>(segtree_end - segtree_start);

		/* Update Octree */
		update_start = high_resolution_clock::now();
		FoVType init_FoV(FOV_shape, init_odom.pos, init_odom.R,
						 FOV_theta_max - FOV_theta_min, FOV_theta_min, FOV_phi_max - FOV_phi_min, FOV_phi_min, FOV_depth);
		pthread_mutex_lock(&map_mutex_lock);
		octree.UpdateTree(&segtree, init_FoV, &cur_depthmap);
		pthread_mutex_unlock(&map_mutex_lock);
		// printf("DEBUG CNT: %d\n",octree.debug_cnt);
		update_end = high_resolution_clock::now();
		duration_update = duration_cast<nanoseconds>(update_end - update_start);

		box_end = high_resolution_clock::now();
		duration_box = duration_cast<nanoseconds>(box_end - box_start);

		/* Insert LiDAR points into grid map (occupied) */

		occ_start = high_resolution_clock::now();

		occupied_map.AddPoints(odom, pointclouds.points);

		occ_end = high_resolution_clock::now();
		duration_occ = duration_cast<nanoseconds>(occ_end - occ_start);

		all_end = high_resolution_clock::now();
		duration_all = duration_cast<nanoseconds>(all_end - all_start);

		/* Time analysis*/
		all_ms = double(duration_all.count()) * 1e-6;
		occ_ms = double(duration_occ.count()) * 1e-6;
		depthmap_ms = double(duration_depthmap.count()) * 1e-6;
		box_ms = double(duration_box.count()) * 1e-6;
		seg_ms = double(duration_seg.count()) * 1e-6;
		update_ms = double(duration_update.count()) * 1e-6;
		time_log_cnt++;
		average_time = average_time / time_log_cnt * (time_log_cnt - 1) + all_ms / time_log_cnt;
		if (PRINT_EN) {
			printf("----------------------------\n");
			printf("\nTime consumption: %0.3fms\n", all_ms);
			printf("Average Time: %0.3fms\n", average_time);
			printf("    Occupied Tree:   %0.3fms, %0.3f%% with size: %d\n", occ_ms, occ_ms / all_ms * 100, occupied_map.size());
			printf("    Depthmap Update: %0.3fms, %0.3f%%\n", depthmap_ms, depthmap_ms / all_ms * 100);
			printf("    Main Process:    %0.3fms, %0.3f%%\n", box_ms, box_ms / all_ms * 100);
			printf("            SegmentTree:     %0.3fms, %0.3f%%\n", seg_ms, seg_ms / all_ms * 100);
			printf("            Octree Update:   %0.3fms, %0.3f%%\n", update_ms, update_ms / all_ms * 100);
			printf("            Sum:             %0.3fms, %0.3f%%\n", seg_ms + update_ms, (seg_ms + update_ms) / all_ms * 100);
			printf("    octree map size: %d\n", octree.size());
			printf("----------------------------\n");
		} else {
			printf("[DeMap] Processing Time: %0.3fms, Average Time: %0.3f\n", all_ms, average_time);
		}

		if (LOG_EN) {
			fp = fopen(log_dir.c_str(), "a");
			fprintf(fp, "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n", all_ms, depthmap_ms, update_ms, seg_ms, occ_ms);
			fclose(fp);
		}
	}

	void dmap::OutputMap(string octree_log, string gridmap_log) {
		FILE *octree_fp = fopen(octree_log.c_str(), "w");
		fprintf(octree_fp, "%0.3f\n", map_res);
		fprintf(octree_fp, "(%0.3f,%0.3f),(%0.3f,%0.3f),(%0.3f,%0.3f)\n", env_box.vertex_min[0], env_box.vertex_max[0],
				env_box.vertex_min[1], env_box.vertex_max[1], env_box.vertex_min[2], env_box.vertex_max[2]);
		octree.OutputMap(octree_fp);
		fclose(octree_fp);
		printf("[DeMap] Octree Output Done\n");

		FILE *gridmap_fp = fopen(gridmap_log.c_str(), "w");
		fprintf(gridmap_fp, "%0.3f\n", map_res);
		fprintf(gridmap_fp, "(%0.3f,%0.3f),(%0.3f,%0.3f),(%0.3f,%0.3f)\n", env_box.vertex_min[0], env_box.vertex_max[0],
				env_box.vertex_min[1], env_box.vertex_max[1], env_box.vertex_min[2], env_box.vertex_max[2]);
		occupied_map.OutputMap(gridmap_fp);
		fclose(gridmap_fp);
		printf("[DeMap]: Grid Map Output Done\n");
	}

	void dmap::unknown_pointcloud_visualize(PointVector &vis_pointclouds) {
		vis_pointclouds.clear();
		octree.RetrieveUnknownPoints(octree.Root_Node_, vis_pointclouds, octree.Root_BBX_);
	}

	void dmap::occupied_pointcloud_visualize(PointVector &pointclouds) {
		pointclouds.clear();
		occupied_map.RetrievePointCenters(pointclouds);
	}

	bool dmap::insideEnvBox(Vector3d p) {
		if (p.x() > env_box.vertex_max[0] + EPSS || p.x() < env_box.vertex_min[0] - EPSS)
			return false;
		if (p.y() > env_box.vertex_max[1] + EPSS || p.y() < env_box.vertex_min[1] - EPSS)
			return false;
		if (p.z() > env_box.vertex_max[2] + EPSS || p.z() < env_box.vertex_min[2] - EPSS)
			return false;
		return true;
	}

	bool dmap::CheckOccupied(Vector3d pos) {
		if (!insideEnvBox(pos))
			return false;
		PointType p;
		p.x = pos.x();
		p.y = pos.y();
		p.z = pos.z();
		return occupied_map.CheckOccupied(p);
	}

	bool dmap::CheckUnknown(Vector3d pos) {
		if (!insideEnvBox(pos))
			return true;
		PointType p;
		p.x = pos.x();
		p.y = pos.y();
		p.z = pos.z();
		return octree.CheckUnknown(octree.Root_Node_, p, octree.Root_BBX_);
	}

	double dmap::getResolution() {
		double res = map_res;
		return res;
	}

	size_t dmap::memUsage() {
		size_t memUsageOctree = octree.GetMemory();
		size_t memUsageGridmap = occupied_map.GetMemory();
		return (memUsageOctree + memUsageGridmap);
	}

	double dmap::getMemoryUsage_KB() {
		return (double(memUsage()) / 1024.0);
	}

	double dmap::getMemoryUsage_MB() {
		return (double(memUsage()) / 1024.0 / 1024.0);
	}

	double dmap::getMemoryUsage_GB() {
		return (double(memUsage()) / 1024.0 / 1024.0 / 1024.0);
	}
} // namespace DMap