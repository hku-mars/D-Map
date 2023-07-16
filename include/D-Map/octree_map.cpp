#include "octree_map.h"

namespace DMap {
	template <typename PointType>
	octree_map<PointType>::octree_map() {
		Resolution_SET = false;
		BoundingBox_SET = false;
		FullRatio_SET = false;
		Map_Param_SET = false;
	}

	template <typename PointType>
	octree_map<PointType>::octree_map(float res) {
		Resolution_SET = false;
		BoundingBox_SET = false;
		SetResolution(res);
	}

	template <typename PointType>
	octree_map<PointType>::octree_map(BoxPointType box) {
		Resolution_SET = false;
		BoundingBox_SET = false;
		SetBoundingBox(box);
	}

	template <typename PointType>
	octree_map<PointType>::octree_map(float res, BoxPointType box) {
		Resolution_SET = false;
		BoundingBox_SET = false;
		SetResolution(res);
		SetBoundingBox(box);
	}

	template <typename PointType>
	octree_map<PointType>::~octree_map() {
		DeleteNodes(&Root_Node_);
		printf("[octree_map] Map deleted\n");
	}

	template <typename PointType>
	int octree_map<PointType>::size() {
		return (Root_Node_->sizeN);
	}

	template <typename PointType>
	void octree_map<PointType>::clear() {
		DeleteNodes(&Root_Node_);
	}

	template <typename PointType>
	void octree_map<PointType>::SetResolution(float res) {
		resolution = res;
		Resolution_SET = true;
		printf("[octree_map] Set Resolution: %0.6f\n", res);
	}

	template <typename PointType>
	void octree_map<PointType>::SetBoundingBox(BoxPointType box) {
		BoundingBox = box;
		BoundingBox_SET = true;
	}

	template <typename PointType>
	void octree_map<PointType>::SetFullRatio(float rate) {
		full_ratio = rate;
		FullRatio_SET = true;
	}

	template <typename PointType>
	void octree_map<PointType>::Set_Sensor_Params(std::vector<float> fov_theta_range, std::vector<float> fov_phi_range,
												  float fov_depth, std::vector<float> res_hor, std::vector<float> res_vert) {
		size_t i = 0;
		FOV_theta_range.clear();
		FOV_phi_range.clear();
		polar_res_hor.clear();
		polar_res_vert.clear();
		for (i = 0; i < fov_theta_range.size(); i++)
			FOV_theta_range.push_back(fov_theta_range[i]);
		for (i = 0; i < fov_phi_range.size(); i++)
			FOV_phi_range.push_back(fov_phi_range[i]);
		for (i = 0; i < res_hor.size(); i++)
			polar_res_hor.push_back(res_hor[i]);
		for (i = 0; i < res_vert.size(); i++)
			polar_res_vert.push_back(res_vert[i]);
		FOV_theta_min = FOV_theta_range[0];
		FOV_theta_max = FOV_theta_range[FOV_theta_range.size() - 1];
		FOV_phi_min = FOV_phi_range[0];
		FOV_phi_max = FOV_phi_range[FOV_phi_range.size() - 1];
		if (FOV_theta_max - FOV_theta_min > 2 * M_PI - EPSS)
			THETA_360 = true;
		if (FOV_phi_max - FOV_phi_min > 2 * M_PI - EPSS)
			PHI_360 = true;

		polar_width = 0;
		polar_height = 0;
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

		polar_res_threshold = 0.0;
		for (size_t i = 0; i < polar_res_hor.size(); i++)
			polar_res_threshold = max(polar_res_hor[i], polar_res_threshold);
		for (size_t i = 0; i < polar_res_vert.size(); i++)
			polar_res_threshold = max(polar_res_vert[i], polar_res_threshold);

		Map_Param_SET = true;
	}

	template <typename PointType>
	float octree_map<PointType>::Idx2Rad(int idx, bool type) {
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

	template <typename PointType>
	int octree_map<PointType>::Rad2Idx(float rad, bool type) {
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

	template <typename PointType>
	void octree_map<PointType>::euc2polar(Vector3f &euc_pt, polar3D *polar_pt) {
		polar_pt->theta_rad = atan2(euc_pt[1], euc_pt[0]);
		polar_pt->phi_rad = atan2(euc_pt[2], euc_pt.head<2>().norm());
		polar_pt->r = euc_pt.norm();
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

	template <typename PointType>
	void octree_map<PointType>::polar2euc(polar3D *polar_pt, Vector3f &euc_pt) {
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

	template <typename PointType>
	BoxPointType octree_map<PointType>::SplitBBX(BoxPointType father_bbx, uint8_t ChildIdx) {
		BoxPointType Child_bbx;
		int idx[3];
		float sizeL = father_bbx.vertex_max[0] - father_bbx.vertex_min[0];
		idx[0] = ChildIdx & 1;
		idx[1] = (ChildIdx & 2) >> 1;
		idx[2] = (ChildIdx & 4) >> 2;
		for (int i = 0; i < 3; i++) {
			Child_bbx.vertex_min[i] = father_bbx.vertex_min[i] + sizeL / 2.0 * idx[i];
			Child_bbx.vertex_max[i] = Child_bbx.vertex_min[i] + sizeL / 2.0;
		}
		return Child_bbx;
	}

	template <typename PointType>
	void octree_map<PointType>::InitNode(OctreeMapNode *root, OctreeMapNode *father_node_) {
		if (father_node_ != nullptr) {
			root->depth = father_node_->depth + 1;
		} else {
			float max_L = BoundingBox.vertex_max[0] - BoundingBox.vertex_min[0];
			for (int i = 1; i < 3; i++) {
				float tmp_L = BoundingBox.vertex_max[i] - BoundingBox.vertex_min[i];
				if (tmp_L > max_L + EPSS)
					max_L = tmp_L;
			}
			int k = ceil(log2(max_L / resolution));
			max_L = resolution * pow(2, k);
			for (int i = 0; i < 3; i++) {
				Root_BBX_.vertex_min[i] = BoundingBox.vertex_min[i];
				Root_BBX_.vertex_max[i] = BoundingBox.vertex_min[i] + max_L;
			}
			root->depth = 0;
		}
		max_depth = max(max_depth, int(root->depth));
	}

	template <typename PointType>
	uint8_t octree_map<PointType>::IntersectedWithBox(BoxPointType cur_bbx, BoxPointType box) {
		/*
			Return
				0: Not Intersected;
				1: Contained;
				2: Intersected
		*/
		if (cur_bbx.vertex_min[0] + EPSS > box.vertex_max[0] || cur_bbx.vertex_max[0] - EPSS < box.vertex_min[0])
			return 0;
		if (cur_bbx.vertex_min[1] + EPSS > box.vertex_max[1] || cur_bbx.vertex_max[1] - EPSS < box.vertex_min[1])
			return 0;
		if (cur_bbx.vertex_min[2] + EPSS > box.vertex_max[2] || cur_bbx.vertex_max[2] - EPSS < box.vertex_min[2])
			return 0;

		if (cur_bbx.vertex_min[0] + EPSS > box.vertex_min[0] && cur_bbx.vertex_max[0] - EPSS < box.vertex_max[0] && cur_bbx.vertex_min[1] + EPSS > box.vertex_min[1] && cur_bbx.vertex_max[1] - EPSS < box.vertex_max[1] && cur_bbx.vertex_min[2] + EPSS > box.vertex_min[2] && cur_bbx.vertex_max[2] - EPSS < box.vertex_max[2]) {
			return 1;
		}

		return 2;
	}

	template <typename PointType>
	PointType octree_map<PointType>::AcquireCenter(BoxPointType box) {
		PointType center;
		center.x = (box.vertex_min[0] + box.vertex_max[0]) / 2.0;
		center.y = (box.vertex_min[1] + box.vertex_max[1]) / 2.0;
		center.z = (box.vertex_min[2] + box.vertex_max[2]) / 2.0;
		center.intensity = (box.vertex_max[0] - box.vertex_min[0]);
		return center;
	}

	template <typename PointType>
	void octree_map<PointType>::DeleteNodes(OctreeMapNode **root) {
		if (*root == nullptr)
			return;
		for (int i = 0; i < 8; i++)
			DeleteNodes(&(*root)->ChildNodes[i]);
		delete *root;
		*root = nullptr;
	}

	template <typename PointType>
	void octree_map<PointType>::RetrieveUnknownPoints(OctreeMapNode *root, PointVector &points, BoxPointType cur_bbx) {
		if (root == nullptr || IntersectedWithBox(cur_bbx, BoundingBox) == 0)
			return;
		PointType p;
		p = AcquireCenter(cur_bbx);
		if (root->isLeaf && p.intensity < 2 * init_resolution) {
			points.push_back(p);
			return;
		}
		for (int i = 0; i < 8; i++)
			RetrieveUnknownPoints(root->ChildNodes[i], points, SplitBBX(cur_bbx, i));
	}

	template <typename PointType>
	void octree_map<PointType>::PurgeNode(OctreeMapNode **root) {
		if (*root == nullptr || (*root)->isLeaf)
			return;
		int size_sum = 0, is_leaf_sum = 0;
		for (int i = 0; i < 8; i++) {
			if ((*root)->ChildNodes[i] != nullptr) {
				size_sum += (*root)->ChildNodes[i]->sizeN;
				if ((*root)->ChildNodes[i]->isLeaf)
					is_leaf_sum++;
			}
		}
		(*root)->sizeN = size_sum;
		if ((*root)->sizeN == 0) {
			delete *root;
			*root = nullptr;
		} else {
			if (is_leaf_sum == 8) {
				for (int i = 0; i < 8; i++) {
					delete (*root)->ChildNodes[i];
					(*root)->ChildNodes[i] = nullptr;
				}
				(*root)->isLeaf = true;
				(*root)->sizeN = 1;
			}
		}
		return;
	}

	// Complete Space Representation without given nodes.
	template <typename PointType>
	void octree_map<PointType>::InitByResolution(float res_init) {
		if (Root_Node_ != nullptr)
			clear();
		Root_Node_ = new OctreeMapNode;
		InitNode(Root_Node_, nullptr);
		init_resolution = res_init;
	}

	template <typename PointType>
	void octree_map<PointType>::RangeOnDepthMap(polar3D &polar_pt, float cover_rad, int min_i[], int min_j[], int max_i[], int max_j[]) {
		// Vertical Range
		min_j[1] = -1;
		max_j[1] = -2;
		min_i[1] = -1;
		max_i[1] = -2;
		float phi_rad = polar_pt.phi_rad;
		if (phi_rad + cover_rad > FOV_phi_max - EPSS) {
			max_i[0] = polar_height - 1;
			if (PHI_360) {
				min_i[1] = 0;
				max_i[1] = Rad2Idx(phi_rad + cover_rad - 2 * M_PI, false);
			}
		} else {
			max_i[0] = Rad2Idx(phi_rad + cover_rad, false);
		}
		if (phi_rad - cover_rad < FOV_phi_min + EPSS) {
			min_i[0] = 0;
			if (PHI_360) {
				max_i[1] = polar_height - 1;
				min_i[1] = Rad2Idx(phi_rad - cover_rad + 2 * M_PI, false);
			}
		} else {
			min_i[0] = Rad2Idx(phi_rad - cover_rad, false);
		}
		// Horizontal Range
		float theta_rad = polar_pt.theta_rad;
		if (theta_rad + cover_rad > FOV_theta_max - EPSS) {
			max_j[0] = polar_width - 1;
			if (THETA_360) {
				min_j[1] = 0;
				max_j[1] = Rad2Idx(theta_rad + cover_rad - 2 * M_PI, true);
			}
		} else {
			max_j[0] = Rad2Idx(theta_rad + cover_rad, true);
		}
		if (theta_rad - cover_rad < FOV_theta_min + EPSS) {
			min_j[0] = 0;
			if (THETA_360) {
				max_j[1] = polar_width - 1;
				min_j[1] = Rad2Idx(theta_rad - cover_rad + 2 * M_PI, true);
			}
		} else {
			min_j[0] = Rad2Idx(theta_rad - cover_rad, true);
		}
		return;
	}

	template <typename PointType>
	void octree_map<PointType>::AcquireRangeOnDepthMap(PointType box_point, int min_i[], int min_j[], int max_i[], int max_j[]) {
		Vector3f temp_pt, dir_vec, dir;
		polar3D polar_pt;
		float cover_dis, cover_rad;
		temp_pt(0) = box_point.x;
		temp_pt(1) = box_point.y;
		temp_pt(2) = box_point.z;
		dir = temp_pt - cur_fov.pos;
		dir_vec = cur_fov.R.transpose() * dir;
		euc2polar(dir_vec, &polar_pt);
		cover_dis = 0.5 * box_point.intensity;
		cover_rad = asin(cover_dis / dir_vec.norm());
		RangeOnDepthMap(polar_pt, cover_rad, min_i, min_j, max_i, max_j);
		return;
	}

	template <typename PointType>
	void octree_map<PointType>::SinglePixelQuery(polar3D *polar_pt, float cover_rad, int min_i[], int min_j[], int max_i[], int max_j[], SegNode &ans) {
		float phi_rad, theta_rad;
		float box_theta = polar_pt->theta_rad;
		float box_phi = polar_pt->phi_rad;
		int i, j;
		for (i = min_i[0]; i <= max_i[0]; i++) {
			for (j = min_j[0]; j <= max_j[0]; j++) {
				if (cur_map->DepthMap(i, j) > 0) {
					phi_rad = cur_map->Polars[0](i, j);
					theta_rad = cur_map->Polars[1](i, j);
					if (theta_rad > box_theta - cover_rad && theta_rad < box_theta + cover_rad && phi_rad > box_phi - cover_rad && phi_rad < box_phi + cover_rad) {
						ans.cnt++;
						ans.Max = max(ans.Max, cur_map->DepthMap(i, j));
						ans.Min = min(ans.Min, cur_map->DepthMap(i, j));
					}
				}
			}
		}
		if (THETA_360 && min_j[1] >= 0) {
			for (i = min_i[0]; i <= max_i[0]; i++) {
				for (j = min_j[1]; j <= max_j[1]; j++) {
					if (cur_map->DepthMap(i, j) > 0) {
						phi_rad = cur_map->Polars[0](i, j);
						theta_rad = cur_map->Polars[1](i, j);
						if (theta_rad > box_theta - cover_rad && theta_rad < box_theta + cover_rad && phi_rad > box_phi - cover_rad && phi_rad < box_phi + cover_rad) {
							ans.cnt++;
							ans.Max = max(ans.Max, cur_map->DepthMap(i, j));
							ans.Min = min(ans.Min, cur_map->DepthMap(i, j));
						}
					}
				}
			}
		}
		if (PHI_360 && min_i[1] >= 0) {
			for (i = min_i[1]; i <= max_i[1]; i++) {
				for (j = min_j[0]; j <= max_j[0]; j++) {
					if (cur_map->DepthMap(i, j) > 0) {
						phi_rad = cur_map->Polars[0](i, j);
						theta_rad = cur_map->Polars[1](i, j);
						if (theta_rad > box_theta - cover_rad && theta_rad < box_theta + cover_rad && phi_rad > box_phi - cover_rad && phi_rad < box_phi + cover_rad) {
							ans.cnt++;
							ans.Max = max(ans.Max, cur_map->DepthMap(i, j));
							ans.Min = min(ans.Min, cur_map->DepthMap(i, j));
						}
					}
				}
			}
		}
	}

	template <typename PointType>
	uint8_t octree_map<PointType>::DetermineSplitType(PointType box_point, uint8_t insideFoV, int &cover_cnt, bool ContainedSensor) {
		// Special Process for those boxes containing the sensor.
		// When the sensor is contained in the box, the box should be split for further determination.
		// If the box has already achieved the minimal resolution, it is obviously free. Just delete it.
		if (ContainedSensor) {
			if (box_point.intensity >= 2 * resolution - EPSS)
				return uint8_t(1);
			else
				return uint8_t(2);
		}
		// if (fabs(box_point.x + 42.950) < 0.05 - EPSS && fabs(box_point.y + 28.950) < 0.05 - EPSS && fabs(box_point.z + 1.75) < 0.05 - EPSS) {
		// 	printf("Box Point: %0.3f,%0.3f,%0.3f, L: %0.3f\n", box_point.x, box_point.y, box_point.z, box_point.intensity);
		// 	print_flag = true;
		// }
		// if (fabs(box_point.x + 43.025) < 0.5 * box_point.intensity - EPSS && fabs(box_point.y + 28.825) < 0.5 * box_point.intensity - EPSS && fabs(box_point.z + 1.825) < 0.5 * box_point.intensity - EPSS) {
		// 	printf("Box Point: %0.3f,%0.3f,%0.3f, L: %0.3f\n", box_point.x, box_point.y, box_point.z, box_point.intensity);
		// 	print_flag = true;
		// }
		bool print_flag = false;
		uint8_t box_flag = 0;

		float box_theta = 0.0, box_phi = 0.0;
		float cur_l = box_point.intensity;

		/* Project Box Point onto Depth Map */
		Vector3f temp_pt, dir_vec, dir;
		polar3D polar_pt;
		temp_pt(0) = box_point.x;
		temp_pt(1) = box_point.y;
		temp_pt(2) = box_point.z;
		dir = temp_pt - cur_fov.pos;
		dir_vec = cur_fov.R.transpose() * dir;
		euc2polar(dir_vec, &polar_pt);

		/* Obtain Range on Depth Map */
		int min_i[2], min_j[2], max_i[2], max_j[2];
		float cover_dis = 0.5 * cur_l;
		float norm = dir_vec.norm();
		float cover_rad = asin(cover_dis / norm);
		RangeOnDepthMap(polar_pt, cover_rad, min_i, min_j, max_i, max_j);
		if (min_i[0] < 0 || min_j[0] < 0 || max_i[0] > polar_height || max_j[0] > polar_width) {
			printf("inside FOV: %d\n", insideFoV);
			printf("Contained Sensor: %d,%d\n", ContainedSensor, cur_fov.check_in_box(box_point));
			printf("Box Point: %0.3f,%0.3f,%0.3f, L: %0.3f\n", box_point.x, box_point.y, box_point.z, box_point.intensity);
			printf("Cover dis: %0.3f, Norm: %0.3f\n", cover_dis, norm);
			printf("polar: (%0.3f,%0.3f), cover_rad: %0.3f  ", box_phi / M_PI * 180, box_theta / M_PI * 180, cover_rad / M_PI * 180);
			printf("i:(%d,%d), j:(%d,%d)\n", min_i[0], max_i[0], min_j[0], max_j[0]);
		}
		// Obtain box coverage area
		float box_min = max(polar_pt.r - cover_dis, 0.0f);
		float box_max = min(polar_pt.r, cur_fov.FoV_depth) + cover_dis;
		int all_cnt = 0;
		if (min_i[0] <= max_i[0] && min_j[0] <= max_j[0]) {
			all_cnt = (max_i[0] - min_i[0] + 1) * (max_j[0] - min_j[0] + 1);
			if (THETA_360 && min_j[1] >= 0)
				all_cnt += (max_i[0] - min_i[0] + 1) * (max_j[1] - min_j[1] + 1);
			if (PHI_360 && min_i[1] >= 0)
				all_cnt += (max_i[1] - min_i[1] + 1) * (max_j[0] - min_j[0] + 1);
		}

		/* Query on Depth Map */
		float d_min = INFINITY, d_max = -INFINITY, ratio = 0.0f;
		int d_cnt = 0;
		SegNode ans;
		box_theta = polar_pt.theta_rad;
		box_phi = polar_pt.phi_rad;
		// segtree_st = high_resolution_clock::now();
		if (cover_rad > 0 && cover_rad < 0.5 * polar_res_threshold) {
			SinglePixelQuery(&polar_pt, cover_rad, min_i, min_j, max_i, max_j, ans);
		} else {
			if (min_i[0] <= max_i[0] && min_j[0] <= max_j[0]) {
				ans = segtree->query(min_i[0], max_i[0], min_j[0], max_j[0]);
				if (THETA_360 && min_j[1] >= 0 && min_j[1] <= max_j[1])
					ans += segtree->query(min_i[0], max_i[0], min_j[1], max_j[1]);
				if (PHI_360 && min_i[1] >= 0 && min_i[1] <= max_i[1])
					ans += segtree->query(min_i[1], max_i[1], min_j[0], max_j[0]);
			}
		}
		// segtree_ed = high_resolution_clock::now();
		// segtree_query_time += duration_cast<nanoseconds>(segtree_ed - segtree_st).count() * 1e-6;
		d_min = ans.Min;
		d_max = ans.Max;
		d_cnt = ans.cnt;
		cover_cnt = d_cnt;
		if (all_cnt == 0) {
			ratio = 0.0f;
		} else {
			ratio = d_cnt / float(all_cnt);
		}

		if (print_flag) {
			printf("	Phi: %0.3f, Theta: %0.3f, R: %0.3f, Cover: %0.3f\n", polar_pt.phi_rad / M_PI * 180, polar_pt.theta_rad / M_PI * 180, polar_pt.r, cover_dis);
			printf("	Range: (%d,%d), (%d,%d)\n", min_i[0], max_i[0], min_j[0], max_j[0]);
			printf("    min: %0.3f, max: %0.3f, cnt: %d\n", d_min, d_max, d_cnt);
			printf("    Box min: %0.3f, max: %0.3f, all: %d\n", box_min, box_max, all_cnt);
			printf("    cover rad: %0.6f, theshold: %0.3f\n\n", cover_rad, polar_res_threshold);
			for (int i = min_i[0]; i <= max_i[0]; i++) {
				printf("    ");
				for (int j = min_j[0]; j <= max_j[0]; j++) {
					printf("%0.2f ", cur_map->DepthMap(i, j));
				}
				printf("\n");
			}
		}
		/* Core Logic for Occupancy State Determination*/
		if (ratio > 0) {
			if (ratio < full_ratio) {
				/* When we find very few points scanned in the box. */
				// If the box is inside the detection range of current depth map, it should be split or removed.
				// depending on its resolution. Otherwise, it should be kept unknown.
				if (ratio > EPSS && d_max > box_min) {
					if (cur_l >= 2 * resolution - EPSS) {
						box_flag = 1;
					} else {
						box_flag = 2;
					}
				} else {
					box_flag = 0;
				}
			} else {
				if (d_max < box_min || d_min > box_max) {
					if (d_max < box_max) {
						/* All points are in front of the box -> unknown */
						box_flag = 0;
					} else {
						/* All points are behind the box -> free */
						if (insideFoV == 2 && cur_l >= resolution * 2.0 - EPSS)
							box_flag = 1;
						else
							box_flag = 2;
					}
				} else {
					if (cur_l >= resolution * 2.0 - EPSS) {
						/* The box should be further splitted if its length is larger than the minimal resolution */
						box_flag = 1;
					} else {
						/*
							The box cannot be splitted:
							1) If the box is occupied, it should be set as occupied.
							2) If there is no points inside the box, set it as free.
							In summary, the box is known and should be deleted from the tree.
						*/
						box_flag = 2;
					}
				}
			}
		}
		if (print_flag)
			printf("    Box_flag: %d\n", box_flag);
		return box_flag;
	}

	template <typename PointType>
	void octree_map<PointType>::UpdateFromDepthMap(OctreeMapNode **root, BoxPointType fov_box, BoxPointType cur_bbx, bool isParentInsideBBX, bool isParentInsideFoV) {
		if (*root == nullptr)
			return;
		if (!isParentInsideBBX) {
			uint8_t tp = IntersectedWithBox(cur_bbx, BoundingBox);
			if (tp == 0)
				return;
			isParentInsideBBX = tp == 1;
		}
		int i;
		PointType box_point = AcquireCenter(cur_bbx);
		int cover_cnt = 0;
		bool print_flag = false;
		// The size of node is larger than the init resolution. It should be split for further process.
		if (box_point.intensity > init_resolution && (*root)->isLeaf) {
			(*root)->isLeaf = false;
			for (i = 0; i < 8; i++) {
				(*root)->ChildNodes[i] = new OctreeMapNode;
				InitNode((*root)->ChildNodes[i], *root);
				BoxPointType child_bbx = SplitBBX(cur_bbx, i);
				if (IntersectedWithBox(child_bbx, fov_box) > 0)
					UpdateFromDepthMap(&(*root)->ChildNodes[i], fov_box, child_bbx, isParentInsideBBX);
			}
			PurgeNode(root);
			return;
		}
		// The size of node is suitable for processing.
		if ((*root)->isLeaf) {
			// If the node is a leaf node and inside the sensor FoV, we apply an operation according to the box operation logic.
			// The implementation is in DetermineSplitType.
			uint8_t insideFoV;
			bool ContainedSensor = cur_fov.check_in_box(box_point);
			if (!isParentInsideFoV) {
				insideFoV = cur_fov.check_with_FoV(box_point);
				if (insideFoV == 0 && ContainedSensor)
					insideFoV = 2;
			} else {
				insideFoV = 1;
			}
			isParentInsideFoV |= insideFoV == 1;
			if (insideFoV > 0) {
				uint8_t flag = DetermineSplitType(box_point, insideFoV, cover_cnt, ContainedSensor);
				// uint8_t flag = DetermineSplitType(box_point, ContainedSensor);
				switch (flag) {
				case 0:
					break;
				case 1:
					(*root)->isLeaf = false;
					// The node is split into eight and processed further.
					for (i = 0; i < 8; i++) {
						(*root)->ChildNodes[i] = new OctreeMapNode;
						InitNode((*root)->ChildNodes[i], *root);
						UpdateFromDepthMap(&(*root)->ChildNodes[i], fov_box, SplitBBX(cur_bbx, i), isParentInsideBBX, insideFoV == 1);
					}
					PurgeNode(root);
					break;
				case 2:
					DeleteNodes(root);
					break;
				default:
					break;
				}
			}
		} else {
			// If the node is not a leaf node, we firstly determine its intersection with the FoV bounding box.
			uint8_t IntersectionType;
			if (isParentInsideFoV)
				IntersectionType = 1;
			else
				IntersectionType = IntersectedWithBox(cur_bbx, fov_box);
			if (IntersectionType > 0) {
				if (box_point.intensity > init_resolution) {
					// If the size of node is larger than the init resolution to process, we directly look
					// into its child nodes.
					for (i = 0; i < 8; i++)
						UpdateFromDepthMap(&(*root)->ChildNodes[i], fov_box, SplitBBX(cur_bbx, i), isParentInsideBBX);
				} else {
					// If the node is suitable for processing, we perform a precise check about whether the
					// node is intersected with the sensor range.
					uint8_t insideFoV;
					bool ContainedSensor = cur_fov.check_in_box(box_point);
					if (!isParentInsideFoV) {
						insideFoV = cur_fov.check_with_FoV(box_point);
						if (insideFoV == 0 && ContainedSensor)
							insideFoV = 2;
					} else {
						insideFoV = 1;
					}
					isParentInsideFoV |= insideFoV == 1;
					if (insideFoV > 0) {
						uint8_t flag = DetermineSplitType(box_point, insideFoV, cover_cnt, ContainedSensor);
						switch (flag) {
						case 0:
							break;
						case 1:
							for (i = 0; i < 8; i++)
								UpdateFromDepthMap(&(*root)->ChildNodes[i], fov_box, SplitBBX(cur_bbx, i), isParentInsideBBX, insideFoV == 1);
							break;
						case 2:
							DeleteNodes(root);
							break;
						default:
							break;
						}
					}
					PurgeNode(root);
				}
			}
		}
		return;
	}

	template <typename PointType>
	void octree_map<PointType>::UpdateTree(SegmentTree *tree, FoVType fov, DepthmapType *depthmap) {
		assert(FullRatio_SET);
		assert(Map_Param_SET);
		segtree = tree;
		cur_fov = fov;
		cur_map = depthmap;
		BoxPointType fov_box = Calc_FoV_Bounding_Box(cur_fov);
		UpdateFromDepthMap(&Root_Node_, fov_box, Root_BBX_);
	}

	// Tree compression
	template <typename PointType>
	void octree_map<PointType>::InOrderTraverse(OctreeMapNode *root, FILE *fp) {
		if (root == nullptr)
			return;
		uint8_t mask = 0;
		for (int i = 0; i < 8; i++) {
			if (root->ChildNodes[i] != nullptr)
				mask += 1 << i;
		}

		fprintf(fp, "%d\n", mask);

		for (int i = 0; i < 8; i++) {
			if (root->ChildNodes[i] != nullptr)
				InOrderTraverse(root->ChildNodes[i], fp);
		}
		return;
	}

	template <typename PointType>
	void octree_map<PointType>::OutputMap(FILE *fp) {
		InOrderTraverse(Root_Node_, fp);
	}

	template <typename PointType>
	void octree_map<PointType>::BuildFromMasks(vector<int> &mask_vec) {
		Root_Node_ = new OctreeMapNode;
		InitNode(Root_Node_, nullptr);
		int start_idx = 0;
		SplitFromMasks(&Root_Node_, mask_vec, start_idx);
	}

	template <typename PointType>
	void octree_map<PointType>::SplitFromMasks(OctreeMapNode **root, vector<int> &mask_vec, int &idx) {
		if (mask_vec[idx] != 0) {
			uint8_t mask = mask_vec[idx];
			(*root)->isLeaf = false;
			for (int i = 0; i < 8; i++) {
				if ((mask & (1 << i))) {
					(*root)->ChildNodes[i] = new OctreeMapNode;
					InitNode((*root)->ChildNodes[i], *root);
					SplitFromMasks(&(*root)->ChildNodes[i], mask_vec, ++idx);
				}
			}
			PurgeNode(root);
		}
		return;
	}

	template <typename PointType>
	bool octree_map<PointType>::CheckUnknown(OctreeMapNode *root, PointType p, BoxPointType cur_bbx) {
		if (root->isLeaf) {
			return true;
		}
		PointType cur_point = AcquireCenter(cur_bbx);
		uint8_t idx = GetChildIdx(cur_point, p);
		if (root->ChildNodes[idx] != nullptr)
			return CheckUnknown(root->ChildNodes[idx], p, SplitBBX(cur_bbx, idx));
		else
			return false;
	}

	template <typename PointType>
	BoxPointType octree_map<PointType>::Calc_FoV_Bounding_Box(FoVType &cur_fov) {
		BoxPointType fov_box;

		fov_box.vertex_max[0] = cur_fov.pos(0);
		fov_box.vertex_min[0] = cur_fov.pos(0);
		fov_box.vertex_max[1] = cur_fov.pos(1);
		fov_box.vertex_min[1] = cur_fov.pos(1);
		fov_box.vertex_max[2] = cur_fov.pos(2);
		fov_box.vertex_min[2] = cur_fov.pos(2);

		Vector3f tmp;
		for (int i = 0; i < cur_fov.edge_vec.size(); i++) {
			tmp = cur_fov.pos + cur_fov.R * cur_fov.edge_vec[i];
			fov_box.vertex_max[0] = max(fov_box.vertex_max[0], tmp(0));
			fov_box.vertex_min[0] = min(fov_box.vertex_min[0], tmp(0));
			fov_box.vertex_max[1] = max(fov_box.vertex_max[1], tmp(1));
			fov_box.vertex_min[1] = min(fov_box.vertex_min[1], tmp(1));
			fov_box.vertex_max[2] = max(fov_box.vertex_max[2], tmp(2));
			fov_box.vertex_min[2] = min(fov_box.vertex_min[2], tmp(2));
		}

		for (int i = 0; i < 3; i++) {
			fov_box.vertex_max[i] += EPSS;
			fov_box.vertex_min[i] -= EPSS;
		}

		// printf("FoV Box: (%0.3f,%0.3f), (%0.3f,%0.3f), (%0.3f,%0.3f)\n", fov_box.vertex_min[0], fov_box.vertex_max[0],
		//     fov_box.vertex_min[1], fov_box.vertex_max[1], fov_box.vertex_min[2], fov_box.vertex_max[2]);

		return fov_box;
	}

	template <typename PointType>
	void octree_map<PointType>::CutFromRoot(OctreeMapNode **root, BoxPointType cur_bbx) {
		int idx = -1;
		int cnt = 0;
		int tp;
		BoxPointType child_bbx;
		for (int i = 0; i < 8; i++) {
			child_bbx = SplitBBX(cur_bbx, i);
			tp = IntersectedWithBox(BoundingBox, child_bbx);
			if (tp > 0) {
				idx = i;
				cnt++;
			}
		}
		if (cnt == 1) {
			for (int i = 0; i < 8; i++) {
				if (i != idx)
					DeleteNodes(&(*root)->ChildNodes[i]);
			}
			OctreeMapNode *tmp = *root;
			*root = (*root)->ChildNodes[idx];
			delete (tmp);
			CutFromRoot(root, SplitBBX(cur_bbx, idx));
		} else {
			Root_BBX_ = cur_bbx;
		}
	}

	template <typename PointType>
	void octree_map<PointType>::SlideMap(BoxPointType new_bbx) {
		BoundingBox = new_bbx;
		BoxPointType new_root_bbx(Root_BBX_);
		int idx = 0, dir;
		if (IntersectedWithBox(new_bbx, Root_BBX_) != 1) {
			for (int i = 0; i < 3; i++) {
				if (new_bbx.vertex_max[i] > Root_BBX_.vertex_max[i] + EPSS) {
					new_root_bbx.vertex_max[i] = Root_BBX_.vertex_max[i] + Root_BBX_.vertex_max[i] - Root_BBX_.vertex_min[i];
					dir = 0;
				} else if (new_bbx.vertex_min[i] < Root_BBX_.vertex_min[i] - EPSS) {
					new_root_bbx.vertex_min[i] = Root_BBX_.vertex_min[i] - (Root_BBX_.vertex_max[i] - Root_BBX_.vertex_min[i]);
					dir = 1;
				} else {
					new_root_bbx.vertex_max[i] = Root_BBX_.vertex_max[i] + Root_BBX_.vertex_max[i] - Root_BBX_.vertex_min[i];
				}
				idx += dir << i;
			}
			OctreeMapNode *tmp = Root_Node_;
			Root_BBX_ = new_root_bbx;
			Root_Node_ = new OctreeMapNode;
			Root_Node_->isLeaf = false;
			for (int i = 0; i < 8; i++) {
				if (i == idx)
					Root_Node_->ChildNodes[i] = tmp;
				else {
					Root_Node_->ChildNodes[i] = new OctreeMapNode;
					InitNode(Root_Node_->ChildNodes[i], Root_Node_);
				}
			}
		} else {
			CutFromRoot(&Root_Node_, Root_BBX_);
		}
		return;
	}

	SegmentTree::SegmentTree() {
		Root_node = nullptr;
		SetSize(0, 0);
	}

	SegmentTree::SegmentTree(int n, int m) {
		Root_node = nullptr;
		SetSize(n, m);
	}

	SegmentTree::~SegmentTree() {
		printf("[SegmentTree]: Tree deleted\n");
		delete_tree(&Root_node);
	}

	void SegmentTree::clear() {
		delete_tree(&Root_node);
	}

	void SegmentTree::SetSize(int n, int m) {
		Seg_N = n;
		Seg_M = m;
		Seg_Threshold = min(int(0.12 * min(Seg_N, Seg_M)), 25);
	}

	void SegmentTree::delete_tree(SegNode **root) {
		if (*root == nullptr)
			return;
		delete_tree(&(*root)->left_child);
		delete_tree(&(*root)->right_child);
		delete[] (*root)->root_y;
		delete *root;
		*root = nullptr;
		return;
	}

	void SegmentTree::init(MatrixXf DepthMap) {
		depthmap = DepthMap;
		build_x(&Root_node, 0, Seg_N - 1, DepthMap);
		return;
	}

	void SegmentTree::update_y(SegNode *root, int l, int r, int pos, int y, float value) {
		if (l == r) {
			if (is_leaf) {
				root->root_y[pos].Max = root->root_y[pos].Min = value;
				root->root_y[pos].cnt = 1;
			} else {
				SegNode tmp_node;
				if (root->left_child != nullptr)
					tmp_node += root->left_child->root_y[pos];
				if (root->right_child != nullptr)
					tmp_node += root->right_child->root_y[pos];
				root->root_y[pos] = tmp_node;
			}
			return;
		}
		int mid = (l + r) >> 1;
		int left_pos = (pos << 1) + 1;
		if (y <= mid)
			update_y(root, l, mid, left_pos, y, value);
		else
			update_y(root, mid + 1, r, left_pos + 1, y, value);
		root->root_y[pos] = root->root_y[left_pos] + root->root_y[left_pos + 1];
		return;
	}

	void SegmentTree::update_x(SegNode **root, int l, int r, int x, int y, float value) {
		if (*root == nullptr) {
			*root = new SegNode(Seg_M);
		}
		if (l == r) {
			is_leaf = true;
			update_y((*root), 0, Seg_M - 1, 0, y, value);
			return;
		}
		int mid = (l + r) >> 1;
		if (x <= mid)
			update_x(&(*root)->left_child, l, mid, x, y, value);
		else
			update_x(&(*root)->right_child, mid + 1, r, x, y, value);
		is_leaf = false;
		update_y((*root), 0, Seg_M - 1, 0, y, value);
		return;
	}

	void SegmentTree::build_y(SegNode *root, int l, int r, int pos, int x, MatrixXf &DepthMap) {
		if (l == r) {
			int y = l;
			if (is_leaf) {
				if (DepthMap(x, y) > 0) {
					root->root_y[pos].Max = root->root_y[pos].Min = DepthMap(x, y);
					root->root_y[pos].cnt = 1;
				} else {
					root->root_y[pos].Max = -INFINITY;
					root->root_y[pos].Min = INFINITY;
					root->root_y[pos].cnt = 0;
				}
			} else {
				SegNode tmp_node;
				if (root->left_child != nullptr)
					tmp_node += root->left_child->root_y[pos];
				if (root->right_child != nullptr)
					tmp_node += root->right_child->root_y[pos];
				root->root_y[pos] = tmp_node;
			}
			return;
		}
		int mid = (l + r) >> 1;
		int left_pos = (pos << 1) + 1;
		build_y(root, l, mid, left_pos, x, DepthMap);
		build_y(root, mid + 1, r, left_pos + 1, x, DepthMap);
		root->root_y[pos] = root->root_y[left_pos] + root->root_y[left_pos + 1];
	}

	void SegmentTree::build_x(SegNode **root, int l, int r, MatrixXf &DepthMap) {
		if (*root == nullptr) {
			*root = new SegNode(Seg_M);
		}
		if (l == r) {
			is_leaf = true;
			build_y(*root, 0, Seg_M - 1, 0, l, DepthMap);
			return;
		}
		int mid = (l + r) >> 1;
		build_x(&(*root)->left_child, l, mid, DepthMap);
		build_x(&(*root)->right_child, mid + 1, r, DepthMap);
		is_leaf = false;
		build_y(*root, 0, Seg_M - 1, 0, l, DepthMap);
	}

	SegNode SegmentTree::query_y(SegNode *root, int yl, int yr, int l, int r, int pos) {
		if (root == nullptr)
			return SegNode();
		if (yl <= l && r <= yr) {
			return root->root_y[pos];
		}
		SegNode tmp_node;
		int mid = (l + r) >> 1;
		int left_pos = (pos << 1) + 1;
		if (yr <= mid) {
			tmp_node = query_y(root, yl, yr, l, mid, left_pos);
		} else if (yl > mid) {
			tmp_node = query_y(root, yl, yr, mid + 1, r, left_pos + 1);
		} else {
			tmp_node = query_y(root, yl, mid, l, mid, left_pos);
			tmp_node += query_y(root, mid + 1, yr, mid + 1, r, left_pos + 1);
		}
		return tmp_node;
	}

	SegNode SegmentTree::query_x(SegNode *root, int xl, int xr, int yl, int yr, int l, int r) {
		if (root == nullptr)
			return SegNode();
		if (xl <= l && r <= xr) {
			return query_y(root, yl, yr, 0, Seg_M - 1, 0);
		}
		SegNode tmp_node;
		int mid = (l + r) >> 1;
		if (xr <= mid) {
			tmp_node = query_x(root->left_child, xl, xr, yl, yr, l, mid);
		} else if (xl > mid) {
			tmp_node = query_x(root->right_child, xl, xr, yl, yr, mid + 1, r);
		} else {
			tmp_node = query_x(root->left_child, xl, mid, yl, yr, l, mid);
			tmp_node += query_x(root->right_child, mid + 1, xr, yl, yr, mid + 1, r);
		}
		return tmp_node;
	}

	SegNode SegmentTree::query(int xl, int xr, int yl, int yr) {
		int i, j;
		if (xr - xl + 1 <= Seg_Threshold && yr - yl + 1 <= Seg_Threshold) {
			ans_node.Max = -INFINITY;
			ans_node.Min = INFINITY;
			ans_node.cnt = 0;
			for (i = xl; i <= xr; i++) {
				for (j = yl; j <= yr; j++) {
					if (depthmap(i, j) > 0) {
						ans_node.cnt++;
						ans_node.Max = max(depthmap(i, j), ans_node.Max);
						ans_node.Min = min(depthmap(i, j), ans_node.Min);
					}
				}
			}
		} else {
			ans_node = query_x(Root_node, xl, xr, yl, yr, 0, Seg_N - 1);
		}
		return ans_node;
	}

	void SegmentTree::update(int x, int y, float value) {
		update_x(&Root_node, 0, Seg_N - 1, x, y, value);
	}
} // namespace DMap

template class DMap::octree_map<PointType>;