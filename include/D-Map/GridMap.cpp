#include "GridMap.h"
#include <tools/tools_kd_hash.hpp>

namespace DMap {
	template <typename PointType>
	gridmap<PointType>::gridmap() {
		MapParams_SET = false;
	}

	template <typename PointType>
	gridmap<PointType>::gridmap(float res, BoxPointType box) {
		resolution = res;
		BoundingBox = box;
		MapParams_SET = true;
	}

	template <typename PointType>
	gridmap<PointType>::~gridmap() {
		clear();
		printf("[GridMap]: Map Deleted\n");
	}

	template <typename PointType>
	int gridmap<PointType>::size() {
		return map.total_size();
	}

	template <typename PointType>
	void gridmap<PointType>::SetMapParams(float res, BoxPointType box, float range) {
		resolution = res;
		BoundingBox = box;
		MapParams_SET = true;
		for (int i = 0; i < 3; i++) {
			side_L[i] = ceil((box.vertex_max[i] - box.vertex_min[i]) / resolution);
		}
		max_range = range;
		// map.reserve(5e7);
	}

	template <typename PointType>
	BoxPointType gridmap<PointType>::AcquireBox(PointType p) {
		BoxPointType box;
		box.vertex_min[0] = floor((p.x - BoundingBox.vertex_min[0]) / resolution) * resolution + BoundingBox.vertex_min[0];
		box.vertex_min[1] = floor((p.y - BoundingBox.vertex_min[1]) / resolution) * resolution + BoundingBox.vertex_min[1];
		box.vertex_min[2] = floor((p.z - BoundingBox.vertex_min[2]) / resolution) * resolution + BoundingBox.vertex_min[2];
		for (int i = 0; i < 3; i++)
			box.vertex_max[i] = box.vertex_min[i] + resolution;
		return box;
	}

	template <typename PointType>
	PointType gridmap<PointType>::AcquireCenter(PointType p) {
		PointType center;
		center.x = floor((p.x - BoundingBox.vertex_min[0]) / resolution) * resolution + BoundingBox.vertex_min[0] + resolution / 2.0;
		center.y = floor((p.y - BoundingBox.vertex_min[1]) / resolution) * resolution + BoundingBox.vertex_min[1] + resolution / 2.0;
		center.z = floor((p.z - BoundingBox.vertex_min[2]) / resolution) * resolution + BoundingBox.vertex_min[2] + resolution / 2.0;
		return center;
	}

	template <typename PointType>
	bool gridmap<PointType>::InsideBox(PointType p) {
		if (p.x < BoundingBox.vertex_min[0] + EPSS || p.x > BoundingBox.vertex_max[0] - EPSS)
			return false;
		if (p.y < BoundingBox.vertex_min[1] + EPSS || p.y > BoundingBox.vertex_max[1] - EPSS)
			return false;
		if (p.z < BoundingBox.vertex_min[2] + EPSS || p.z > BoundingBox.vertex_max[2] - EPSS)
			return false;
		return true;
	}

	template <typename PointType>
	void gridmap<PointType>::AddPoints(OdomType odom, PointVector &points) {
		assert(MapParams_SET && "GridMap Params Unset");
		size_t i;
		float cur_dist, new_dist;
		for (i = 0; i < points.size(); i++) {
			if (!InsideBox(points[i]))
				continue;
			Vector3f p(points[i].x, points[i].y, points[i].z);
			if ((p - odom.pos).norm() > max_range)
				continue;
			PointType center = AcquireCenter(points[i]);
			if (map.if_exist(center.x, center.y, center.z)) {
				PointType *cur_p = map.get_data(center.x, center.y, center.z);
				cur_dist = CalcDist(*cur_p, center);
				new_dist = CalcDist(points[i], center);
				if (new_dist < cur_dist) {
					*cur_p = points[i];
				}
			} else {
				map.insert(center.x, center.y, center.z, points[i]);
			}
		}
	}

	template <typename PointType>
	void gridmap<PointType>::AddPoints(PointVector &points) {
		assert(MapParams_SET && "GridMap Params Unset");
		size_t i;
		float cur_dist, new_dist;
		for (i = 0; i < points.size(); i++) {
			if (!InsideBox(points[i]))
				continue;
			PointType center = AcquireCenter(points[i]);
			if (map.if_exist(center.x, center.y, center.z)) {
				PointType *cur_p = map.get_data(center.x, center.y, center.z);
				cur_dist = CalcDist(*cur_p, center);
				new_dist = CalcDist(points[i], center);
				if (new_dist < cur_dist) {
					*cur_p = points[i];
				}
			} else {
				map.insert(center.x, center.y, center.z, points[i]);
			}
		}
	}

	template <typename PointType>
	float gridmap<PointType>::CalcDist(PointType p1, PointType p2) {
		return ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
	}

	template <typename PointType>
	void gridmap<PointType>::RetrievePoints(PointVector &points) {
		std::vector<PointType> tmp;
		map.all_data(tmp);
		points.clear();
		points.resize(tmp.size());
		for (size_t i = 0; i < points.size(); i++)
			points[i] = tmp[i];
	}

	template <typename PointType>
	void gridmap<PointType>::RetrievePointCenters(PointVector &points) {
		std::vector<PointType> tmp;
		map.all_data(tmp);
		points.clear();
		points.resize(tmp.size());
		for (size_t i = 0; i < points.size(); i++)
			points[i] = AcquireCenter(tmp[i]);
	}

	template <typename PointType>
	void gridmap<PointType>::OutputMap(FILE *fp) {
		std::vector<PointType> tmp;
		map.all_data(tmp);
		for (size_t i = 0; i < tmp.size(); i++) {
			fprintf(fp, "%0.6f,%0.6f,%0.6f\n", tmp[i].x, tmp[i].y, tmp[i].z);
		}
	}

	template <typename PointType>
	bool gridmap<PointType>::CheckOccupied(PointType point) {
		if (!InsideBox(point))
			return false;
		PointType center = AcquireCenter(point);
		return (map.if_exist(center.x, center.y, center.z));
	}

	template <typename PointType>
	void gridmap<PointType>::SlideMap(BoxPointType new_bbx) {
		map.erase_data_out_of_range(new_bbx.vertex_min[0], new_bbx.vertex_max[0], new_bbx.vertex_min[1], new_bbx.vertex_max[1], new_bbx.vertex_min[2], new_bbx.vertex_max[2]);
		BoundingBox = new_bbx;
		// }
	}

	template <typename PointType>
	void gridmap<PointType>::clear() {
		map.clear();
	}

	template <typename PointType>
	size_t gridmap<PointType>::GetMemory() {
		return (map.get_memory() + sizeof(*this));
	}

	template <typename PointType>
	double gridmap<PointType>::GetMemKB() {
		return double(GetMemory()) / 1024.0;
	}

	template <typename PointType>
	double gridmap<PointType>::GetMemMB() {
		return double(GetMemory()) / 1024.0 / 1024.0;
	}

	template <typename PointType>
	double gridmap<PointType>::GetMemGB() {
		return double(GetMemory()) / 1024.0 / 1024.0 / 1024.0;
	}

} // namespace DMap

template class DMap::gridmap<PointType>;