#include <D-Map/lib_common.h>
#include <tools/tools_kd_hash.hpp>

namespace DMap {
	template <typename PointType>

	class gridmap {
	public:
		using PointVector = vector<PointType, Eigen::aligned_allocator<PointType>>;

	private:
		BoxPointType BoundingBox;
		// BoxPointType ActualBox;
		float resolution;
		float max_range;
		bool MapParams_SET = false;
		Hash_map_3d<float, PointType> map;
		int len_low[3], len_high;
		int size_low, size_high;
		int active_cnt = 0;

		float CalcDist(PointType p1, PointType p2);
		bool InsideBox(PointType p);
		BoxPointType AcquireBox(PointType p);
		PointType AcquireCenter(PointType p);

		float l_hit;

		int side_L[3];

	public:
		gridmap();
		gridmap(float res, BoxPointType box);
		~gridmap();
		int size();
		void SetMapParams(float res, BoxPointType box, float range = INFINITY);
		void AddPoints(PointVector &points);
		void AddPoints(OdomType odom, PointVector &points);
		void RetrievePoints(PointVector &points);
		void RetrievePointCenters(PointVector &points);
		void SlideMap(BoxPointType new_bbx);

		bool CheckOccupied(PointType point);
		void OutputMap(FILE *fp);
		void clear();
		double GetMemGB();
		double GetMemMB();
		double GetMemKB();
		size_t GetMemory();
	};
} // namespace DMap