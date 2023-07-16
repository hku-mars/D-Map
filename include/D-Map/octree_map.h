#include <D-Map/lib_common.h>
#include <queue>

namespace DMap {
	struct SegNode {
		float Max, Min;
		SegNode *left_child = nullptr;
		SegNode *right_child = nullptr;
		int cnt;
		SegNode *root_y = nullptr;
		SegNode(int Segnodesize = 0, float max_v = -INFINITY, float min_v = INFINITY, int cnt_v = 0) {
			if (Segnodesize > 0)
				root_y = new SegNode[Segnodesize << 2];
			Max = max_v;
			Min = min_v;
			cnt = cnt_v;
		}
		SegNode operator+(const struct SegNode &t) {
			SegNode node;
			node.Max = max(this->Max, t.Max);
			node.Min = min(this->Min, t.Min);
			node.cnt = this->cnt + t.cnt;
			return node;
		};
		SegNode operator+=(const struct SegNode &t) {
			this->cnt += t.cnt;
			this->Max = max(this->Max, t.Max);
			this->Min = min(this->Min, t.Min);
			return *this;
		}
	};

	class SegmentTree {
	private:
		SegNode *Root_node = nullptr;
		MatrixXf depthmap;
		bool is_leaf = false;
		int Seg_N, Seg_M, Seg_Threshold;
		SegNode ans_node;
		void update_y(SegNode *root, int l, int r, int pos, int y, float value);
		void update_x(SegNode **root, int l, int r, int x, int y, float value);
		SegNode query_y(SegNode *root, int yl, int yr, int l, int r, int pos);
		SegNode query_x(SegNode *root, int xl, int xr, int yl, int yr, int l, int r);
		void build_y(SegNode *root, int l, int r, int pos, int x, MatrixXf &DepthMap);
		void build_x(SegNode **root, int l, int r, MatrixXf &DepthMap);
		void delete_tree(SegNode **root);

	public:
		SegmentTree();
		SegmentTree(int n, int m);
		~SegmentTree();
		void init(MatrixXf DepthMap);
		void clear();
		void SetSize(int n, int m);
		void update(int x, int y, float value);
		SegNode query(int xl, int xr, int yl, int yr);
	};

	template <typename PointType>
	class octree_map {
	public:
		using PointVector = vector<PointType, Eigen::aligned_allocator<PointType>>;
		using Ptr = shared_ptr<octree_map<PointType>>;

		struct OctreeMapNode {
			OctreeMapNode *ChildNodes[8];
			uint8_t depth;
			int sizeN;
			bool isLeaf, inBBX;
			OctreeMapNode() {
				for (int i = 0; i < 8; i++)
					this->ChildNodes[i] = nullptr;
				this->sizeN = 1;
				this->isLeaf = true;
			}
		};

	private:
		// Octree Base:
		BoxPointType BoundingBox;
		float resolution, init_resolution;
		bool Resolution_SET = false, BoundingBox_SET = false;
		// Depth Map Usage:
		std::vector<float> FOV_theta_range, FOV_phi_range;
		std::vector<float> polar_res_hor, polar_res_vert;
		std::vector<int> polar_N, polar_M;
		float FOV_depth;
		float FOV_phi_min, FOV_phi_max;
		float FOV_theta_min, FOV_theta_max;
		float polar_res_threshold;
		int polar_width, polar_height;
		bool THETA_360 = false, PHI_360 = false;

		bool FullRatio_SET, Map_Param_SET;
		float full_ratio;
		int init_depth = 0;
		FoVType cur_fov;
		DepthmapType *cur_map;
		SegmentTree *segtree = nullptr;

		// Depth Map Processing
		int Rad2Idx(float rad, bool type);

		float Idx2Rad(int idx, bool type);
		void euc2polar(Vector3f &euc_pt, polar3D *polar_pt);
		void polar2euc(polar3D *polar_pt, Vector3f &euc_pt);
		void AcquireRangeOnDepthMap(PointType box_point, int min_i[], int min_j[], int max_i[], int max_j[]);
		void RangeOnDepthMap(polar3D &polar_pt, float cover_rad, int min_i[], int min_j[], int max_i[], int max_j[]);
		void SinglePixelQuery(polar3D *polar_pt, float cover_rad, int min_i[], int min_j[], int max_i[], int max_j[], SegNode &ans);

		uint8_t IntersectedWithBox(BoxPointType cur_bbx, BoxPointType box);
		PointType AcquireCenter(BoxPointType box);
		BoxPointType SplitBBX(BoxPointType father_bbx, uint8_t ChildIdx);

		// Octree Map
		void InitNode(OctreeMapNode *root, OctreeMapNode *father_node_);
		void DeleteNodes(OctreeMapNode **root);
		void PurgeNode(OctreeMapNode **root);
		void UpdateFromDepthMap(OctreeMapNode **root, BoxPointType fov_box, BoxPointType cur_bbx, bool isParentInsideBBX = false, bool isParentInsideFoV = false);
		void CutFromRoot(OctreeMapNode **root, BoxPointType cur_bbx);

		// State Determination Module
		uint8_t DetermineSplitType(PointType box_point, uint8_t insideFoV, int &cover_cnt, bool ContainedSensor = false);
		BoxPointType Calc_FoV_Bounding_Box(FoVType &cur_fov);

		// Compression
		void InOrderTraverse(OctreeMapNode *root, FILE *fp);
		void SplitFromMasks(OctreeMapNode **root, vector<int> &mask_vec, int &idx);

		// inline functions
		inline uint8_t GetChildIdx(PointType root_p, PointType p) {
			uint8_t idx, i, j, k;
			i = p.x < root_p.x ? 0 : 1;
			j = p.y < root_p.y ? 0 : 1;
			k = p.z < root_p.z ? 0 : 1;
			idx = i + (j << 1) + (k << 2);
			return idx;
		}

		inline bool InsideBBX(BoxPointType box, PointType p) {
			if (p.x > box.vertex_max[0] + EPSS || p.x < box.vertex_min[0] - EPSS)
				return false;
			if (p.y > box.vertex_max[1] + EPSS || p.y < box.vertex_min[1] - EPSS)
				return false;
			if (p.z > box.vertex_max[2] + EPSS || p.z < box.vertex_min[2] - EPSS)
				return false;
			return true;
		}

	public:
		OctreeMapNode *Root_Node_ = nullptr;
		BoxPointType Root_BBX_;
		octree_map();
		octree_map(float res);
		octree_map(BoxPointType box);
		octree_map(float res, BoxPointType box);
		~octree_map();
		int size();
		void clear();
		void RetrieveUnknownPoints(OctreeMapNode *root, PointVector &points, BoxPointType cur_bbx);
		void SetResolution(float res);
		void SetBoundingBox(BoxPointType box);
		void SetFullRatio(float rate);
		void Set_Sensor_Params(std::vector<float> fov_theta_range, std::vector<float> fov_phi_range,
							   float fov_depth, std::vector<float> res_hor, std::vector<float> res_vert);
		void UpdateTree(SegmentTree *segtree, FoVType fov, DepthmapType *depthmap);
		void InitByResolution(float res_init);
		void OutputMap(FILE *fp);
		// For benchmark
		void BuildFromMasks(vector<int> &mask_vec);
		int debug_cnt = 0;
		bool CheckUnknown(OctreeMapNode *root, PointType p, BoxPointType cur_bbx);

		// Map Sliding
		void SlideMap(BoxPointType new_bbx);

		int max_depth = 0;

		// For memory analysis
		inline size_t GetMemory() {
			size_t mem = size() * (sizeof(DMap::octree_map<PointType>::OctreeMapNode) - sizeof(BoxPointType)) + sizeof(DMap::octree_map<PointType>);
			return mem;
		}

		inline double GetMemKB() { return double(GetMemory()) / 1024.0; }

		inline double GetMemMB() { return double(GetMemory()) / 1024.0 / 1024.0; }

		inline double GetMemGB() { return double(GetMemory()) / 1024.0 / 1024.0 / 1024.0; }
	};
} // namespace DMap