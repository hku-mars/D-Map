#pragma once
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>

#define EPSS 1e-5
#define MAX_INT 1e5
#define DegToRad(x) (x * 0.01745329251)

using namespace std;
using namespace Eigen;
using namespace chrono;

typedef pcl::PointXYZI PointType;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef pcl::PointCloud<PointType> PointClouds;
typedef time_point<high_resolution_clock, nanoseconds> TimePoint;

struct OdomType {
	Vector3f pos;
	Matrix3f R;
	Quaternionf q;
	OdomType() :
		pos(0, 0, 0), R(Matrix3f::Identity()), q() {}
	OdomType(Quaternionf qua, float px = 0.0f, float py = 0.0f, float pz = 0.0f) {
		pos(0) = px;
		pos(1) = py;
		pos(2) = pz;
		q = qua;
		R = qua.matrix();
	}
	OdomType(Matrix3f Rot, float px = 0.0f, float py = 0.0f, float pz = 0.0f) {
		pos(0) = px;
		pos(1) = py;
		pos(2) = pz;
		R = Rot;
		q = R;
	}
};

struct DepthmapType {
	OdomType odom;
	MatrixXf DepthMap, Polars[2];
	int min_phi, max_phi, min_theta, max_theta;
	DepthmapType(int Min_phi = MAX_INT, int Max_phi = -MAX_INT, int Min_theta = MAX_INT, int Max_theta = -MAX_INT, int cnt = 0) {
		min_phi = Min_phi;
		max_phi = Max_phi;
		min_theta = Min_theta;
		max_theta = Max_theta;
	}
};

struct polar3D {
	int theta;
	int phi;
	float r, theta_rad, phi_rad;
	polar3D() :
		theta(), phi(), r() {}
	polar3D(int theta_in, int phi_in, float r_in) :
		theta(theta_in), phi(phi_in), r(r_in) {}
	polar3D(int theta_in, int phi_in, float r_in, float theta_rad_in, float phi_rad_in) :
		theta(theta_in), phi(phi_in), r(r_in), theta_rad(theta_rad_in), phi_rad(phi_rad_in) {}
};

struct BoxPointType {
	float vertex_min[3];
	float vertex_max[3];
	BoxPointType() {
		for (int i = 0; i < 3; i++) {
			this->vertex_min[i] = INFINITY;
			this->vertex_max[i] = -INFINITY;
		}
	}
	BoxPointType(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
		this->vertex_min[0] = min_x;
		this->vertex_min[1] = min_y;
		this->vertex_min[2] = min_z;
		this->vertex_max[0] = max_x;
		this->vertex_max[1] = max_y;
		this->vertex_max[2] = max_z;
	}
	BoxPointType(const BoxPointType &bbx) {
		this->vertex_min[0] = bbx.vertex_min[0];
		this->vertex_min[1] = bbx.vertex_min[1];
		this->vertex_min[2] = bbx.vertex_min[2];
		this->vertex_max[0] = bbx.vertex_max[0];
		this->vertex_max[1] = bbx.vertex_max[1];
		this->vertex_max[2] = bbx.vertex_max[2];
	}
};

struct PlaneType {
	Eigen::Vector3f p[4];
};

struct FoVType {
	uint8_t FoV_shape = 0;
	Vector3f pos;
	Matrix3f R;
	float FoV_depth;
	float FoV_theta_min, FoV_theta;
	float FoV_phi_min, FoV_phi;
	vector<Vector3f> edge_vec;
	FoVType() :
		FoV_shape(0), pos(0.0f, 0.0f, 0.0f), FoV_theta(0.0f), FoV_theta_min(0.0f), FoV_phi(0.0f), FoV_phi_min(0.0f), FoV_depth(0.0f) {}
	FoVType(uint8_t shape, Vector3f cur_pos, Matrix3f rot, float theta, float theta_min, float phi, float phi_min, float depth) {
		FoV_shape = shape;
		pos = cur_pos;
		R = rot;
		FoV_theta_min = theta_min;
		FoV_theta = theta;
		FoV_phi_min = phi_min;
		FoV_phi = phi;
		FoV_depth = depth;
		// Initialize edge vector for calculation of FoV bounding box .
		Matrix3f Rz_neg, Rz_pos, Ry_neg, Ry_pos;
		Vector3f vec_x(1, 0, 0), vec_y(0, 1, 0), vec_z(0, 0, 1);
		vec_x = FoV_depth * vec_x;

		if (FoV_theta >= FoV_phi) {
			Vector3f vz_pos, vz_neg;
			if (FoV_phi > M_PI) {
				vz_pos = vec_z * FoV_depth;
				vz_neg = -vec_z * FoV_depth;
			} else {
				vz_neg = vec_z * FoV_depth * tan(FoV_phi_min);
				vz_pos = vec_z * FoV_depth * tan(FoV_phi_min + FoV_phi);
			}
			edge_vec.push_back(vec_x + vz_pos);
			edge_vec.push_back(vec_x + vz_neg);

			Rz_neg << cos(FoV_theta_min), -sin(FoV_theta_min), 0,
				sin(FoV_theta_min), cos(FoV_theta_min), 0,
				0, 0, 1;

			Rz_pos << cos(FoV_theta_min + FoV_theta), -sin(FoV_theta_min + FoV_theta), 0,
				sin(FoV_theta_min + FoV_theta), cos(FoV_theta_min + FoV_theta), 0,
				0, 0, 1;

			edge_vec.push_back(Rz_neg * vec_x + vz_pos);
			edge_vec.push_back(Rz_pos * vec_x + vz_pos);
			edge_vec.push_back(Rz_neg * vec_x + vz_neg);
			edge_vec.push_back(Rz_pos * vec_x + vz_neg);
			if (FoV_theta > M_PI_2) {
				vec_y = vec_y * FoV_depth;
				edge_vec.push_back(vec_y + vz_pos);
				edge_vec.push_back(-vec_y + vz_pos);
				edge_vec.push_back(vec_y + vz_neg);
				edge_vec.push_back(-vec_y + vz_neg);
			}
		} else {
			Vector3f vy_pos, vy_neg;
			if (FoV_theta > M_PI_2) {
				vy_pos = vec_y * FoV_depth;
				vy_neg = -vec_y * FoV_depth;
			} else {
				vy_neg = vec_y * FoV_depth * tan(FoV_theta_min);
				vy_pos = vec_y * FoV_depth * tan(FoV_theta_min + FoV_theta);
			}

			edge_vec.push_back(vec_x + vy_pos);
			edge_vec.push_back(vec_x + vy_neg);

			Ry_neg << cos(FoV_phi_min), 0, sin(FoV_phi_min),
				0, 1, 0,
				-sin(FoV_phi_min), 0, cos(FoV_phi_min);

			Ry_pos << cos(FoV_phi_min + FoV_phi), 0, sin(FoV_phi_min + FoV_phi),
				0, 1, 0,
				-sin(FoV_phi_min + FoV_phi), 0, cos(FoV_phi_min + FoV_phi);

			edge_vec.push_back(Ry_neg * vec_x + vy_pos);
			edge_vec.push_back(Ry_pos * vec_x + vy_pos);
			edge_vec.push_back(Ry_neg * vec_x + vy_neg);
			edge_vec.push_back(Ry_pos * vec_x + vy_neg);
			if (FoV_theta > M_PI_2) {
				vec_z = vec_z * FoV_depth;
				edge_vec.push_back(vec_z + vy_pos);
				edge_vec.push_back(-vec_z + vy_pos);
				edge_vec.push_back(vec_z + vy_neg);
				edge_vec.push_back(-vec_z + vy_neg);
			}
		}
	}

	bool check_point(Vector3f point) {
		Vector3f vec;
		vec = this->R.transpose() * (point - this->pos);
		if (vec.norm() > FoV_depth)
			return false;
		if (FoV_shape == 1) {
			float a = vec.x() * tan(FoV_theta / 2.0);
			float b = vec.x() * tan(FoV_phi / 2.0);
			if (vec.y() * vec.y() / (a * a) + vec.z() * vec.z() / (b * b) < 1)
				return true;
			else
				return false;
		} else {
			float theta, phi;
			theta = atan2(vec(1), vec(0));
			if (theta < this->FoV_theta_min - EPSS || theta > this->FoV_theta_min + this->FoV_theta + EPSS) {
				return false;
			}
			phi = atan2(vec(2), vec.head<2>().norm());
			if (phi < this->FoV_phi_min - EPSS || phi > this->FoV_phi_min + this->FoV_phi + EPSS) {
				return false;
			}
		}
		return true;
	}

	bool check_point(PointType p) {
		Vector3f vec, point;
		point << p.x, p.y, p.z;
		vec = this->R.transpose() * (point - this->pos);
		if (vec.norm() > FoV_depth)
			return false;
		if (FoV_shape == 1) {
			float a = vec.x() * tan(FoV_theta / 2.0);
			float b = vec.x() * tan(FoV_phi / 2.0);
			if (vec.y() * vec.y() / (a * a) + vec.z() * vec.z() / (b * b) < 1)
				return true;
			else
				return false;
		} else {
			float theta, phi;
			theta = atan2(vec(1), vec(0));
			if (theta < this->FoV_theta_min - EPSS || theta > this->FoV_theta_min + this->FoV_theta + EPSS) {
				return false;
			}
			phi = atan2(vec(2), vec.head<2>().norm());
			if (phi < this->FoV_phi_min - EPSS || phi > this->FoV_phi_min + this->FoV_phi + EPSS) {
				return false;
			}
		}
		return true;
	}

	bool check_box_at_edge(PointType point) {
		int i, j, k, t;
		bool in_flag = false, out_flag = false;
		Vector3f p;
		for (t = 0; t < 8; t++) {
			i = (t & 4) >> 2;
			j = (t & 2) >> 1;
			k = t & 1;
			p.x() = point.x + (2 * i - 1) * point.intensity / 2.0;
			p.y() = point.y + (2 * j - 1) * point.intensity / 2.0;
			p.z() = point.z + (2 * k - 1) * point.intensity / 2.0;
			// printf("    Check Point: (%0.3f,%0.3f,%0.3f)",p.x(),p.y(),p.z());
			if (this->check_point(p)) {
				in_flag = true;
			} else {
				out_flag = true;
			}
			if (in_flag && out_flag)
				return true;
		}
		return false;
	}

	bool check_in_box(PointType center) {
		if (this->pos(0) > center.x + center.intensity / 2.0 + EPSS || this->pos(0) < center.x - center.intensity / 2.0 - EPSS)
			return false;
		if (this->pos(1) > center.y + center.intensity / 2.0 + EPSS || this->pos(1) < center.y - center.intensity / 2.0 - EPSS)
			return false;
		if (this->pos(2) > center.z + center.intensity / 2.0 + EPSS || this->pos(2) < center.z - center.intensity / 2.0 - EPSS)
			return false;
		return true;
	}

	uint8_t check_with_FoV(PointType point) {
		int i, j, k, t;
		bool in_flag = false, out_flag = false;
		Vector3f p;
		for (t = 0; t < 8; t++) {
			i = (t & 4) >> 2;
			j = (t & 2) >> 1;
			k = t & 1;
			p.x() = point.x + (2 * i - 1) * point.intensity / 2.0;
			p.y() = point.y + (2 * j - 1) * point.intensity / 2.0;
			p.z() = point.z + (2 * k - 1) * point.intensity / 2.0;
			// printf("    Check Point: (%0.3f,%0.3f,%0.3f)",p.x(),p.y(),p.z());
			if (this->check_point(p)) {
				in_flag = true;
			} else {
				out_flag = true;
			}
		}
		/*
			Return
				0: Not Intersected;
				1: Contained;
				2: Intersected
		*/
		if (in_flag && !out_flag)
			return 1;
		else if (!in_flag && out_flag)
			return 0;
		else if (in_flag && out_flag)
			return 2;
		else
			return 0;
	}
};

template <typename T>
class MANUAL_Q {
private:
	int counter = 0;
	int Q_LEN;
	bool is_empty, initialized = false;

public:
	T *q;
	int head = 0, tail = 0;
	MANUAL_Q();
	MANUAL_Q(int len);
	~MANUAL_Q();

	void init(int len);
	void pop();
	T front();
	T back();
	void clear();
	void push(T op);
	bool empty();
	int size();
};

template <typename T>
MANUAL_Q<T>::MANUAL_Q() {
	initialized = false;
}

template <typename T>
MANUAL_Q<T>::MANUAL_Q(int len) {
	Q_LEN = len;
	q = new T[Q_LEN];
	initialized = true;
}

template <typename T>
MANUAL_Q<T>::~MANUAL_Q() {
	if (initialized)
		delete[] q;
}

template <typename T>
void MANUAL_Q<T>::init(int len) {
	Q_LEN = len;
	q = new T[Q_LEN];
	initialized = true;
}

template <typename T>
void MANUAL_Q<T>::pop() {
	assert(initialized && "Queue is not initialized!");
	if (counter == 0)
		return;
	head++;
	head %= Q_LEN;
	counter--;
	if (counter == 0)
		is_empty = true;
	return;
}

template <typename T>
T MANUAL_Q<T>::front() {
	assert(initialized && "Queue is not initialized!");
	return q[head];
}

template <typename T>
T MANUAL_Q<T>::back() {
	assert(initialized && "Queue is not initialized!");
	return q[(tail + Q_LEN - 1) % Q_LEN];
}

template <typename T>
void MANUAL_Q<T>::clear() {
	assert(initialized && "Queue is not initialized!");
	head = 0;
	tail = 0;
	counter = 0;
	is_empty = true;
	return;
}

template <typename T>
void MANUAL_Q<T>::push(T op) {
	assert(initialized && "Queue is not initialized!");
	if (counter == Q_LEN) {
		printf("Queue FULL. Head Element Popped!\n");
		pop();
	}
	q[tail] = op;
	counter++;
	if (is_empty)
		is_empty = false;
	tail++;
	tail %= Q_LEN;
}

template <typename T>
bool MANUAL_Q<T>::empty() {
	assert(initialized && "Queue is not initialized!");
	return is_empty;
}

template <typename T>
int MANUAL_Q<T>::size() {
	assert(initialized && "Queue is not initialized!");
	return counter;
}
