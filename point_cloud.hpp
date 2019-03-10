#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP
#include "point.hpp"
#include <vector>

/** @brief Data class containing vector of actual points. Other data structures holds indices to this vector.
*/
template <typename T>
struct point_cloud
{
	std::vector<point> points; // points of point cloud themselves (there are expected to be millions of points == tens of millions of bytes)

	// Must return the number of data points
	size_t kdtree_get_point_count() const
	{
		return points.size();
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	T kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		return points[idx].data[dim];
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const
	{
		return false;
	}

};
#endif // POINT_CLOUD_HPP
