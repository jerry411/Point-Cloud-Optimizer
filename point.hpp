#pragma once

#include <array>

class point : public std::array<float, 9>
{
public:
	// dimension of space / "k" of k-d tree (KDTree class accesses this member)
	static const int dimension = 3;

	bool is_centroid = false;
	bool is_marked = false;;

	// default constructor
	point() : array() {}

	// X/Y/Z world coordinates, R/G/B colors, NX/NY/NZ coordinates of normal vectors
	point(const float x, const float y, const float z, const float r, const float g, const float b, const float nx, const float ny, const float nz) : array()
	{
		(*this)[0] = x;
		(*this)[1] = y;
		(*this)[2] = z;

		(*this)[3] = r;
		(*this)[4] = g;
		(*this)[5] = b;

		(*this)[6] = nx;
		(*this)[7] = ny;
		(*this)[8] = nz;
	}
};