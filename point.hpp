#pragma once

/** @brief Data class holding info for 1 point from point cloud. Data is stored in array for easier iterative access.
*/
class point
{
public:
	static const int dimension = 3;

	bool is_centroid = false;
	bool is_marked = false;

	float data[9]{}; // actual data of point (coordinates, color, normal vector)

	// X/Y/Z world coordinates, R/G/B colors, NX/NY/NZ coordinates of normal vectors
	point(const float x, const float y, const float z, const float r, const float g, const float b, const float nx, const float ny, const float nz)
	{
		data[0] = x;
		data[1] = y;
		data[2] = z;

		data[3] = r;
		data[4] = g;
		data[5] = b;

		data[6] = nx;
		data[7] = ny;
		data[8] = nz;
	}

	explicit point(const float array[9])
	{
		for (size_t i = 0; i < 9; i++)
		{
			data[i] = array[i];
		}
	}

	/** @brief Euclidian distance of this point to another point.
	*/
	double distance(const point& other) const
	{
		double distance = 0;

		for (size_t i = 0; i < 3; i++)
			distance += (data[i] - other.data[i]) * (data[i] - other.data[i]);

		return sqrt(distance);
	}
};