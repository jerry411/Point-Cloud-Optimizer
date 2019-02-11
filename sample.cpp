
#include <array>
#include <vector>
#include <string>

#include "kdtree.hpp"
#include <iostream>

// user-defined point type
// inherits std::array in order to use operator[]
class my_point : public std::array<float, 9>
{
public:

	// dimension of space (or "k" of k-d tree)
	// KDTree class accesses this member
	static const int dimension = 3;

	// the constructors
	my_point() : array() {}
	my_point(const float x, const float y, const float z, const float r, const float g, const float b, const float nx, const float ny, const float nz) : array()
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

int main(int argc, char **argv)
{
	const int seed = argc > 1 ? std::stoi(argv[1]) : 0;
	srand(seed);

	// generate space
	const int width = 500;
	const int height = 500;

	// generate points
	const int npoints = 1000;
	std::vector<my_point> points(npoints);

	for (int i = 0; i < npoints; i++)
	{
		const int x = rand() % width;
		const int y = rand() % height;
		const int z = rand() % height;
		points[i] = my_point(x, y, z, 123, 123, 123, 456, 456, 456);
	}

	// build k-d tree
	kdt::KDTree<my_point> kdtree(points);

	// generate query (center of the space)
	const my_point query(0.5 * width, 0.5 * height, 0.5 * height, 0, 0, 0, 0, 0, 0);
	//cv::circle(img, cv::Point2d(query), 1, cv::Scalar(0, 0, 255), -1);

	// nearest neigbor search
	//const cv::Mat I0 = img.clone();
	const int idx = kdtree.nnSearch(query);
	//cv::circle(I0, cv::Point2d(points[idx]), 1, cv::Scalar(255, 255, 0), -1);
	//cv::line(I0, cv::Point2d(query), cv::Point2d(points[idx]), cv::Scalar(0, 0, 255));

	// k-nearest neigbors search
	//const cv::Mat I1 = img.clone();
	const int k = 1000;
	const std::vector<int> knn_indices = kdtree.knnSearch(query, k);
	for (int i : knn_indices)
	{
		std::cout << points[i][0] << "  " << points[i][1] << "  " << points[i][2] << "  " << points[i][3] << "  " << points[i][4] << "  " << points[i][5]
			<< "  " << points[i][6] << "  " << points[i][7] << "  " << points[i][8] << std::endl;

		//cv::circle(I1, cv::Point2d(points[i]), 1, cv::Scalar(255, 255, 0), -1);
		//cv::line(I1, cv::Point2d(query), cv::Point2d(points[i]), cv::Scalar(0, 0, 255));
	}
	
	// radius search
	//const cv::Mat I2 = img.clone();
	const double radius = 50;
	const std::vector<int> radIndices = kdtree.radiusSearch(query, radius);
	//for (int i : radIndices)
		//cv::circle(I2, cv::Point2d(points[i]), 1, cv::Scalar(255, 255, 0), -1);
	//cv::circle(I2, cv::Point2d(query), cvRound(radius), cv::Scalar(0, 0, 255));

	// show results
	//cv::imshow("Nearest neigbor search", I0);
	//cv::imshow("K-nearest neigbors search (k = 10)", I1);
	//cv::imshow("Radius search (radius = 50)", I2);

	//cv::waitKey(0);

	return 0;
}