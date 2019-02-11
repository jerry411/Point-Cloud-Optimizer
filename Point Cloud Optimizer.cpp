#include <iostream>
#include <array>
#include <vector>
#include <string>

#include "kdtree.hpp"

using namespace std;
using namespace kdt;

class point : public std::array<float, 9>
{
public:
	// dimension of space / "k" of k-d tree (KDTree class accesses this member)
	static const int dimension = 3;

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

vector<point> points; // points of point cloud themselves (there are expected to be millions of points == tens of millions of MB)
unique_ptr<KDTree<point>> tree; // K-D tree holding indices from "points" vector

void import_point_cloud(string &file_name)
{

	/**/

	cout << "Importing and parsing file: " + file_name << endl;

	size_t size = 123456;
	points = vector<point>(size);

	/**/

	cout << "File " + file_name + " successfully imported and parsed";
}

string file_name_extention(".ply");
string default_file_name("PointCloud" + file_name_extention);

void build_tree()
{
	cout << "Building K-D tree. This may take several seconds depending on point cloud size." << endl;

	tree = make_unique<KDTree<point>>(points);

	cout << "Finished building K-D tree." << endl;
}

int maina(const int argc, char *argv[])
{
	string file_name;

	if (argc > 1)
	{
		file_name = argv[1];

		if (file_name.length() < 5 || file_name.find_last_of(file_name_extention) != file_name.length() - file_name_extention.length())
		{
			file_name += file_name_extention;
		}
	}
	else
	{
		cout << "Enter file name: ";
		getline(cin, file_name);

		if (file_name.empty())
		{
			cout << endl << "Using default file name: " << default_file_name << endl << endl;
			file_name = default_file_name;
		}
		else if(file_name.length() < 5 || file_name.find_last_of(file_name_extention) != file_name.length() - file_name_extention.length())
		{
			file_name += file_name_extention;
		}
	}

	try
	{
		import_point_cloud(file_name);
	}
	catch (const std::exception&)
	{
		cout << "Error! File " + file_name + " was not successfully imported or parsed!";
		return -1;
	}

	build_tree();	

	/**/

	return 0;
}