#include <iostream>
#include <array>
#include <vector>
#include <string>

using namespace std;

class point : public std::array<float, 9>
{
public:
	// dimension of space (or "k" of k-d tree)
	// KDTree class accesses this member
	static const int dimension = 3;

	// the constructors
	point() : array() {}
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

unique_ptr<vector<point>> points;

void import_point_cloud(string &file_name)
{
	int size = 123456;
	points = make_unique<vector<point>>(size);


}

string file_name_extention(".ply");

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
			file_name = "PointCloud" + file_name_extention;
		}
		else if(file_name.length() < 5 || file_name.find_last_of(file_name_extention) != file_name.length() - file_name_extention.length())
		{
			file_name += file_name_extention;
		}
	}

	import_point_cloud(file_name);


	return 0;
}