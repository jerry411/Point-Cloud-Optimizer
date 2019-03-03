#include <iostream>
#include <vector>
#include <string>

#include "point.hpp"
#include "kdtree.hpp"

using namespace std;
using namespace kdt;

typedef vector<int> cluster; //TODO: int to size_t

vector<point> points; // points of point cloud themselves (there are expected to be millions of points == tens of millions of bytes)
vector<cluster> clusters;
kd_tree<point> tree; // K-D tree holding indices from "points" vector

float space_interval_dt;
float vector_deviation_nt;

float space_interval_dt_default = 1;
float vector_deviation_nt_default = 0.5;

string file_name_extention(".ply");
string default_file_name("PointCloud" + file_name_extention);

void import_point_cloud(string& file_name)
{

	/**/

	cout << "Importing and parsing file: " + file_name << endl;

	size_t size = 123456;
	points = vector<point>(size);

	/**/

	cout << "File " + file_name + " successfully imported and parsed";
}

void build_tree()
{
	cout << "Building K-D tree. This may take several seconds depending on point cloud size." << endl;

	tree = kd_tree<point>(points);

	cout << "Finished building K-D tree." << endl;
}

// if point is not marked, it becames centroid of new cluster which contains neighbours from space_interval_dt distance
void cluster_initialization()
{
	for (size_t i = 0; i < points.size(); i++)
	{
		point selected = points[i];

		if (!selected.is_marked)
		{
			selected.is_centroid = true;
			selected.is_marked = true;

			vector<int> neighbours_indices = tree.knn_search(selected, space_interval_dt);

			vector<int> new_cluster(i); // index of centroid of a cluster is first in vector
			new_cluster.reserve(neighbours_indices.size() + 1);

			for (size_t j = 0; j < neighbours_indices.size(); j++)
			{
				if (!points[j].is_marked)
				{
					new_cluster.push_back(neighbours_indices[j]);
					points[j].is_marked = true;
				}				
			}

			// move new cluster to global variable clusters
			clusters.resize(clusters.size() + 1);
			clusters[clusters.size() - 1] = move(new_cluster);
		}
	}
}

float manual_float_input(const bool is_space_interval)
{
	string text;
	float default_value;

	if (is_space_interval)
	{
		text = "Space Interval (DT)";
		default_value = space_interval_dt_default;
	}
	else
	{
		text = "Space Interval (DT)";
		default_value = vector_deviation_nt_default;
	}

	string input;

	cout << endl << "Enter " + text + ": ";
	getline(cin, input);

	if (input.empty())
	{
		cout << endl << "Using default" + text + ": " << default_value << endl << endl;
		return default_value;
	}

	float return_value;

	try
	{
		return_value = stoi(input);
	}
	catch (const std::exception&)
	{
		cout << endl << "Using default" + text + ": " << default_value << endl << endl;
		return default_value;
	}

	return  return_value;
}

void process_float_arg(const int argc, char** argv, const int index, const bool is_space_interval)
{
	if (argc > index)	// third argument is space interval (DT)
	{
		const string arg(argv[index]);

		try
		{
			space_interval_dt = stoi(arg);
		}
		catch (const std::exception&)
		{
			manual_float_input(true);
		}	
	}
	else
	{
		manual_float_input(true);
	}
}

string process_args(const int argc, char* argv[])
{
	string file_name;

	if (argc > 1) // first argument is automatically name of this program; second argument is filename
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
		else if (file_name.length() < 5 || file_name.find_last_of(file_name_extention) != file_name.length() - file_name_extention.length())
		{
			file_name += file_name_extention;
		}
	}

	process_float_arg(argc, argv, 2, true); // third argument is space interval (DT)

	process_float_arg(argc, argv, 3, false); // fourth argument is vector deviation (NT)
}

int main(const int argc, char* argv[])
{
	string file_name = process_args(argc, argv);

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

	cluster_initialization();

	return 0;
}