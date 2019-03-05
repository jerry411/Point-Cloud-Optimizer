#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "point.hpp"
#include "kdtree.hpp"
#include <sstream>

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

enum user_def_variables { space_interval_var, vector_deviation_var };

string file_name_extention(".ply");
string default_file_name("PointCloud" + file_name_extention);

void import_point_cloud(string& file_name)
{
	ifstream in_file(file_name);

	cout << endl << "Importing and parsing file: " + file_name << endl;

	// process first part of header
	string line, token;
	for (size_t i = 0; i < 3; i++)
		getline(in_file, line);

	// process number of vertices
	stringstream line_ss(line);
	for (size_t i = 0; i < 3; i++)
		getline(line_ss, token, ' ');

	points = vector<point>();

	// process rest of header
	while (in_file >> line)
		if (line == "end_header")
			break;

	// process actual vertices
	float x, y, z, r, g, b, nx, ny, nz;
	while (in_file >> x >> y >> z >> r >> g >> b >> nx >> ny >> nz)
	{
		points.emplace_back(x, y, z, r, g, b, nx, ny, nz);
	}

	cout << "File " + file_name + " successfully imported and parsed" << endl << endl;
}

void build_tree()
{
	cout << "Building K-D tree. This may take even few minutes depending on point cloud size." << endl;

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

			vector<int> neighbours_indices = tree.knn_search(selected, static_cast<int>(space_interval_dt));

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

float manual_float_input(const user_def_variables variable)
{
	string text;
	float default_value;

	if (variable == space_interval_var)
	{
		text = "Space Interval (DT)";
		default_value = space_interval_dt_default;
	}
	else
	{
		text = "Normal Vector Deviation (NT)";
		default_value = vector_deviation_nt_default;
	}

	string input;

	cout << endl << "Enter " + text + ": ";
	getline(cin, input);

	if (input.empty())
	{
		cout << "Invalid value. Using default value (" << default_value << ") for " << text << " instead."<< endl;

		return default_value;
	}

	float return_value;

	try
	{
		return_value = stof(input);
	}
	catch (const std::exception&)
	{
		cout << "Invalid value. Using default value (" << default_value << ") for " << text << " instead." << endl << endl;
		return default_value;
	}

	return  return_value;
}

float process_float_arg(const int argc, char** argv, const int index, const user_def_variables variable)
{
	if (argc > index)
	{
		const string arg(argv[index]);

		try
		{
			return stof(arg);
		}
		catch (const std::exception&)
		{
			return manual_float_input(variable);
		}	
	}
	else
	{
		return manual_float_input(variable);
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
			cout << "Using default file name: " << default_file_name << endl;
			file_name = default_file_name;
		}
		else if (file_name.length() < 5 || file_name.find_last_of(file_name_extention) != file_name.length() - file_name_extention.length())
		{
			file_name += file_name_extention;
		}
	}

	space_interval_dt = process_float_arg(argc, argv, 2, space_interval_var); // third argument is space interval (DT)

	vector_deviation_nt = process_float_arg(argc, argv, 3, vector_deviation_var); // fourth argument is vector deviation (NT)

	return file_name;
}

int main(const int argc, char* argv[])
{
	string file_name = process_args(argc, argv);

	try
	{
		import_point_cloud(file_name);
	}
	catch (const std::exception& e)
	{
		cout << endl << endl << "Error! File " + file_name + " was not successfully imported or parsed!";
		return -1;
	}

	build_tree();	

	cluster_initialization();

	return 0;
}