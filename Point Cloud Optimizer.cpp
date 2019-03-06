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
vector<cluster> clusters; // vector of clusters, where cluster holds indices to its members (first index refers to cluster centroid)
kd_tree<point> tree; // K-D tree holding indices from "points" vector

// Space Interval Threshold (DT) - largest distance from cluster centroid to any cluster member
float space_interval_dt;
float space_interval_dt_default = 1;

// Normal Vector Deviation Threshold (NT) - largest deviation of normal vectors of any pair of cluster members (otherwise cluster is divided)
float vector_deviation_nt;
float vector_deviation_nt_default = 0.5;

enum user_def_variables { space_interval_var, vector_deviation_var };

string file_name_extention(".ply");
string default_file_name("PointCloud" + file_name_extention);

/** @brief Parses point cloud from external ASCII .ply file. 
 *File is expected to comply .fly standards with this specific structure/header:
 
ply
format ascii 1.0
element vertex <number of vertices>
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property float nx
property float ny
property float nz
end_header

*/
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

/** @brief Builds k-d tree from points
*/
void build_tree()
{
	cout << "Building K-D tree. This may take even few minutes depending on point cloud size." << endl;

	tree = kd_tree<point>(points);

	cout << "Finished building K-D tree." << endl;
}

/** @brief Creates initial clusters. If point is not marked, it becames centroid of new cluster. 
 *	This new cluster contains non-marked neighbours of centroid whose distance is less than or equal to Space Interval Threshold (DT).
*/
void cluster_initialization()
{
	for (size_t i = 0; i < points.size(); i++)
	{
		point& selected = points[i];

		if (!points[i].is_marked)
		{
			selected.is_centroid = true;

			// index of centroid of a cluster is first in vector (always returned as first from knn_search)
			vector<int> neighbours_indices = tree.knn_search(points[i], static_cast<int>(space_interval_dt));

			// create new cluster
			clusters.resize(clusters.size() + 1);
			vector<int>& current_cluster = clusters[clusters.size() - 1];
			current_cluster.reserve(neighbours_indices.size());

			// fill the new cluster
			for (size_t j = 0; j < neighbours_indices.size(); j++)
			{
				int point_index = neighbours_indices[j];

				if (!points[point_index].is_marked) // do not copy indices to marked points to cluster
				{
					current_cluster.push_back(point_index);
					points[point_index].is_marked = true;
				}
			}
		}
	}
}

/** @brief Decides whether value of specific user variable is valid.
*/
bool user_var_value_is_valid(const float value, const user_def_variables user_var)
{
	switch (user_var)
	{
	case space_interval_var:
		return value > 0; // space interval must be positive (for 0 we would output same point cloud as in input)

	case vector_deviation_var:
		return !(value < 0 || value > 1); // Normal Vector Deviation Threshold (NT) must be between 0 and 1

	default:
		return false;
	}
}

/** @brief Returns name of specific user variable (Space Interval Threshold (DT) or Normal Vector Deviation Threshold (NT)).
*/
string text_for_user_variable(const user_def_variables user_var)
{
	switch (user_var)
	{
	case space_interval_var:
		return "Space Interval Threshold (DT)";

	case vector_deviation_var:
		return "Normal Vector Deviation Threshold(NT)";

	default:
		return "";
	}
}

/** @brief Returns default value of specific user variable (Space Interval Threshold (DT) or Normal Vector Deviation Threshold (NT)).
*/
float default_for_user_variable(const user_def_variables user_var)
{
	switch (user_var)
	{
	case space_interval_var:
		return space_interval_dt_default;

	case vector_deviation_var:
		return vector_deviation_nt_default;

	default:
		return -1;
	}
}

/** @brief Processes manual input of float value for user variables to console. If values are not valid, default values are used.
*/
float manual_float_input(const user_def_variables user_var)
{
	const string text = text_for_user_variable(user_var);
	const float default_value = default_for_user_variable(user_var);

	string input;

	cout << endl << "Enter " + text + ": ";
	getline(cin, input);

	if (input.empty())
	{
		cout << "Invalid value. Using default value (" << default_value << ") for " << text << " instead."<< endl << endl;
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

	switch (user_var)
	{
	case space_interval_var:
		if (!user_var_value_is_valid(return_value, space_interval_var))
		{
			cout << "Value must be greater than 0. Using default value (" << default_value << ") for " << text << " instead." << endl << endl;
			return default_value;
		}
		break;

	case vector_deviation_var:
		if (!user_var_value_is_valid(return_value, vector_deviation_var))
		{
			cout << "Value must be between 0 and 1. Using default value (" << default_value << ") for " << text << " instead." << endl << endl;
			return default_value;
		}
		break;

	default:
		break;
	}

	return  return_value;
}

/** @brief Processes input float arguments. If there are no arguments or values are invalid, used is asked to provide them to console.
*/
float process_float_arg(const int argc, char** argv, const int index, const user_def_variables user_var)
{
	if (argc > index)
	{
		const string arg(argv[index]);

		const string text = text_for_user_variable(user_var);
		const float default_value = default_for_user_variable(user_var);

		try
		{
			const float return_value = stof(arg);

			switch (user_var)
			{
			case space_interval_var:
				if (!user_var_value_is_valid(return_value, space_interval_var))
				{
					cout << "Invalid value in argument. Value must be between 0 and 1. Using default value (" << default_value << ") for " << text << " instead." << endl << endl;
					return manual_float_input(user_var);
				}
				break;

			case vector_deviation_var:
				if (!user_var_value_is_valid(return_value, vector_deviation_var))
				{
					cout << "Invalid value in argument. Value must be between 0 and 1. Using default value (" << default_value << ") for " << text << " instead." << endl << endl;
					return manual_float_input(user_var);
				}
				break;

			default:
				break;
			}

			return return_value;
		}
		catch (const std::exception&)
		{
			cout << "Invalid value in argument." << endl << endl;
			return manual_float_input(user_var);
		}	
	}
	else
	{
		return manual_float_input(user_var);
	}
}

/** @brief Processes input arguments containing filename and user defined variables (Space Interval Threshold (DT) and Normal Vector Deviation Threshold (NT)).
 *	If there are no arguments, used is asked to provide them to console.
*/
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

	space_interval_dt = process_float_arg(argc, argv, 2, space_interval_var); // third argument is Space Interval Threshold (DT)

	vector_deviation_nt = process_float_arg(argc, argv, 3, vector_deviation_var); // fourth argument is Normal Vector Deviation Threshold (NT)

	return file_name;
}

/*vector<int> boundary_cluster_detection()
{
	
}

void boundary_cluster_subdivision(const vector<int>& boundary_clusters)
{
	
}*/

/** @brief Standard deviation of normal vectors of 2 points. Normal vectors are expected to be normalized, therefore return value is between 0 and 1.
*/
float standard_deviation(point& p1, point& p2)
{
	float sum = 0;

	for (size_t i = 0; i < 3; i++)
		sum += pow(p1[i] - p2[i], 2);

	return sqrt((sum) / 2);
}

/** @brief Returns new means (indices to cluster of pair of points with largest deviation of normal vectors).
 *	If this deviation is larger than Normal Vector Deviation Threshold (NT), cluster should be divided.
 *	Returns pair (-1, -1) as indicator if cluster should not be divided.
*/
pair<int, int> new_means(cluster& cluster)
{
	if (cluster.size() <= 1) // cluster with 1 member should not be divided
	{
		return {-1, -1};
	}

	float max_deviation = 0;
	int max_index1, max_index2;

	for (size_t i = 0; i < cluster.size() - 1; i++)
	{
		for (size_t j = i; j < cluster.size(); j++)
		{
			const float local_deviation = standard_deviation(points[cluster[i]], points[cluster[j]]);

			if (local_deviation > max_deviation)
			{
				max_deviation = local_deviation;
				max_index1 = i;
				max_index2 = j;
			}
		}
	}

	if (max_deviation >= vector_deviation_nt)
	{
		return { max_index1, max_index2 }; // indices to cluster of pair of points with largest deviation of normal vectors 
	}
	else
	{
		return { -1, -1 }; // indicator that cluster should not be divided
	}
}

pair<cluster, cluster> k_means_clustering(const cluster& init_cluster)
{
	//************
	cluster temp1, temp2;
	return { temp1, temp2 };
}

vector<cluster> new_clusters;

/** @brief Decides whether cluster should be divided. If yes, it is recursively divided using k-means. If no, it is added to new_clusters.
*/
void recursive_cluster_subdivision(cluster& init_cluster)
{
	const pair<int, int> means = new_means(init_cluster);

	if (means.first == -1 || means.second == -1) // cluster should not be divided anymore
	{
		new_clusters.push_back(init_cluster);
	}
	else // recursively divide cluster
	{
		pair<cluster, cluster> divided_clusters = k_means_clustering(init_cluster);

		// means became new centroids for new clusters
		points[init_cluster[0]].is_centroid = false;
		points[init_cluster[means.first]].is_centroid = true;
		points[init_cluster[means.second]].is_centroid = true;

		// recursion
		recursive_cluster_subdivision(divided_clusters.first);
		recursive_cluster_subdivision(divided_clusters.second);
	}
}

/** @brief Calls subdivision on all clusters
*/
void main_cluster_subdivision()
{
	for (size_t i = 0; i < clusters.size(); i++)
	{
		recursive_cluster_subdivision(clusters[i]);
	}
}

/** @brief Exports centroid from new_clusters. Exported file has same header and format as input file.
*/
void export_point_cloud(const string& output_file_name)
{
	ofstream output_file(output_file_name);

	cout << endl << "Exporting reduced point cloud to file: " + output_file_name << endl;

	// write header
	output_file << "ply"<< endl << "format ascii 1.0" << endl << "element vertex " << new_clusters.size() << endl;
	output_file << "property float x" << endl << "property float y" << endl << "property float z" << endl;
	output_file << "property uchar red" << endl << "property uchar green" << endl << "property uchar blue" << endl;
	output_file << "property float nx" << endl << "property float ny" << endl << "property float nz" << endl;

	for (size_t i = 0; i < new_clusters.size(); i++)
	{
		string line; // for simple buffering

		for (size_t j = 0; j < 9; j++)
		{
			// goes through all clusters, takes points from index 0 (centroid of that cluster) and writes its array elements (coordinates, color and normal vectors)
			line += points[new_clusters[i][0]][j];
		}

		output_file << line << endl;
	}

	cout << "Reduced point cloud successfully exported to file: " << output_file_name << endl << endl;
	cout << "Point cloud was reduced from " << points.size() << " points to " << new_clusters.size() << " points." << endl;
	cout << "That is " << new_clusters.size() / points.size() * 100 << "%.";
}

/** @brief Entry point. Arguments should contain filename as string, Space Interval Threshold (DT) as float 
 *	and normal Normal Vector Deviation Threshold (NT) as float.
 *	If any of these arguments is missing or is invalid, user is asked to provide them to console.
*/
int main(const int argc, char* argv[])
{
	string input_file_name = process_args(argc, argv);

	try
	{
		import_point_cloud(input_file_name);
	}
	catch (const std::exception&)
	{
		cout << endl << endl << "Error! File " + input_file_name + " was not successfully imported or parsed!";
		return -1;
	}

	build_tree();

	cluster_initialization();

	//const vector<int> boundary_clusters = boundary_cluster_detection();
	//boundary_cluster_subdivision(boundary_clusters);

	//main_cluster_subdivision();

	const string output_file_name = input_file_name.substr(0, input_file_name.size() - 4) + "_REDUCED" + file_name_extention;

	try
	{
		export_point_cloud(output_file_name);
	}
	catch (const std::exception&)
	{
		cout << endl << endl << "Error! Could not write to output file (" + output_file_name + ").";
		return -1;
	}

	return 0;
}
