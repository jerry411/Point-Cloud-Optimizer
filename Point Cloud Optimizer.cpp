#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "point.hpp"
#include <sstream>
#include <iomanip>
#include "rply.h"
#include "nanoflann.hpp"
#include "point_cloud.hpp"

using namespace std;
using namespace nanoflann;

typedef vector<size_t> cluster;
typedef KDTreeSingleIndexAdaptor <L2_Simple_Adaptor<float, point_cloud<float> >, point_cloud<float>, 3> tree; // K-D tree holding indices from "points" vector

point_cloud<float> cloud; // point cloud itself holding actual data to points
vector<cluster> initial_clusters; // cluster holds indices to its members (index 0 refers to cluster centroid)
vector<cluster> new_clusters; // used as final storage of clusters after subdivision of initial clusters

// Space Interval Threshold (DT) - largest distance from cluster centroid to any cluster member
float space_interval_dt;
float space_interval_dt_default = 1;

// Normal Vector Deviation Threshold (NT) - largest deviation of normal vectors of any pair of cluster members (otherwise cluster is divided)
float vector_deviation_nt;
float vector_deviation_nt_default = 0.5;

enum user_def_variables { space_interval_var, vector_deviation_var };

string file_name_extention(".ply");
string default_file_name("PointCloud" + file_name_extention);


int position = 0;
float buffer[9];
/** @brief Callback for parse. This method is called for every element found.
*/
static int vertex_cb(const p_ply_argument argument)
{
	buffer[position] = static_cast<float>(ply_get_argument_value(argument));
	position++;

	if (position == 9)
	{
		position = 0;
		cloud.points.emplace_back(buffer);
	}

	return 1;
}

/** @brief Uses RPly to parse point cloud from external ASCII .ply file.
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
void import_point_cloud(const string& file_name)
{
	cout << endl << "Importing and parsing file: " + file_name << endl;

	const p_ply ply = ply_open(file_name.c_str(), nullptr, 0, nullptr);

	if (!ply) 
		throw exception();

	if (!ply_read_header(ply)) 
		throw exception();

	const long number_of_elements = ply_set_read_cb(ply, "vertex", "x", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "y", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "z", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "red", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "green", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "blue", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "nx", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "ny", vertex_cb, nullptr, 0);
	ply_set_read_cb(ply, "vertex", "nz", vertex_cb, nullptr, 1);

	cloud.points.reserve(number_of_elements);

	if (!ply_read(ply))
		throw exception();
	
	ply_close(ply);
}

/** @brief Creates initial clusters. If point is not marked, it becames centroid of new cluster. 
 *	This new cluster contains non-marked neighbours of centroid whose distance is less than or equal to Space Interval Threshold (DT).
*/
void cluster_initialization(tree& my_tree)
{
	cout << "Initializing clusters." << endl;

	for (size_t i = 0; i < cloud.points.size(); i++)
	{
		if (!cloud.points[i].is_marked)
		{
			cloud.points[i].is_centroid = true;

			float* centroid = cloud.points[i].data; // index to centroid is at index 0 in cluster
			const float radius = space_interval_dt;
			vector<std::pair<size_t, float>> indices_dists;

			my_tree.radiusSearch(centroid, radius, indices_dists, SearchParams());

			// create new cluster
			initial_clusters.resize(initial_clusters.size() + 1);
			vector<size_t>& current_cluster = initial_clusters[initial_clusters.size() - 1];
			current_cluster.reserve(indices_dists.size());

			// fill the new cluster
			for (size_t j = 0; j < indices_dists.size(); j++)
			{
				size_t point_index = indices_dists[j].first;

				if (!cloud.points[point_index].is_marked) // do not copy indices to marked points to cluster
				{
					current_cluster.push_back(point_index);
					cloud.points[point_index].is_marked = true;
				}
			}
		}
	}
}

/** @brief Decides whether value of specific user variable is valid.
*/
bool user_var_value_is_valid(const float value, const user_def_variables& user_var)
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
string text_for_user_variable(const user_def_variables& user_var)
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
float default_for_user_variable(const user_def_variables& user_var)
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
float manual_float_input(const user_def_variables& user_var)
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
float process_float_arg(const int argc, char** argv, const int index, const user_def_variables& user_var)
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

		if (file_name.length() < 5 || file_name.find(file_name_extention) == string::npos)
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
		else if (file_name.length() < 5 || file_name.find(file_name_extention) == string::npos)
		{
			file_name += file_name_extention;
		}
	}

	space_interval_dt = process_float_arg(argc, argv, 2, space_interval_var); // third argument is Space Interval Threshold (DT)

	vector_deviation_nt = process_float_arg(argc, argv, 3, vector_deviation_var); // fourth argument is Normal Vector Deviation Threshold (NT)

	return file_name;
}

/** @brief Cluster is boudary if there are less less than 6 centroids in vicinity of sqrt(3) * space_interval_dt.
*/
bool is_boundary_cluster (const cluster& init_cluster, const tree& my_tree)
{
	float* centroid = cloud.points[init_cluster[0]].data; // index to centroid is at index 0 in cluster
	const float radius = static_cast<float>(sqrt(3) * space_interval_dt);
	vector<std::pair<size_t, float>> indices_dists;

	my_tree.radiusSearch(centroid, radius, indices_dists, SearchParams());

	int number_of_centroid = 0;

	for (size_t i = 0; i < indices_dists.size(); i++)
	{
		if (cloud.points[indices_dists[i].first].is_centroid)
		{
			number_of_centroid++;
		}
	}

	// there is always one centroid from initial cluster - that one does not count to neighbouring centroids count but it is always returned from knn_search
	return number_of_centroid < 7;
}

/** @brief Returns indices to clusters which are boundary clusters
*/
vector<size_t> boundary_cluster_detection(const tree& tree)
{
	vector<size_t> cluster_indices;

	for (size_t i = 0; i < initial_clusters.size(); i++)
	{
		if (is_boundary_cluster(initial_clusters[i], tree))
		{
			cluster_indices.push_back(i);
		}
	}

	return cluster_indices;
}

/*
void boundary_cluster_subdivision(const vector<int>& boundary_clusters)
{
	
}*/

/** @brief Standard deviation of normal vectors of 2 points. Normal vectors are expected to be normalized, therefore return value is between 0 and 1.
*/
float standard_deviation(const point& p1, const point& p2)
{
	float sum = 0;

	for (size_t i = 0; i < 3; i++)
		sum += pow(p1.data[i] - p2.data[i], 2);

	return sqrt((sum) / 2);
}

/** @brief Returns new means (indices to cluster of pair of points with largest deviation of normal vectors).
 *	If this deviation is larger than Normal Vector Deviation Threshold (NT), cluster should be divided.
 *	Returns pair (-1, -1) as indicator if cluster should not be divided.
*/
pair<int, int> new_means(const cluster& cluster)
{
	// cluster with 1 member should not be divided; cluster division can be skipped if vector_deviation_nt is too close to 1
	if (cluster.size() <= 1 || vector_deviation_nt > 0.99999)
		return {-1, -1};

	float max_deviation = 0;
	int max_index1, max_index2;

	for (size_t i = 0; i < cluster.size() - 1; i++)
	{
		for (size_t j = i; j < cluster.size(); j++)
		{
			const float local_deviation = standard_deviation(cloud.points[cluster[i]], cloud.points[cluster[j]]);

			if (local_deviation > max_deviation)
			{
				max_deviation = local_deviation;
				max_index1 = static_cast<int>(i);
				max_index2 = static_cast<int>(j);
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

/** @brief Simplified k-means clustering algorithm with k=2 and non-moving predetermined centroid (1-iteration)
*/
pair<cluster, cluster> k_means_clustering(const cluster& init_cluster, const pair<int, int>& means)
{
	cluster temp1, temp2;

	temp1.push_back(init_cluster[means.first]);
	temp2.push_back(init_cluster[means.second]);

	for (size_t i = 0; i < init_cluster.size(); i++)
	{
		if (static_cast<int>(i) == means.first || static_cast<int>(i) == means.second)
			continue;

		const double distance_to_mean1 = cloud.points[init_cluster[i]].distance(cloud.points[init_cluster[means.first]]);
		const double distance_to_mean2 = cloud.points[init_cluster[i]].distance(cloud.points[init_cluster[means.second]]);

		if (distance_to_mean1 < distance_to_mean2)
			temp1.push_back(init_cluster[i]);
		else
			temp2.push_back(init_cluster[i]);
	}

	return { temp1, temp2 };
}

/** @brief Decides whether cluster should be divided. If yes, it is recursively divided using k-means. If no, it is added to new_clusters.
*/
void recursive_cluster_subdivision(const cluster& init_cluster)
{
	const pair<int, int> means = new_means(init_cluster);

	if (means.first == -1 || means.second == -1) // cluster should not be divided anymore
	{
		new_clusters.push_back(init_cluster);
	}
	else // recursively divide cluster
	{
		const pair<cluster, cluster> divided_clusters = k_means_clustering(init_cluster, means);

		// means became new centroids for new clusters
		cloud.points[init_cluster[0]].is_centroid = false;
		cloud.points[init_cluster[means.first]].is_centroid = true;
		cloud.points[init_cluster[means.second]].is_centroid = true;

		// recursion
		recursive_cluster_subdivision(divided_clusters.first);
		recursive_cluster_subdivision(divided_clusters.second);
	}
}

/** @brief Calls subdivision on all clusters
*/
void main_cluster_subdivision()
{
	cout << "Dividing clusters." << endl;

	for (size_t i = 0; i < initial_clusters.size(); i++)
	{
		recursive_cluster_subdivision(initial_clusters[i]);
	}
}

/** @brief Exports centroid from new_clusters. Exported file has same header and format as input file.
*/
void export_point_cloud(const string& output_file_name)
{
	ofstream output_file(output_file_name);

	cout << "Exporting reduced point cloud to file: " + output_file_name << endl;

	// write header
	output_file << "ply"<< endl << "format ascii 1.0" << endl << "element vertex " << new_clusters.size() << endl;
	output_file << "property float x" << endl << "property float y" << endl << "property float z" << endl;
	output_file << "property uchar red" << endl << "property uchar green" << endl << "property uchar blue" << endl;
	output_file << "property float nx" << endl << "property float ny" << endl << "property float nz" << endl;
	output_file << "end_header" << endl;

	for (size_t i = 0; i < new_clusters.size(); i++)
	{
		stringstream line_stream; // for simple buffering		

		for (size_t j = 0; j < 9; j++)
		{
			// goes through all clusters, takes points from index 0 (centroid of that cluster) and writes its array elements (coordinates, color and normal vectors)
			line_stream << std::setprecision(7) << cloud.points[new_clusters[i][0]].data[j];

			if (j < 8)
				line_stream << ' ';
		}

		output_file << line_stream.rdbuf() << endl;
	}

	cout << endl << endl << "Point cloud was reduced from " << cloud.points.size() << " points to " << new_clusters.size() << " points." << endl;
	cout << "That is " << new_clusters.size() / static_cast<float>(cloud.points.size()) * 100 << "%.";
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

		cout << endl << endl << "Press ANY key to exit the program...";
		std::getchar();

		return -1;
	}

	cout << "Building K-D tree." << endl;
	tree tree(3, cloud, KDTreeSingleIndexAdaptorParams(50));
	tree.buildIndex();

	cluster_initialization(tree);

	//const vector<size_t> boundary_clusters_indices = boundary_cluster_detection(tree);
	//boundary_cluster_subdivision(boundary_clusters);

	main_cluster_subdivision();

	const string output_file_name = input_file_name.substr(0, input_file_name.size() - 4) + "_REDUCED" + file_name_extention;

	try
	{
		export_point_cloud(output_file_name);
	}
	catch (const std::exception&)
	{
		cout << endl << endl << "Error! Could not write to output file (" + output_file_name + ").";

		cout << endl << endl << "Press ANY key to exit the program...";
		std::getchar();

		return -1;
	}

	cout << endl << endl << "Press ANY key to exit the program...";
	std::getchar();

	return 0;
}