#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <exception>
#include <functional>

namespace kdt
{
	/** @brief k-d tree class.
	*/
	template <class PointT>
	class kd_tree
	{
	public:
		/** @brief The constructors.
		*/
		kd_tree() : root_(nullptr) {};

		explicit kd_tree(const std::vector<PointT>& points) : root_(nullptr)
		{
			build(points);
		}

		/** @brief The destructor.
		*/
		~kd_tree()
		{
			clear();
		}

		/** @brief Re-builds k-d tree.
		*/
		void build(const std::vector<PointT>& points)
		{
			clear();

			points_ = points;

			std::vector<int> indices(points.size());
			std::iota(std::begin(indices), std::end(indices), 0);

			root_ = build_recursive(indices.data(), static_cast<int>(points.size()), 0);
		}

		/** @brief Clears k-d tree.
		*/
		void clear()
		{ 
			clear_recursive(root_);
			root_ = nullptr;
			points_.clear();
		}

		/** @brief Validates k-d tree.
		*/
		bool validate() const
		{
			try
			{
				validate_recursive(root_, 0);
			}
			catch (const kd_tree_exception&)
			{
				return false;
			}

			return true;
		}

		/** @brief Searches the nearest neighbor.
		*/
		int nn_search(const PointT& query, double* min_dist = nullptr) const
		{
			int guess;
			double _minDist = std::numeric_limits<double>::max();

			nn_search_recursive(query, root_, &guess, &_minDist);

			if (min_dist)
				*min_dist = _minDist;

			return guess;
		}

		/** @brief Searches k-nearest neighbors.
		*/
		std::vector<int> knn_search(const PointT& query, int k) const
		{
			knn_queue queue(k);
			knn_search_recursive(query, root_, queue, k);
			
			std::vector<int> indices(queue.size());

			for (size_t i = 0; i < queue.size(); i++)
				indices[i] = queue[i].second;

			return indices;
		}

		/** @brief Searches neighbors within radius.
		*/
		std::vector<int> radius_search(const PointT& query, const double radius) const
		{
			std::vector<int> indices;

			radius_search_recursive(query, root_, indices, radius);

			return indices;
		}

	private:

		/** @brief k-d tree node.
		*/
		struct tree_node
		{
			int idx;       //!< index to the original point
			tree_node* next[2]; //!< pointers to the child nodes
			int axis;      //!< dimension's axis

			tree_node() : idx(-1), axis(-1)
			{
				next[0] = next[1] = nullptr;
			}
		};

		/** @brief k-d tree exception.
		*/
		class kd_tree_exception : public std::exception
		{
			using std::exception::exception;
		};

		/** @brief Bounded priority queue.
		*/
		template <class T, class Compare = std::less<T>>
		class bounded_priority_queue
		{
		public:

			bounded_priority_queue() = delete;

			explicit bounded_priority_queue(const size_t bound) : bound_(bound)
			{
				elements_.reserve(bound + 1);
			};

			void push(const T& val)
			{
				auto it = std::find_if(std::begin(elements_), std::end(elements_),
					[&](const T& element)
				{
					return Compare()(val, element);
				});
				elements_.insert(it, val);

				if (elements_.size() > bound_)
					elements_.resize(bound_);
			}

			const T& back() const
			{
				return elements_.back();
			};

			const T& operator[](size_t index) const
			{
				return elements_[index];
			}

			size_t size() const
			{
				return elements_.size();
			}

		private:
			size_t bound_;
			std::vector<T> elements_;
		};

		/** @brief Priority queue of <distance, index> pair.
		*/
		using knn_queue = bounded_priority_queue<std::pair<double, int>>;

		/** @brief Builds k-d tree recursively.
		*/
		tree_node* build_recursive(int* indices, const int npoints, int depth)
		{
			if (npoints <= 0)
				return nullptr;

			const int axis = depth % PointT::dimension;
			const int mid = (npoints - 1) / 2;

			std::nth_element(indices, indices + mid, indices + npoints, 
				[&](int lhs, int rhs)
			{
				return points_[lhs][axis] < points_[rhs][axis];
			});

			tree_node* node = new tree_node();
			node->idx = indices[mid];
			node->axis = axis;

			node->next[0] = build_recursive(indices, mid, depth + 1);
			node->next[1] = build_recursive(indices + mid + 1, npoints - mid - 1, depth + 1);

			return node;
		}

		/** @brief Clears k-d tree recursively.
		*/
		void clear_recursive(tree_node* node)
		{
			if (node == nullptr)
				return;

			if (node->next[0])
				clear_recursive(node->next[0]);

			if (node->next[1])
				clear_recursive(node->next[1]);

			delete node;
		}

		/** @brief Validates k-d tree recursively.
		*/
		void validate_recursive(const tree_node* node, const int depth) const
		{
			if (node == nullptr)
				return;

			const int axis = node->axis;
			const tree_node* node0 = node->next[0];
			const tree_node* node1 = node->next[1];

			if (node0 && node1)
			{
				if (points_[node->idx][axis] < points_[node0->idx][axis])
					throw kd_tree_exception();

				if (points_[node->idx][axis] > points_[node1->idx][axis])
					throw kd_tree_exception();
			}

			if (node0)
				validate_recursive(node0, depth + 1);

			if (node1)
				validate_recursive(node1, depth + 1);
		}

		static double distance(const PointT& p, const PointT& q)
		{
			double dist = 0;
			for (size_t i = 0; i < PointT::dimension; i++)
				dist += (p[i] - q[i]) * (p[i] - q[i]);
			return sqrt(dist);
		}

		/** @brief Searches the nearest neighbor recursively.
		*/
		void nn_search_recursive(const PointT& query, const tree_node* node, int* guess, double* min_dist) const
		{
			if (node == nullptr)
				return;

			const PointT& train = points_[node->idx];

			const double dist = distance(query, train);
			if (dist < *min_dist)
			{
				*min_dist = dist;
				*guess = node->idx;
			}

			const int axis = node->axis;
			const int dir = query[axis] < train[axis] ? 0 : 1;
			nn_search_recursive(query, node->next[dir], guess, min_dist);

			const double diff = fabs(query[axis] - train[axis]);
			if (diff < *min_dist)
				nn_search_recursive(query, node->next[!dir], guess, min_dist);
		}

		/** @brief Searches k-nearest neighbors recursively.
		*/
		void knn_search_recursive(const PointT& query, const tree_node* node, knn_queue& queue, const int k) const
		{
			if (node == nullptr)
				return;

			const PointT& train = points_[node->idx];

			const double dist = distance(query, train);
			queue.push(std::make_pair(dist, node->idx));

			const int axis = node->axis;
			const int dir = query[axis] < train[axis] ? 0 : 1;
			knn_search_recursive(query, node->next[dir], queue, k);

			const double diff = fabs(query[axis] - train[axis]);
			if (static_cast<int>(queue.size()) < k || diff < queue.back().first)
				knn_search_recursive(query, node->next[!dir], queue, k);
		}

		/** @brief Searches neighbors within radius.
		*/
		void radius_search_recursive(const PointT& query, const tree_node* node, std::vector<int>& indices, const double radius) const
		{
			if (node == nullptr)
				return;

			const PointT& train = points_[node->idx];

			const double dist = distance(query, train);
			if (dist < radius)
				indices.push_back(node->idx);

			const int axis = node->axis;
			const int dir = query[axis] < train[axis] ? 0 : 1;
			radius_search_recursive(query, node->next[dir], indices, radius);

			const double diff = fabs(query[axis] - train[axis]);
			if (diff < radius)
				radius_search_recursive(query, node->next[!dir], indices, radius);
		}

		tree_node* root_;				//!< root node
		std::vector<PointT> points_;	//!< points
	};
} // kdt

#endif // !KDTREE_H