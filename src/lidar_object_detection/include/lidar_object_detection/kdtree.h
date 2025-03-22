// #ifndef KDTREE_H
// #define KDTREE_H

// #include <vector>

// struct Node
// {
//     pcl::PointXYZI point; // Change from generic PointT to pcl::PointXYZI
//     int id;
//     Node* left;
//     Node* right;

//     Node(pcl::PointXYZI p, int setId)
//         : point(p), id(setId), left(NULL), right(NULL)
//     {}

//     ~Node()
//     {
//         delete left;
//         delete right;
//     }
// };

// struct KdTree
// {
//     Node* root;

//     KdTree()
//         : root(NULL)
//     {}

//     ~KdTree()
//     {
//         delete root;
//     }

//     // Insert function to insert a point into the KD tree
//     void insert(pcl::PointXYZI point, int id)
//     {
//         Node** node = &root;
//         int depth = 0;

//         while (*node != NULL) {
//             // Determine current dimension (assuming 3D points: x, y, z)
//             int tree_dim = depth % 3;

//             // Traverse left or right based on the current dimension
//             if (point.data[tree_dim] < ((*node)->point.data[tree_dim])) {
//                 node = &((*node)->left);
//             } else {
//                 node = &((*node)->right);
//             }

//             depth++;
//         }

//         // Insert the new node
//         *node = new Node(point, id);
//     }

//     // Search function to find the points within a given distance of the target point
//     void searchHelper(Node* node, pcl::PointXYZI target, float distanceTol, int depth, std::vector<int>& ids)
//     {
//         if (node == NULL) return;

//         // Check if the node is within the target bounding box
//         if (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol) &&
//             node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol) &&
//             node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol))
//         {
//             // Calculate the distance between points
//             float distance = sqrt(pow((node->point.x - target.x), 2) + 
//                                   pow((node->point.y - target.y), 2) + 
//                                   pow((node->point.z - target.z), 2));
//             if (distance <= distanceTol) {
//                 ids.push_back(node->id);
//             }
//         }

//         // Determine current dimension (x, y, or z)
//         int tree_dim = depth % 3;

//         // Recursively search children based on their bounds
//         if (target.data[tree_dim] - distanceTol < node->point.data[tree_dim]) {
//             searchHelper(node->left, target, distanceTol, depth + 1, ids);
//         }
//         if (target.data[tree_dim] + distanceTol > node->point.data[tree_dim]) {
//             searchHelper(node->right, target, distanceTol, depth + 1, ids);
//         }
//     }

//     // Return a list of point IDs in the tree that are within distance of target
//     std::vector<int> search(pcl::PointXYZI target, float distanceTol)
//     {
//         std::vector<int> ids;
//         searchHelper(root, target, distanceTol, 0, ids);
//         return ids;
//     }
// };

// #endif

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	// Node(std::vector<float> arr, int setId)
	// :	point(arr), id(setId), left(NULL), right(NULL)
	// {}

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(PointT point, int id)
	{
		Node<PointT>** node = &root;
		int depth = 0;

		while (*node != NULL) {
			// Determine current dimension (assuming 3D points)
			int tree_dim = depth % 3;

			// Traverse left or right based on the current dimension
			if (point.data[tree_dim] < ((*node)->point.data[tree_dim])) {
				node = &((*node)->left);
			} else {
				node = &((*node)->right);
			}

			depth++;
		}

		// Insert the new node
		*node = new Node<PointT>(point, id);
	
	}

	
	void searchHelper(Node<PointT>* node, PointT target, float distanceTol, int depth, std::vector<int>& ids) {
		if (node == NULL) return;

		// Check if the node is within the target bounding box
		if (node->point.data[0] >= (target.data[0] - distanceTol) && node->point.data[0] <= (target.data[0] + distanceTol) &&
			node->point.data[1] >= (target.data[1] - distanceTol) && node->point.data[1] <= (target.data[1] + distanceTol) &&
			node->point.data[2] >= (target.data[2] - distanceTol) && node->point.data[2] <= (target.data[2] + distanceTol)) {

			float distance = sqrt(pow((node->point.data[0] - target.data[0]), 2) + pow((node->point.data[1] - target.data[1]), 2) +
										 pow((node->point.data[2] - target.data[2]), 2));
			if (distance <= distanceTol) {
				ids.push_back(node->id);
			}
		}

		// Determine current dimension (assuming #D points)
		int tree_dim = depth % 3;

		// Recursively search children based on their bounds
		if (target.data[tree_dim] - distanceTol < node->point.data[tree_dim]) {
			searchHelper(node->left, target, distanceTol, depth + 1, ids);
		}
		if (target.data[tree_dim] + distanceTol > node->point.data[tree_dim]) {
			searchHelper(node->right, target, distanceTol, depth + 1, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol) {
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
	

};

#endif




