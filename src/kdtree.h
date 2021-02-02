#include "render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			int cd = depth % 3;

			if (point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);

		}

	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(Node* node, int depth, std::vector<int>& ids, std::vector<float> target, float distanceTol)
	{
		if (node == NULL)
			return;

		if (node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol) && node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol))
		{
			float dist = sqrt(pow(target[0] - node->point[0], 2) +pow(target[1] - node->point[1], 2));
			if (dist <= distanceTol)
				ids.push_back(node->id);
		}

		int cd = depth % 3;

		if (target[cd] - distanceTol < node->point[cd])
			searchHelper(node->left, depth + 1, ids, target, distanceTol);

		if (target[cd] + distanceTol > node->point[cd])
			searchHelper(node->right, depth + 1, ids, target, distanceTol);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, ids, target, distanceTol);
		return ids;
	}

};
