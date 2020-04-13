/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if (!root) 
		{
			root = new Node(point, id);
			return;
		}

		Node* new_node = new Node(point, id);
		Node* temp = root;
		int depth = 0;
		while (temp)
		{
			if (point[depth % 2] > temp->point[depth % 2])
			{
				if (!temp->right)
				{
					temp->right = new_node;
					return;	
				}
				temp = temp->right;
			} else
			{
				if (!temp->left)
				{
					temp->left = new_node;
					return;
				}
				temp = temp->left;
			}
			depth++; 
		}
	}

	// return a list of point ids in the tree that are within distance of target
	void searchHelp(std::vector<float> target, float distanceTol, int depth, Node* node, std::vector<int> &ids)
	{
		if (node)
		{	
			if ( (target[2] <= node->point[0]) && (node->point[0] <= target[3]) && (target[4] <= node->point[1]) && (node->point[1] <= target[5]) )
			{
				float d = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
				if (d <= distanceTol)
					ids.push_back(node->id);
			}
			if ( node->point[depth%2] < (target[depth%2] + distanceTol) )
				searchHelp(target, distanceTol, depth+1, node->right, ids);
			if ( node->point[depth%2] > (target[depth%2] - distanceTol) )
				searchHelp(target, distanceTol, depth+1, node->left, ids);;
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		Node* temp = root;
		int depth = 0;
		float dx1 = target[0] - distanceTol;
		float dx2 = target[0] + distanceTol;
		float dy1 = target[1] - distanceTol;
		float dy2 = target[1] + distanceTol;
		target.push_back(dx1);
		target.push_back(dx2);
		target.push_back(dy1);
		target.push_back(dy2);
		searchHelp(target, distanceTol, depth, temp, ids);
		return ids;
	}

};




