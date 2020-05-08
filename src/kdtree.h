#ifndef KDTREE_H
#define KDTREE_H
#include "render/render.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <pcl/common/distances.h>

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI *point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI *setPoint, int setId)
	:	point(setPoint), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	const int n_dim = 3;

	Node* root;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

	KdTree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	: root(NULL)
	{
		this->cloud = cloud;
	
		for (int i=0; i < cloud->points.size(); i++)
        	this->insert(&cloud->points[i],i);
	}

	void insertHelper(Node** node, uint depth, pcl::PointXYZI *point, int id) {

		// Tree is empty
		if(*node==NULL){
			*node = new Node(point,id);
		} else {
			// Calculate current dim
			uint cd = depth % n_dim;

			if(point->getArray4fMap()[cd] < ((*node)->point->getArray4fMap()[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
	}

	void insert(pcl::PointXYZI *point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(pcl::PointXYZI *target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			auto tp = target->getArray4fMap();
			auto np = node->point->getArray4fMap();
			

			if((np[0]>=(tp[0]-distanceTol)&&np[0]<=(tp[0]+distanceTol)) 
			&& (np[1]>=(tp[1]-distanceTol)&&np[1]<=(tp[1]+distanceTol))
			&& (np[2]>=(tp[2]-distanceTol)&&np[2]<=(tp[2]+distanceTol)))
			{
				float dx = np[0]-tp[0];
				float dy = np[1]-tp[1];
				float dz = np[2]-tp[2];
				float distance = sqrt(dx*dx+dy*dy+dz*dz);
				// float distance = sqrt(dx*dx+dy*dy);
				// float distance = pcl::euclideanDistance(*target, *node->point);
				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// check across boundary
			if((tp[depth%n_dim]-distanceTol)<np[depth%n_dim])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((tp[depth%n_dim]+distanceTol)>np[depth%n_dim])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI *target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids); 
		return ids;
	}
};

#endif


