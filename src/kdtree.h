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

template<typename PointT>
struct KdTree
{
	Node* root;

	typename pcl::PointCloud<PointT>::Ptr cloud;

	KdTree(typename pcl::PointCloud<PointT>::Ptr cloud)
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
			uint cd = depth % 3;

			if(point->getArray3fMap()[cd] < ((*node)->point->getArray3fMap()[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
	}

	void insert(pcl::PointXYZI *point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(pcl::PointXYZI *target, Node* node, int depth, float distanceToL, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			auto tp = target->getArray3fMap();
			auto np = node->point->getArray3fMap();

			if((np[0]>=(tp[0]-distanceToL)&&np[0]<=(tp[0]+distanceToL)) 
			&& (np[1]>=(tp[1]-distanceToL)&&np[1]<=(tp[1]+distanceToL))
			&& (np[1]>=(tp[2]-distanceToL)&&np[2]<=(tp[2]+distanceToL)))
			{
				float dx = np[0]-tp[0];
				float dy = np[1]-tp[1];
				float dz = np[2]-tp[2];
				float distance = sqrt(dx*dx+dy*dy+dz*dz);
				// float distance = pcl::euclideanDistance(*target, *node->point);
				if (distance <= distanceToL)
					ids.push_back(node->id);
			}

			// check across boundary
			if((tp[depth%3]-distanceToL)<np[depth%3])
				searchHelper(target, node->left, depth+1, distanceToL, ids);
			if((tp[depth%3]+distanceToL)>np[depth%3])
				searchHelper(target, node->right, depth+1, distanceToL, ids);
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


