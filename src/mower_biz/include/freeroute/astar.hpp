/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROS] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#ifndef __ASTAR_HPP__
#define __ASTAR_HPP__

#include <iostream>
#include <vector>
#include <functional>
#include <set>
#include <queue>
#include <unordered_set>
#include "mr_navi_types.h"


using uint = unsigned int;
using HeuristicFunction = uint (*)(mr_local_point, mr_local_point);

using CoordinateList = std::vector<mr_local_point>;

struct Node
{
    uint G, H;
    mr_local_point coordinates;
    Node *parent;

    Node(mr_local_point coord_, Node *parent_ = nullptr);
};


struct NodeHash {
    std::size_t operator()(const Node* node) const {
        std::size_t h1 = std::hash<int>()(node->coordinates.x);
        std::size_t h2 = std::hash<int>()(node->coordinates.y);

        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
};

struct NodeEqual {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->coordinates.x == rhs->coordinates.x && lhs->coordinates.y == rhs->coordinates.y;
    }
};

struct NodeCompare {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return (lhs->G + lhs->H) > (rhs->G + rhs->H);
    }
};

using PriorityQueue = std::priority_queue<Node*, std::vector<Node*>, NodeCompare>;

using NodeSet = std::vector<Node*>;

using NodeSetLookup = std::unordered_set<Node*, NodeHash, NodeEqual>;


class Generator
{
private:    
    bool specialCase(mr_local_point coord1, mr_local_point coord2);
    bool detectCollision(mr_local_point coordinates_);
    Node* findNodeOnList(NodeSetLookup& nodeSetLookup,
            mr_local_point coordinates_);

    //void releaseNodes(NodeSet& nodes_);
    void releaseNodes_openSet(PriorityQueue& nodeSet);
    void releaseNodes_closedSet(NodeSet& nodeSet);

    NodeSetLookup openSetLookup;
    NodeSetLookup closedSetLookup;

    mr_local_point worldSize;

    std::vector<int8_t>* vecMapData;

    LocalPointHashMap obstacleLookup;
    
    HeuristicFunction heuristic;
    CoordinateList direction8;
    CoordinateList direciton_diagonal;
    uint directions;

    bool isObstacle(const mr_local_point& coordinates_);
    bool queryNearBoundary(const mr_local_point& point);
    bool queryNearObstacle(const mr_local_point& point);
    void findValidPoint(mr_local_point& input ,mr_local_point& output);
public:
    Generator();
    void setMapData(int width, int height, std::vector<int8_t>& vecData);
    void setObstacleData(LocalPointHashMap& obstacle);

    void setDiagonalMovement(bool enable_);
    void setHeuristic(HeuristicFunction heuristic_);
    CoordinateList findPath(mr_local_point source_, mr_local_point target_, bool extendValid = false);

};

class Heuristic
{
    static mr_local_point getDelta(mr_local_point source_, mr_local_point target_);

public:
    static uint manhattan(mr_local_point source_, mr_local_point target_);
    static uint euclidean(mr_local_point source_, mr_local_point target_);
    static uint octagonal(mr_local_point source_, mr_local_point target_);
};


#endif
