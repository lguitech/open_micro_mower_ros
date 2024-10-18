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


#include <ros/ros.h>
#include "astar.hpp"
#include <algorithm>
#include <math.h>


using namespace std::placeholders;

Node::Node(mr_local_point coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}


Generator::Generator()
{
    setDiagonalMovement(true);
    setHeuristic(&Heuristic::euclidean);
    direciton_diagonal = {
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };

    direction8 = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}


void Generator::setMapData(int width, int height, std::vector<int8_t>& vecData)
{
    this->worldSize.x = width;
    this->worldSize.y = height;
    this->vecMapData = &vecData;
}
void Generator::setObstacleData(LocalPointHashMap& obstacle)
{
	this->obstacleLookup = obstacle;
}



void Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void Generator::setHeuristic(HeuristicFunction heuristic_)
{
    //heuristic = std::bind(heuristic_, _1, _2);
	heuristic = heuristic_;
}



bool Generator::isObstacle(const mr_local_point& coordinates_) 
{
    if (obstacleLookup.size() == 0) {
        return false;
    }

    auto foundNodeIterator = obstacleLookup.find(coordinates_);


    if (foundNodeIterator != obstacleLookup.end()) {
        return true;            
    } 
    else {
        return false;
    }
}



bool Generator::queryNearBoundary(const mr_local_point& point)
{
	return false;
    int size = direction8.size();
    for (int i=0; i<size; i++)  {
        mr_local_point curr = point + direction8[i];
        if (detectCollision(curr)) {
            return true;
        } 
    }
    return false;
}

bool Generator::queryNearObstacle(const mr_local_point& point)
{
	return false;
    int size = direction8.size();
    for (int i=0; i<size; i++)  {
        mr_local_point curr = point + direction8[i];
        if (isObstacle(curr)) {
            return true;
        } 
    }
    return false;
}


void Generator::findValidPoint(mr_local_point& input ,mr_local_point& output)
{
	LocalPointHashMap visited;
    std::queue<mr_local_point> pointsToCheck;
    mr_local_point intputCopy = input;

    intputCopy.x = std::max(intputCopy.x, 0);
    intputCopy.y = std::max(intputCopy.y, 0);

    intputCopy.x = std::min(intputCopy.x, worldSize.x-1);
    intputCopy.y = std::min(intputCopy.y, worldSize.y-1);
    
    visited.insert(intputCopy);

	pointsToCheck.push(intputCopy);

    while (!pointsToCheck.empty()) {
        mr_local_point currentPoint = pointsToCheck.front();
        pointsToCheck.pop();

        int index = currentPoint.y * worldSize.x + currentPoint.x;
        if (vecMapData->at(index) == MAP_CELL_FREE && !isObstacle(currentPoint)) {
            output = currentPoint;
            return;
        }

        for (const mr_local_point &dir : direction8) {
            mr_local_point neighbor = currentPoint + dir;

            if (neighbor.x >= 0 && neighbor.x < worldSize.x &&
                neighbor.y >= 0 && neighbor.y < worldSize.y &&
                visited.find(neighbor) == visited.end()) 
			{
                visited.insert(neighbor);				
                pointsToCheck.push(neighbor);
            }
        }
    }
}


CoordinateList Generator::findPath(mr_local_point source_, mr_local_point target_, bool extendValid)
{
    CoordinateList pathResult;

	bool collisionSource = detectCollision(source_);
	if (collisionSource) {
		return pathResult;
	}

    mr_local_point targetReconfirmed;

    bool collisionTarget = detectCollision(target_);


    if (!extendValid) {
        if (collisionTarget) {
            return pathResult;
        }
        else {
            targetReconfirmed = target_;
        }
    }
    else {
        if (collisionTarget) {
            findValidPoint(target_, targetReconfirmed);
        }
        else {
            targetReconfirmed = target_;
        }
    }
    

    bool success = false;
    Node *current = nullptr;
    openSetLookup.clear();
    closedSetLookup.clear();

    PriorityQueue openSet;
    NodeSet closedSet;

    Node* sourceNode = new Node(source_);
    openSet.push(sourceNode);
    openSetLookup.insert(sourceNode);

    while (!openSet.empty()) {
        current = openSet.top();
        openSet.pop();
        openSetLookup.erase(current);

        if (current->coordinates == targetReconfirmed) {
            success = true;
            break;
        }

        closedSet.push_back(current);
        closedSetLookup.insert(current);

        for (uint i = 0; i < directions; ++i) {
            mr_local_point newCoordinates(current->coordinates + direction8[i]);
            if (detectCollision(newCoordinates) || findNodeOnList(closedSetLookup, newCoordinates)) 
            {
                continue;
            }
            if (specialCase(current->coordinates, newCoordinates)) {
                continue;;
            }

            uint score = ((i < 4) ? 100 : 141);
            if (queryNearBoundary(current->coordinates) || queryNearObstacle(current->coordinates)) {
                score *= 2;
            }

            uint totalCost = current->G + score;

            Node *successor = findNodeOnList(openSetLookup, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, targetReconfirmed);
                openSet.push(successor);
                openSetLookup.insert(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    
    if (success) {
        while (current != nullptr) {

            pathResult.emplace_back(current->coordinates);
            current = current->parent;
        }
    }

    releaseNodes_openSet(openSet);
    releaseNodes_closedSet(closedSet);

    return pathResult;
}

Node* Generator::findNodeOnList(NodeSetLookup& nodeSetLookup, mr_local_point coordinates_) 
{
    Node targetNode = {coordinates_, nullptr};

    auto foundNodeIterator = nodeSetLookup.find(&targetNode);

    if (foundNodeIterator != nodeSetLookup.end()) {
        Node* foundNode = *foundNodeIterator;
        return foundNode;
    } 
    else {
        return nullptr;
    }

}

void Generator::releaseNodes_openSet(PriorityQueue& nodeSet)
{
    while (!nodeSet.empty()) {
        Node* node = nodeSet.top();
        nodeSet.pop();
        delete node;
    }
}

void Generator::releaseNodes_closedSet(NodeSet& nodeSet)
{
    for (auto it = nodeSet.begin(); it != nodeSet.end();) {
        delete *it;
        it = nodeSet.erase(it);
    }
}

bool Generator::specialCase(mr_local_point coord1, mr_local_point coord2)
{
	return false;
    mr_local_point diff = coord2 - coord1;
    for (int i=0; i<direciton_diagonal.size(); i++) {
        if (diff == direciton_diagonal.at(i)) {
            mr_local_point coordTest1(coord1.x, coord2.y);
            mr_local_point coordTest2(coord2.x, coord1.y);
            if (detectCollision(coordTest1) && detectCollision(coordTest2)) {
                return true;
            }
        }
    }
    return false;
}

bool Generator::detectCollision(mr_local_point coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y) {
        return true;
    }
    if (vecMapData->at(coordinates_.y * worldSize.x + coordinates_.x) != MAP_CELL_FREE){
        return true;
    }

    if (isObstacle(coordinates_)) {
        return true;
    }
    return false;
}

mr_local_point Heuristic::getDelta(mr_local_point source_, mr_local_point target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

uint Heuristic::manhattan(mr_local_point source_, mr_local_point target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(100 * (delta.x + delta.y));
}

uint Heuristic::euclidean(mr_local_point source_, mr_local_point target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(100 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

uint Heuristic::octagonal(mr_local_point source_, mr_local_point target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 100 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
