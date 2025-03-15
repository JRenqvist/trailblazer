// This is the CPP file you will edit and turn in.
// Also remove these comments here and add your own, along with
// comments on every function and on complex code sections.
// TODO: write comment header for this file; remove this comment

#include <stack>
#include <queue>
#include <set>
#include <unordered_map>
#include <algorithm>
#include <limits.h>


#include "costs.h"
#include "trailblazer.h"
#include "pqueue.h"
// TODO: include any other headers you need; remove this comment
using namespace std;

vector<Node *> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    vector<Vertex*> path;

    // Reset graph incase we have previously ran it
    graph.resetData();

    // Add start to path
    path.push_back(start);

    // Mark start as visited
    start->visited = true;

    // Call recursive function
    DFSRec(graph, path, start, end);

    // If path only has start, there isn't a way to get to end
    if (path.back() != end) {
        path.clear();
    } else if (path.back() == end) {

        // Mark path as green
        for (Vertex* vertex : path) {
            vertex->setColor(GREEN);
        }
    } else {
        cout << "Error" << endl;
    }



    return path;
}

void DFSRec(BasicGraph& graph, vector<Vertex*>& path, Vertex* current, Vertex* end) {

    // Mark current as visited
    current->visited = true;

    // Add current to path
    path.push_back(current);

    // Check if we have found a correct path
    if (current == end) {
        return;
    } else {

        // For every neighbour to top, call recursively
        for (Vertex* neighbour : graph.getNeighbors(current)) {

            // Make sure we haven't previously visited the neighbour
            if (!neighbour->visited) {

                // Mark neighbour as yellow
                neighbour->setColor(YELLOW);

                // Call recursion on the neighbour
                DFSRec(graph, path, neighbour, end);

                // If we get here, we have backtracked.
                // Mark neighbour as gray
                neighbour->setColor(GRAY);

                // Remove last element in path (top) if we have not found a correct way
                if (path.back() != end) {
                    path.pop_back();
                } else {
                    return;
                }
            }
        }
    }
}

vector<Node *> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {

    // Clear graph incase we previously ran it
    graph.resetData();

    vector<Vertex*> path;

    // Create queue for vertexes to be visited
    queue<Vertex*> queue;

    // Map to store parent of each vertex
    unordered_map<Vertex*, Vertex*> parent;

    // Add start node to queue and mark as visited
    queue.push(start);
    start->visited = true;

    while (!queue.empty()) {

        // Get first item in queue
        Vertex* current = queue.front();
        queue.pop();

        // Mark as green
        current->setColor(GREEN);

        // If we have reached the target node, backtrack to find the path
        if (current == end) {

            while (current != nullptr) {
                path.push_back(current);
                current = parent[current];
            }

            // Reverse since its in the wrong order
            reverse(path.begin(), path.end());
            break;
        }

        // For all neighbours to that item
        for (Vertex* neighbour : graph.getNeighbors(current)) {

            // If not visited:
            if (!neighbour->visited) {
                // Add to queue and mark as visited
                queue.push(neighbour);
                neighbour->visited = true;

                // Set color to yellow
                neighbour->setColor(YELLOW);

                // Record current node as the parent of the neighbour
                parent[neighbour] = current;
            }
        }

    }

    return path;
}


vector<Node *> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    // TODO: implement this function; remove these comments
    //       (The function body code provided below is just a stub that returns
    //        an empty vector so that the overall project will compile.
    //        You should remove that code and replace it with your implementation.)
    vector<Vertex*> path;

    graph.resetData();
    // priority_queue that contains verticies
    PriorityQueue<Vertex*> pqueue;

    // Set all verticies cost to INF and add them to pqueue
    for (Vertex* v : graph.getNodeSet()) {
        v->cost = INT_MAX;
        pqueue.enqueue(v, v->cost);
    }

    // Set start node cost to 0
    start->cost = 0.0;

    // Adjust start in pqueue with new cost
    pqueue.changePriority(start, start->cost);

    while (!pqueue.isEmpty()) {

        // Initialize current node
        Vertex* current = pqueue.dequeue();
        current->visited = true;
        current->setColor(GREEN);

        // If current == end, we know this is the shortest path, since we use a priority queue
        if (current == end) {

            // If we get in here, we have found the lowest-cost path from start to end
            // Backtrack through the path and push to path variable
            while (current != NULL) {
                path.push_back(current);
                current = current->previous;
            }

            // Reverse path and return it
            reverse(path.begin(), path.end());
            return path;
        }

        // For all neighbours:
        for (Vertex* neighbour : graph.getNeighbors(current)) {

            // Make sure we haven't visited the neighbour before
            if (!neighbour->visited) {

                // Calculate cost to travel to neighbour
                Edge* edge = graph.getEdge(current, neighbour);
                double cost = current->cost + edge->cost;

                // If new cost is lower than old one, change
                if (cost < neighbour->cost) {
                    neighbour->cost = cost;
                    neighbour->previous = current;
                    neighbour->setColor(YELLOW);

                    // Rearrange in the pqueue to account of new cost of neighbour
                    pqueue.changePriority(neighbour, neighbour->cost);
                }
            }
        }
    }

    // If we get here, we couldn't find a path from start to finish
    return path;
}

vector<Node *> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {

    graph.resetData();
    vector<Vertex*> path;

    // Priority queue for storing the verticies
    PriorityQueue<Vertex*> pqueue;

    // Set all to INF and add to pqueue
    for (Vertex* v : graph.getNodeSet()) {
        v->cost = INT_MAX;
        pqueue.enqueue(v, v->cost);
    }

    // Set start cost to 0
    start->cost = 0.0;

    // Add start to pqueue with updated cost
    pqueue.changePriority(start, start->heuristic(end));

    while (!pqueue.isEmpty()) {

        // Initialize current node
        Vertex* current = pqueue.dequeue();
        current->visited = true;
        current->setColor(GREEN);

        // If current == end, we know this is the shortest path, since we use a priority queue
        if (current == end) {

            // If we get in here, we have found the lowest-cost path from start to end
            // Backtrack through the path and push to path variable
            while (current != NULL) {
                path.push_back(current);
                current = current->previous;
            }

            // Reverse path and return it
            reverse(path.begin(), path.end());
            return path;
        }

        // For all neighbours:
        for (Vertex* neighbour : graph.getNeighbors(current)) {

            // Make sure we haven't visited this neighbour before
            if (!neighbour->visited) {

                // Calculate cost to travel to neighbour
                Edge* edge = graph.getEdge(current, neighbour);
                double cost = current->cost + edge->cost;

                // If new cost is lower than old one, change
                if (cost < neighbour->cost) {
                    neighbour->cost = cost;
                    neighbour->previous = current;
                    neighbour->setColor(YELLOW);

                    // Rearrange in the pqueue to account of new cost of neighbour
                    pqueue.changePriority(neighbour, neighbour->cost + neighbour->heuristic(end));
                }

            }
        }
    }


    return path;
}
