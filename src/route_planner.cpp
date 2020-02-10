#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    start_node = &m_Model.FindClosestNode(start_x, start_y) ;
    end_node = &m_Model.FindClosestNode(end_x, end_y) ;
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calculate the manhattan distance
    return node->distance(*end_node);

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors() ;
    for (int i = 0; i < current_node->neighbors.size(); i++) {
        RouteModel::Node* neighbor = current_node->neighbors[i] ;
        // set parent of neighbor node to current node
        neighbor->parent = current_node;
        // calculate g and h values
        neighbor->h_value = this->CalculateHValue(neighbor) ;
        neighbor->g_value = current_node->g_value+current_node->distance(*neighbor);
        //add to open list
        open_list.emplace_back(neighbor);
        //set to visited
        neighbor->visited = true;
    }

}

RouteModel::Node *RoutePlanner::NextNode() {
    //sort open list
    std::sort(open_list.begin(), open_list.end(),
              [](const RouteModel::Node *a, const RouteModel::Node *b) {
                  return a->h_value + a->g_value >
                         b->h_value + b->g_value;
              });
    //get node with lowest sum
    RouteModel::Node* lowest = open_list.back() ;
    //remove this element from open_list
    open_list.pop_back() ;

    return lowest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* n = current_node;

    while(n != start_node) {
        //distance += n->g_value ;
        path_found.emplace(path_found.begin(), *n);
        distance += n->distance(*(n->parent));
        n = n->parent ;
    }

    //include start node
    path_found.emplace(path_found.begin(), *start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    //add start node to open_list
    open_list.emplace_back(start_node);
    //calculate heuristic for start_node
    start_node->h_value = this->CalculateHValue(start_node);
    start_node->g_value = 0;
    //set visted to true
    start_node->visited = true;

    while (!open_list.empty()){
        current_node = this->NextNode();

        if(current_node->distance(*end_node) == 0){
            m_Model.path = this->ConstructFinalPath(current_node);
            return;
        }

        this->AddNeighbors(current_node);
    }

    // We've run out of new nodes to explore and haven't found a path.
    std::cout << "No path found!" << "\n";
    return ;

}