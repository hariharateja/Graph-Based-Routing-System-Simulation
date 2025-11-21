#include "delivery_scheduler.hpp"
#include "../Phase-1/shortest_path.hpp"
#include <iostream>
#include <set>
#include <limits>
#include <algorithm>
#include <unordered_map>

std::map<int, std::map<int, double>> compute_distance_matrix(const Graph& graph, const std::vector<int>& interest_points){
    std::map<int, std::map<int, double>> matrix;
    for(int src : interest_points){
        std::map<int, double> dist;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

        for (int target : interest_points){ 
            dist[target] = std::numeric_limits<double>::max();
        }
        dist[src] = 0.0;
        pq.push({0.0, src});

        std::set<int> targets_found;
        while (!pq.empty()){
            double d = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            if (d > dist[u]) continue;

            // Check if this node is one of our targets
            for (int target : interest_points) {
                if (u == target) targets_found.insert(target);
            }
            // Stop if we found paths to all interest points
            if (targets_found.size() == interest_points.size()) break;

            for (int edgeId : graph.getNeighborEdges(u)){
                const Edge& e = graph.getEdge(edgeId);
                // assuming we have average time 
                double new_dist = d + e.average_time;
                int v = (e.u == u) ? e.v : e.u;

                if (dist.find(v) == dist.end()) dist[v] = std::numeric_limits<double>::max();

                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    pq.push({new_dist, v});
                }
            }
        }
        for (int target : interest_points) {
            if (dist.find(target) != dist.end()) {
                matrix[src][target] = dist[target];
            } else {
                matrix[src][target] = std::numeric_limits<double>::max(); // Unreachable
            }
        } 
    }
    return matrix;
}
std::tuple<int, int, bool> find_min_max_move(
    const std::vector<DriverState>& drivers,
    const std::vector<OrderStatus>& orders,
    const std::map<int, std::map<int, double>>& dist_matrix){

    int best_driver_idx = -1;
    int best_order_idx = -1;
    bool best_is_pickup = false;
    
    //we want the assignment such that , max time should minimise 
    double min_overall_max_time = std::numeric_limits<double>::max();
    
    // finding max time at current situation 
    double current_max_driver_time = 0.0;

    for(const auto& dr : drivers){
        if (dr.current_time > current_max_driver_time){
            current_max_driver_time = dr.current_time;
        }
    }

    //iterating through every possible rider , task 
    for (size_t d_idx = 0; d_idx < drivers.size(); ++d_idx){
        const DriverState& driver = drivers[d_idx];
        int current_node = driver.current_node;
        for (size_t o_idx = 0; o_idx < orders.size(); ++o_idx){
            const OrderStatus& order = orders[o_idx];
            double travel_cost = std::numeric_limits<double>::max();
            bool is_pickup = false;

            // if pick up valid ??
            if (!order.is_picked_up && dist_matrix.at(current_node).count(order.pickup)) {
                travel_cost = dist_matrix.at(current_node).at(order.pickup);
                is_pickup = true;
            }

            // is drop-off valid ??
            else if (!order.is_delivered){
                if (std::find(driver.carried_orders.begin(), driver.carried_orders.end(), order.id) != driver.carried_orders.end()) {
                    if (dist_matrix.at(current_node).count(order.dropoff)) {
                        travel_cost = dist_matrix.at(current_node).at(order.dropoff);
                        is_pickup = false;
                    }
                }
            }

            if (travel_cost != std::numeric_limits<double>::max()){
                // logic 
                double potential_finish_time = driver.current_time + travel_cost;

                double new_max_time = 0.0;
                if (potential_finish_time > current_max_driver_time){
                    new_max_time = potential_finish_time;
                }
                else{
                    new_max_time = current_max_driver_time;
                }

                if (new_max_time < min_overall_max_time){
                    min_overall_max_time = new_max_time;
                    best_driver_idx = d_idx;
                    best_order_idx = o_idx;
                    best_is_pickup = is_pickup;
                }
            }
        }
    }
    return {best_driver_idx, best_order_idx, best_is_pickup};
}
json solve_delivery_scheduling(const Graph& graph, const json& query){
    int depot = query["fleet"]["depot_node"];
    int num_drivers = query["fleet"]["num_delievery_guys"];

    std::vector<OrderStatus> orders;
    std::vector<int> interest_points;
    interest_points.push_back(depot);

    for (const auto& o : query["orders"]){
        OrderStatus ord;
        ord.id = o["order_id"];
        ord.pickup = o["pickup"];
        ord.dropoff = o["dropoff"];
        orders.push_back(ord);
        
        interest_points.push_back(ord.pickup);
        interest_points.push_back(ord.dropoff);
    }
    // removing duplicate nodes
    std::sort(interest_points.begin(), interest_points.end());
    interest_points.erase(std::unique(interest_points.begin(), interest_points.end()), interest_points.end());

    auto dist_matrix = compute_distance_matrix(graph, interest_points);

    std::vector<DriverState> drivers(num_drivers);

    for (int i = 0; i < num_drivers; ++i){
        drivers[i].id = i;
        drivers[i].current_node = depot;
        drivers[i].current_time = 0.0;
        drivers[i].route_path.push_back(depot);
    }

    int orders_delivered = 0;
    int total_orders = orders.size();

    // greedy 

    while (orders_delivered < total_orders){

        // picking driver who is free at the earliest
        int best_driver_idx = -1;
        double min_driver_time = std::numeric_limits<double>::max();

        for (int i = 0; i < num_drivers; ++i) {
            if (drivers[i].current_time < min_driver_time) {
                min_driver_time = drivers[i].current_time;
                best_driver_idx = i;
            }
        }

        DriverState& driver = drivers[best_driver_idx];
        int current_node = driver.current_node;

        // finding the valid move for the driver 
        // Pickup an order that hasn't been picked up by ANYONE yet.
        // Dropoff an order that THIS driver is currently carrying.

        int best_order_idx = -1;
        bool is_pickup_move = false;
        double min_travel_cost = std::numeric_limits<double>::max();

        // checking all orders 
        for (size_t i = 0; i < orders.size(); ++i){
            // pick up 
            if (!orders[i].is_picked_up){
                double time_to_pickup = dist_matrix[current_node][orders[i].pickup];
                if (time_to_pickup < min_travel_cost){
                    min_travel_cost = time_to_pickup;
                    best_order_idx = i;
                    is_pickup_move = true;
                }
            }

            // drop off 
            else if (!orders[i].is_delivered){
                // Check if this driver is the one carrying it
                bool carrying = false;
                for (int carried_id : driver.carried_orders) {
                    if (carried_id == orders[i].id) { carrying = true; break; }
                }

                if (carrying){
                    double time_to_drop = dist_matrix[current_node][orders[i].dropoff];
                    if (time_to_drop < min_travel_cost){
                        min_travel_cost = time_to_drop;
                        best_order_idx = i;
                        is_pickup_move = false;
                    }
                }
            }
        }

        // perform the move 

        if (best_order_idx != -1){
            OrderStatus& order = orders[best_order_idx];
            int target_node = is_pickup_move ? order.pickup : order.dropoff;
            double travel_time = min_travel_cost;

            // we should update the driver 
            json path_query = {
                {"id", -1}, // Dummy ID
                {"source", driver.current_node},
                {"target", target_node},
                {"mode", "time"}, 
                {"constraints", {}} // No constraints
            };

            ShortestPathResult path_res = findShortestPath(graph, path_query);
            
            // We verify the path exists and is not empty
            if (path_res.possible && !path_res.path.empty()){
                // We start from index 1 because index 0 is the current_node, 
                // which is already in the route_path from the previous step.
                driver.route_path.insert(driver.route_path.end(), path_res.path.begin() + 1, path_res.path.end());
            }
            else{
                driver.route_path.push_back(target_node); //fallback 
            }

            driver.current_time += travel_time;
            driver.current_node = target_node;

            if (is_pickup_move){
                order.is_picked_up = true;
                driver.carried_orders.push_back(order.id);
            }
            else{
                order.is_delivered = true;
                order.completion_time = driver.current_time;
                //removing rider bag 
                auto& bag = driver.carried_orders;
                bag.erase(std::remove(bag.begin(), bag.end(), order.id), bag.end());

                orders_delivered++;

                driver.delivered_orders_history.push_back(order.id);
            }

        }
        else{
            // if it is not connected or infinite loop
            driver.current_time += 1.0;
        }
    }

    // format output 

    json output;
    output["assignments"] = json::array();
    double total_delivery_time_s = 0.0;

    for (const auto& dr : drivers){
        json assignment;
        assignment["driver_id"] = dr.id;
        assignment["route"] = dr.route_path;
    }
    for (const auto& o : orders) {
        total_delivery_time_s += o.completion_time;
    }

    output["metrics"]["total_delivery_time_s"] = total_delivery_time_s;

    return output;
}