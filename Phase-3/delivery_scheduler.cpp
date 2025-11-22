#include "delivery_scheduler.hpp"
#include "../Phase-1/shortest_path.hpp"
#include <iostream>
#include <set>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <chrono> // Added for processing time

// Helper to compute all-pairs shortest paths between interest points
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

            for (int target : interest_points) {
                if (u == target) targets_found.insert(target);
            }
            if (targets_found.size() == interest_points.size()) break;

            for (int edgeId : graph.getNeighborEdges(u)){
                const Edge& e = graph.getEdge(edgeId);
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
                matrix[src][target] = std::numeric_limits<double>::max();
            }
        } 
    }
    return matrix;
}

// Helper strategy: Find move that minimizes the Global Max Time (Min-Max Strategy)
std::tuple<int, int, bool> find_min_max_move(
    const std::vector<DriverState>& drivers,
    const std::vector<OrderStatus>& orders,
    const std::map<int, std::map<int, double>>& dist_matrix){

    int best_driver_idx = -1;
    int best_order_idx = -1;
    bool best_is_pickup = false;
    
    double min_overall_max_time = std::numeric_limits<double>::max();
    
    // Find current max time among all drivers to compare against
    double current_max_driver_time = 0.0;
    for(const auto& dr : drivers){
        if (dr.current_time > current_max_driver_time){
            current_max_driver_time = dr.current_time;
        }
    }

    // Iterate through every possible driver-order combination
    for (size_t d_idx = 0; d_idx < drivers.size(); ++d_idx){
        const DriverState& driver = drivers[d_idx];
        int current_node = driver.current_node;
        
        for (size_t o_idx = 0; o_idx < orders.size(); ++o_idx){
            const OrderStatus& order = orders[o_idx];
            double travel_cost = std::numeric_limits<double>::max();
            bool is_pickup = false;

            // Check Validity: Pickup
            if (!order.is_picked_up && dist_matrix.at(current_node).count(order.pickup)) {
                travel_cost = dist_matrix.at(current_node).at(order.pickup);
                is_pickup = true;
            }
            // Check Validity: Dropoff
            else if (!order.is_delivered){
                // Only valid if this driver is carrying the order
                bool carrying = false;
                for(int id : driver.carried_orders) { if(id == order.id) carrying = true; }
                
                if (carrying && dist_matrix.at(current_node).count(order.dropoff)) {
                    travel_cost = dist_matrix.at(current_node).at(order.dropoff);
                    is_pickup = false;
                }
            }

            // Evaluate Impact on Max Time
            if (travel_cost != std::numeric_limits<double>::max()){
                double potential_finish_time = driver.current_time + travel_cost;

                double new_max_time = std::max(potential_finish_time, current_max_driver_time);

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

json solve_delivery_scheduling(const Graph& graph, const json& query, bool min_max){
    // Start Timer
    auto start_time = std::chrono::high_resolution_clock::now();

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

    while (orders_delivered < total_orders){

        int best_driver_idx = -1;
        int best_order_idx = -1;
        bool is_pickup_move = false;
        double move_travel_cost = std::numeric_limits<double>::max();

        if (min_max) {
            // STRATEGY 1: Minimize MAX Time (using helper function)
            std::tie(best_driver_idx, best_order_idx, is_pickup_move) = find_min_max_move(drivers, orders, dist_matrix);
            
            if (best_driver_idx != -1) {
                int u = drivers[best_driver_idx].current_node;
                int v = is_pickup_move ? orders[best_order_idx].pickup : orders[best_order_idx].dropoff;
                move_travel_cost = dist_matrix.at(u).at(v);
            }

        } else {
            // STRATEGY 2: Minimize TOTAL Time (Greedy Local)
            // 1. Pick the driver who is free earliest
            double min_driver_time = std::numeric_limits<double>::max();
            for (int i = 0; i < num_drivers; ++i) {
                if (drivers[i].current_time < min_driver_time) {
                    min_driver_time = drivers[i].current_time;
                    best_driver_idx = i;
                }
            }

            // 2. Find best move for THIS driver
            if (best_driver_idx != -1) {
                DriverState& driver = drivers[best_driver_idx];
                int current_node = driver.current_node;
                
                for (size_t i = 0; i < orders.size(); ++i){
                    if (!orders[i].is_picked_up){
                        double t = dist_matrix[current_node][orders[i].pickup];
                        if (t < move_travel_cost){
                            move_travel_cost = t;
                            best_order_idx = i;
                            is_pickup_move = true;
                        }
                    }
                    else if (!orders[i].is_delivered){
                        // Check if THIS driver is carrying
                        bool carrying = false;
                        for (int cid : driver.carried_orders) if (cid == orders[i].id) carrying = true;
                        
                        if (carrying){
                            double t = dist_matrix[current_node][orders[i].dropoff];
                            if (t < move_travel_cost){
                                move_travel_cost = t;
                                best_order_idx = i;
                                is_pickup_move = false;
                            }
                        }
                    }
                }
            }
        }

        // PERFORM THE MOVE
        if (best_driver_idx != -1 && best_order_idx != -1){
            DriverState& driver = drivers[best_driver_idx];
            OrderStatus& order = orders[best_order_idx];
            int target_node = is_pickup_move ? order.pickup : order.dropoff;

            // Generate detailed path for output
            json path_query = {
                {"id", -1},
                {"source", driver.current_node},
                {"target", target_node},
                {"mode", "time"}, 
                {"constraints", {}} 
            };

            ShortestPathResult path_res = findShortestPath(graph, path_query);
            
            if (path_res.possible && !path_res.path.empty()){
                driver.route_path.insert(driver.route_path.end(), path_res.path.begin() + 1, path_res.path.end());
            } else {
                driver.route_path.push_back(target_node);
            }

            driver.current_time += move_travel_cost;
            driver.current_node = target_node;

            if (is_pickup_move){
                order.is_picked_up = true;
                driver.carried_orders.push_back(order.id);
            } else {
                order.is_delivered = true;
                order.completion_time = driver.current_time;
                
                auto& bag = driver.carried_orders;
                bag.erase(std::remove(bag.begin(), bag.end(), order.id), bag.end());

                orders_delivered++;
                driver.delivered_orders_history.push_back(order.id);
            }
        } else {
            // Deadlock or waiting state: advance all idle drivers slightly
            bool active = false;
            for(auto& dr : drivers) {
                if(!dr.carried_orders.empty()) active = true; // Assuming someone is moving
                else dr.current_time += 1.0;
            }
            if(!active && orders_delivered < total_orders) break; // Safety break
        }
    }

    // Stop Timer
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // FORMAT OUTPUT
    json output;
    output["assignments"] = json::array(); // Fix: Initialize array

    double total_delivery_time_s = 0.0;
    double max_delivery_time_s = 0.0;

    for (const auto& dr : drivers){
        json assignment;
        assignment["driver_id"] = dr.id;
        assignment["route"] = dr.route_path;
        assignment["order_ids"] = dr.delivered_orders_history; // Added order IDs
        output["assignments"].push_back(assignment); // Fix: Push to array
    }

    for (const auto& o : orders) {
        total_delivery_time_s += o.completion_time;
        if (o.completion_time > max_delivery_time_s) {
            max_delivery_time_s = o.completion_time;
        }
    }

    output["metrics"]["total_delivery_time_s"] = total_delivery_time_s;
    output["metrics"]["max_delivery_time_s"] = max_delivery_time_s;
    output["metrics"]["processing_time_ms"] = duration.count();
    output["metrics"]["strategy_used"] = min_max ? "Minimize Max Time" : "Minimize Total Time";

    return output;
}