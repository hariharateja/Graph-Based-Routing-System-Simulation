//ALT landmark shortest path

#include "Asp.hpp"
#include <queue>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>


static constexpr double INF = std::numeric_limits<double>::infinity();
//
//
static const Graph* asp_graph = nullptr;
static bool asp_built = false;
static int asp_num_landmarks = 32; // number of landmarks
//
//
static std::vector<int> asp_landmarks; // landmark node ids
static std::vector<std::vector<double>> asp_distFromLandmarks; // dist from landmarks

void asp_dijsktra(const Graph& graph, int source, std::vector<double>& dist) {
    int n = graph.getAllNodeIds().size();
    dist.assign(n,INF);
    dist[source] = 0.0;

    using PDI = std::pair<double, int>;
    std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq;
    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [currdist, u] = pq.top();
        pq.pop();

        if (currdist > dist[u]) continue;

        for (auto [v, edgeid, w] : graph.neighborsWithEdge(u)) {
            double newDist = dist[u] + w;
            if (newDist < dist[v]) {
                dist[v] = newDist;
                pq.push({newDist, v});
            }
        }
    }
}


json findAsp(const Graph& graph , const json& query){
    std::string type = query.at("type");
    int id = query.at("id");
    double time_budget_ms = query.at("time_budget_ms");
    double acceptable_error_pct = query.at("acceptable_error_pct");

    const auto& qlist = query.at("queries");

    //no. of landmarks based on the error;
    if(acceptable_error_pct == 15.0){
        asp_num_landmarks = 8;
    }else if(acceptable_error_pct == 10.0){
        asp_num_landmarks = 16;
    }else if(acceptable_error_pct == 5.0){
        asp_num_landmarks = 32;
    }

    //preprocess
    if(!asp_built){
        auto nodes = graph.getAllNodeIds();
        int n = nodes.size();
        int l = std::min(n,asp_num_landmarks);

        std::shuffle(nodes.begin(), nodes.end(),std::mt19937{std::random_device{}()});
        asp_landmarks.assign(nodes.begin(),nodes.begin()+l);
        
        asp_distFromLandmarks.assign(l,std::vector<double>(n,INF));
        for(int i=0; i<l; i++){
            asp_dijsktra(graph,asp_landmarks[i],asp_distFromLandmarks[i]);
        }

        asp_built = true;
    }

    json out;

    out["id"] = id;
    out["distances"] = json::array();

    auto start = std::chrono::high_resolution_clock::now();

    int l = asp_landmarks.size();
    double alpha; //factor btw lowerbound and upperbound
    if(acceptable_error_pct == 15.0){
        alpha = 0.3;
    }else if(acceptable_error_pct == 10.0){
       alpha = 0.4;
    }else if(acceptable_error_pct == 5.0){
        alpha = 0.5;
    }
    //loop over every landmark
    for(auto &q : qlist){
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double,std::milli>(now-start).count();

        int s = q.at("source");
        int t = q.at("target");

        if(elapsed > time_budget_ms){
            break;
        }

        double LB = 0.0;
        double UB = INF;
        for(int i=0; i<l; i++){
            double ds = asp_distFromLandmarks[i][s];
            double dt = asp_distFromLandmarks[i][t];

            if(dt < INF && ds < INF){
                double diff = std::fabs(ds-dt);
                if(diff > LB) LB = diff;
                double sum = ds+dt;
                if(sum < UB ) UB = sum;
            }
        }

        double approx;

        if(UB < INF){
            approx = alpha*LB + (1-alpha)*UB;
        }else if(LB >0.0){
            approx = LB;
        }else{
            approx = -1.0;
        }   


        json item;
        item["source"] = s;
        item["target"] = t;
        item["approx_shortest_distance"] = approx;
        out["distances"].push_back(item);
    }

    return out;
}