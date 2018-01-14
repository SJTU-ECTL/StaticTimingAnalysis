#include "sta.h"

static const int INF = 1000000;

struct T_Node {
    BnetNodeID name;
    int max_delay_to_sink;

    T_Node(BnetNodeID name, int max_delay_to_sink) {
        this->name = std::move(name);
        this->max_delay_to_sink = max_delay_to_sink;
    }
};

struct PartialPath {
    std::vector<BnetNodeID> path;
    int max_delay;

    PartialPath(BnetNodeID node, int max_delay,
                std::vector<BnetNodeID> path = std::vector<BnetNodeID>()) {
        this->path = std::move(path);
        this->path.emplace_back(node);
        this->max_delay = max_delay;
    }
};

struct Edge {
    int u, v;
    double cap, flow;

    Edge() = default;

    Edge(int u, int v, double cap) : u(u), v(v), cap(cap), flow(0) {}
};

struct Dinic {
    int N;
    std::vector<Edge> E;
    std::vector<std::vector<int>> g;
    std::vector<int> d, pt;
    std::vector<bool> res_visited;

    explicit Dinic(int N) : N(N), E(0), g(N), d(N), pt(N), res_visited(N, false) {}

    void AddEdge(int u, int v, double cap) {
        if (u != v) {
            E.emplace_back(Edge(u, v, cap));
            g[u].emplace_back(E.size() - 1);
            E.emplace_back(Edge(v, u, 0));
            g[v].emplace_back(E.size() - 1);
        }
    }

    bool BFS(int S, int T) {
        std::queue<int> q({S});
        fill(d.begin(), d.end(), N + 1);
        d[S] = 0;
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            if (u == T) break;
            for (int k: g[u]) {
                Edge &e = E[k];
                if (e.flow < e.cap && d[e.v] > d[e.u] + 1) {
                    d[e.v] = d[e.u] + 1;
                    q.emplace(e.v);
                }
            }
        }
        return d[T] != N + 1;
    }

    double DFS(int u, int T, double flow = -1) {
        if (u == T || flow == 0) return flow;
        for (int &i = pt[u]; i < g[u].size(); ++i) {
            Edge &e = E[g[u][i]];
            Edge &oe = E[g[u][i] ^ 1];
            if (d[e.v] == d[e.u] + 1) {
                double amt = e.cap - e.flow;
                if (flow != -1 && amt > flow) amt = flow;
                if (double pushed = DFS(e.v, T, amt)) {
                    e.flow += pushed;
                    oe.flow -= pushed;
                    return pushed;
                }
            }
        }
        return 0;
    }

    double MaxFlow(int S, int T) {
        double total = 0;
        while (BFS(S, T)) {
            fill(pt.begin(), pt.end(), 0);
            while (double flow = DFS(S, T))
                total += flow;
        }
        return total;
    }

    void DFS_ResidualNetwork(int u) {
        res_visited[u] = true;
        for (auto i:g[u]) {
            Edge &e = E[i];
            if (!res_visited[e.v]) {
                if (e.cap > 0 && e.flow > 0 && e.cap - e.flow > 0)
                    DFS_ResidualNetwork(e.v);
                else if (e.cap == 0 && e.flow < 0)
                    DFS_ResidualNetwork(e.v);
            }
        }
    }

    std::vector<Edge> MinCut(int S, int T) {
        double max_flow = MaxFlow(S, T);
        std::vector<Edge> min_cut;
        DFS_ResidualNetwork(S);
        for (auto e:E)
            if (e.cap > 0 && e.flow > 0 && res_visited[e.u] && !res_visited[e.v])
                min_cut.push_back(e);
        return min_cut;
    }
};

std::vector<BnetNodeID> TopologicalSort(const BnetNetwork *net) {
    std::vector<BnetNodeID> sortedNodes;
    std::deque<BnetNodeID> nodesQueue;
    std::map<BnetNodeID, int> inDegrees;

    for (auto node:net->getNodesList()) {
        inDegrees[node->getName()] = (int) node->getFanIns().size();
        if (node->getFanIns().empty())
            nodesQueue.emplace_back(node->getName());
    }

    while (!nodesQueue.empty()) {
        BnetNodeID id = nodesQueue.front();
        sortedNodes.emplace_back(id);
        nodesQueue.pop_front();
        const BnetNode *node = net->getNodebyName(id);
        for (const auto &output:node->getFanOuts()) {
            inDegrees[output] -= 1;
            if (inDegrees[output] == 0)
                nodesQueue.push_back(output);
        }
    }

    return sortedNodes;
}

std::map<BnetNodeID, int> CalculateSlack(const BnetNetwork *net) {
    std::vector<BnetNodeID> sorted_nodes_id = TopologicalSort(net);
    std::map<BnetNodeID, int> arrival_time;
    std::map<BnetNodeID, int> required_time;
    std::map<BnetNodeID, int> slack;
    /* Initialization */
    for (auto it = sorted_nodes_id.begin(); it != sorted_nodes_id.end(); ++it) {
        arrival_time.insert(std::pair<BnetNodeID, int>(*it, -1 * INF));
        required_time.insert(std::pair<BnetNodeID, int>(*it, INF));
        slack.insert(std::pair<BnetNodeID, int>(*it, 0));
    }
    int max_at = -1 * INF;
    /* Update at */
    arrival_time.at(SOURCE_NAME) = 0;
    for (auto it = sorted_nodes_id.begin(); it != sorted_nodes_id.end(); ++it) {
        for (const auto &input:net->getNodebyName(*it)->getFanIns())
            arrival_time.at(*it) = std::max(arrival_time.at(*it), arrival_time.at(input) + 1);
        if (arrival_time.at(*it) > max_at)
            max_at = arrival_time.at(*it);
    }
    /* Update rat */
    required_time.at(SINK_NAME) = max_at;
    for (auto rit = sorted_nodes_id.rbegin(); rit != sorted_nodes_id.rend(); ++rit) {
        for (const auto &output:net->getNodebyName(*rit)->getFanOuts())
            required_time.at(*rit) = std::min(
                    required_time.at(*rit), required_time.at(output) - 1);
    }
    /* Update slack */
    for (auto node:net->getNodesList())
        slack.at(node->getName()) = required_time.at(node->getName())
                                    - arrival_time.at(node->getName());

    return slack;
}

void KMostCriticalPaths(const BnetNetwork *net, int k, bool show_slack) {
    std::map<BnetNodeID, int> slack = CalculateSlack(net);
    std::vector<BnetNodeID> sorted_nodes_id = TopologicalSort(net);
    std::map<BnetNodeID, int> max_delay_to_sink;
    /* Computation of Maximum Delays to Sink */
    for (auto it = sorted_nodes_id.begin(); it != sorted_nodes_id.end(); ++it)
        max_delay_to_sink.insert(std::pair<BnetNodeID, int>(*it, 0));

    for (auto rit = sorted_nodes_id.rbegin(); rit != sorted_nodes_id.rend(); ++rit) {
        for (const auto &fan_out_id:net->getNodebyName(*rit)->getFanOuts())
            max_delay_to_sink.at(*rit) = std::max(
                    max_delay_to_sink.at(*rit),
                    max_delay_to_sink.at(fan_out_id) + 1);
    }

    /* Sorting the Successors of Each Vertex */
    auto t_comp = [](T_Node a, T_Node b) {
        return a.max_delay_to_sink > b.max_delay_to_sink;
    };
    for (auto node: net->getNodesList()) {
        std::vector<T_Node> t_fan_outs;
        std::vector<BnetNodeID> fan_outs_id;

        for (const auto &fan_out_id:node->getFanOuts())
            t_fan_outs.emplace_back(T_Node(fan_out_id, max_delay_to_sink.at(fan_out_id)));

        std::sort(t_fan_outs.begin(), t_fan_outs.end(), t_comp);

        for (const auto &t_fan_out:t_fan_outs)
            fan_outs_id.emplace_back(t_fan_out.name);
        node->setFanOuts(fan_outs_id);
    }

    /* Path Enumeration */
    auto comp = [](PartialPath a, PartialPath b) { return a.max_delay < b.max_delay; };
    std::priority_queue<PartialPath, std::vector<PartialPath>, decltype(comp)> paths(comp);
    paths.push(PartialPath(SOURCE_NAME, max_delay_to_sink.at(SOURCE_NAME)));

    while (!paths.empty() && k > 0) {
        PartialPath t_path = paths.top();
        paths.pop();
        if (t_path.path.back() == SINK_NAME) {
            k--;
            std::cout << "Delay: " << t_path.max_delay << "\t";
            for (const auto &node_id : t_path.path) {
                std::cout << node_id;
                if (show_slack)
                    std::cout << "=" << slack.at(node_id);
                std::cout << "\t";
            }
            std::cout << std::endl;
        } else {
            BnetNode *node_t = net->getNodebyName(t_path.path.back());
            for (const auto &successor_id : node_t->getFanOuts()) {
                paths.push(PartialPath(successor_id,
                                       (int) t_path.path.size() + max_delay_to_sink.at(successor_id),
                                       t_path.path));
            }
        }
    }
}

std::vector<BnetNodeID> MinCut(const BnetNetwork *net, std::map<BnetNodeID, double> error) {
    std::map<BnetNodeID, int> slack = CalculateSlack(net);

    std::vector<BnetNodeID> id_to_name;
    std::map<BnetNodeID, int> name_to_id;
    int source = -1, sink = -1;
    auto N = (int) net->getNodesList().size();
    Dinic dinic(N * 2);


    for (auto node:net->getNodesList()) {
        if (slack.at(node->getName()) == 0) {
            id_to_name.push_back(node->getName());
            name_to_id.insert(std::pair<BnetNodeID, int>
                                      (node->getName(), id_to_name.size() - 1));
            if (node->getName() == SOURCE_NAME)
                source = (int) id_to_name.size() - 1;
            if (node->getName() == SINK_NAME)
                sink = (int) id_to_name.size() - 1;
        }
    }

    for (auto node:net->getNodesList()) {
        if (slack.at(node->getName()) == 0) {
            int u = name_to_id.at(node->getName());
            dinic.AddEdge(u, u + N, error.at(node->getName()));
            for (auto &fanout_name:node->getFanOuts()) {
                if (slack.at(fanout_name) == 0) {
                    int v = name_to_id.at(fanout_name);
                    dinic.AddEdge(u + N, v, INF);
                }
            }
        }
    }

    std::vector<Edge> min_cut = dinic.MinCut(source, sink);
    std::vector<BnetNodeID> min_cut_nodes;
    for (auto e:min_cut)
        min_cut_nodes.push_back(id_to_name[e.u]);
    return min_cut_nodes;
}
