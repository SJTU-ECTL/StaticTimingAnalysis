#include <iostream>
#include <random>
#include <queue>
#include "bnet.h"
#include "sta.h"

int main() {
    BnetNetwork *net;
    std::string blif_file_path(PROJECT_SOURCE_DIR);
    blif_file_path += "/benchmark/C1908.blif";
    net = new BnetNetwork(blif_file_path, true);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 30);

    std::map<BnetNodeID, double> error;
    for (auto node:net->getNodesList()) {
        if (node->getName() == SOURCE_NAME || node->getName() == SINK_NAME)
            error.insert(std::pair<BnetNodeID, double>
                                 (node->getName(), 1));
        else
            error.insert(std::pair<BnetNodeID, double>
                                 (node->getName(), (double) dis(gen) / 100));
    }

    KMostCriticalPaths(net, 10);
    std::vector<BnetNodeID> min_cut = MinCut(net, error);
    std::cout << "Min Cut Nodes: ";
    for (auto &node_name:min_cut) {
        std::cout << node_name << " ";
    }

    delete (net);
    return 0;
}
