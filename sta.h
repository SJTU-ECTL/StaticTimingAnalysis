#ifndef BOOLEANNETWORK_STA_H
#define BOOLEANNETWORK_STA_H

#include <iostream>
#include <queue>
#include <deque>
#include "bnet.h"

std::vector<BnetNodeID> TopologicalSort(const BnetNetwork *net);

std::map<BnetNodeID, int> CalculateSlack(const BnetNetwork *net);

void KMostCriticalPaths(const BnetNetwork *net, int k = 10, bool show_slack = false);

#endif
