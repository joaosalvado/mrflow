#include <iostream>
#include <map>

#include <lemon/matching.h>
#include <lemon/list_graph.h>

using namespace lemon;
using namespace std;

int main()
{

    lemon::ListBpGraph g;
    std::vector<std::vector<int>> w =
        {
            {0, -1},
            {-1, 0}

        };

    auto b_n1 = g.addBlueNode();
    auto b_n2 = g.addBlueNode();

    auto r_n1 = g.addRedNode();
    auto r_n2 = g.addRedNode();

    auto edge11 = g.addEdge(b_n1, r_n1);
    auto edge12 = g.addEdge(b_n1, r_n2);
    auto edge21 = g.addEdge(b_n2, r_n1);
    auto edge22 = g.addEdge(b_n2, r_n2);

    lemon::ListBpGraph::EdgeMap<int> weight(g);
    weight[edge11] = w[0][0];
    weight[edge12] = w[1][0];
    weight[edge21] = w[0][1];
    weight[edge22] = w[1][1];

    MaxWeightedMatching<lemon::ListBpGraph> mwm(g, weight);

    mwm.run();
    std::cout << mwm.matching(edge11) << std::endl;
    std::cout << mwm.matching(edge12) << std::endl;
    std::cout << mwm.matching(edge21) << std::endl;
    std::cout << mwm.matching(edge22) << std::endl;
}
