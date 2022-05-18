#include "base/Assignment.hpp"


void mrflow::base::Assignment::setProblem(
    const std::vector<std::vector<int>> &weight)
{
    if (weight.size() == 0 || weight[0].size() == 0)
    {
        std::cerr << "[Assignment] Problem is not set correclty" << std::endl;
    }

    auto R_pout = weight.size();
    auto P_pin = weight[0].size();

    //Create blue nodes  (LEFT) - robots in Pout
    for (int r = 0; r < R_pout; ++r)
    {
        auto &&r_pout_node = bg_.addBlueNode();
        blue_nodes_[r_pout_node] = r;
    }

    //Create red nodes (RIGHT) - poligons Pin
    for (int p = 0; p < P_pin; ++p)
    {
        auto &&p_pin_node = bg_.addRedNode();
        red_nodes_[p_pin_node] = p;
    }

    //Create the edges between blue -> red nodes
    //And assign weight
    for (auto &r_pout_node : bg_.blueNodes())
    {
        for (auto &p_pin_node : bg_.redNodes())
        {
            auto &&edge_r_p = bg_.addEdge(r_pout_node, p_pin_node);
            weigthed_edges_[edge_r_p] = weight[blue_nodes_[r_pout_node]]
                                              [red_nodes_[p_pin_node]];
        }
    }
}

void mrflow::base::Assignment::solve()
{
    lemon::MaxWeightedMatching<lemon::ListBpGraph> 
        mwm(bg_, weigthed_edges_);

    mwm.run();
}