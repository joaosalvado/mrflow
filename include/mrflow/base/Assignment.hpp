#ifndef ASSIGNMENT
#define ASSIGNMENT

#include <lemon/matching.h>
#include <lemon/list_graph.h>
#include <vector>

namespace mrflow::base
{
    class Assignment
    {
    public:
        Assignment()
        : weigthed_edges_(bg_),
          blue_nodes_(bg_),
          red_nodes_(bg_){};

        virtual ~Assignment(){};

        void setProblem( const std::vector<std::vector<int>> &weight );
        void solve();
    private:
        lemon::ListBpGraph::EdgeMap<int> weigthed_edges_;
        lemon::ListBpGraph::NodeMap<int> blue_nodes_;
        lemon::ListBpGraph::NodeMap<int> red_nodes_;
        lemon::ListBpGraph bg_; //bipartite graph 
    };
} // namespace mrflow

#endif