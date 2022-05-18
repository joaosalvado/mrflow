#include "mrflow/planner/MrFlowPlanner.h"

bool mrflow::planner::MrFlowPlanner::iterativeSolver(
    std::vector<int> &start,
    std::vector<int> &goal,
    int Tmin, int Tmax)
{
    for (int T = Tmin; T <= Tmax; ++T)
    {
        bool solution_found = mrnet_->query(start, goal, T);
        if (solution_found)
            return true;
    }
    return false;
}

bool mrflow::planner::MrFlowPlanner::solve(
    std::vector<int> &start,
    std::vector<int> &goal,
    std::vector<std::vector<int>> &solution)
{
    if (start == goal)
        return true; //Case start equal to goal

    int T_max = mrnet_->getMaxT();
    int T_min = 0; // TODO: compute better estimate (too conservative)
    bool solved = iterativeSolver(start, goal, T_min, T_max);

    std::vector<std::vector<int>> flow_solution;
    //flow_solution = this->fr_->getAbstractPath(this->mrnet_);
    this->fr_->toSingleMove(this->mrnet_, flow_solution);
    fr_->printAbstractPath(flow_solution); //TODO: comment me

    solution = flow_solution;
    return solved;
}

std::vector<std::vector<int>>
mrflow::planner::MrFlowPlanner::dummyLabelledPath(
        std::vector<std::vector<int>> single_move_unlabeled_path){
    auto labeled_path = std::vector<std::vector<int>>();

    int path_length = single_move_unlabeled_path.size();
     auto convertLabeled = [this](std::vector<std::set<int>> labeled_set){
         auto labeled = std::vector<int>(R);
         for(int p_id = 0; p_id < P; ++p_id){
             for(auto robot_id : labeled_set[p_id]){
                 labeled[robot_id] = p_id;
             }
         }
         return labeled;
     };

    // Label start config
    auto start_unlabeled = single_move_unlabeled_path[0];
    auto start_label = std::vector<std::set<int>>(P, std::set<int>());
    int robot_id = 0;
    for(int p_id = 0; p_id < P; ++p_id){
        for( int numRobots = 0; numRobots < start_unlabeled[p_id]; ++numRobots){
            start_label[p_id].insert(robot_id++);
        }
    }

    // Label entire path
    labeled_path.push_back(convertLabeled(start_label));
    auto curr_label = start_label;
    for(int path_seq_id = 0; path_seq_id < path_length; ++path_seq_id){

    }

    return labeled_path;
}

void mrflow::planner::MrFlowPlanner::setup()
{
    //code
    auto env_connectivity = cfree->getConnectivity();
    auto polygons_capacity = cfree->polygon_info_.maxNumRobots;
    auto poly_transition_cost_matrix = cfree->transition_cost_matrix_;

    this->mrnet_ =
        std::make_shared<mrflow::base::MultirobotNetwork>(
            env_connectivity,
            polygons_capacity,
            poly_transition_cost_matrix,
            this->cost_stay_,
            this->R);
    mrnet_->create_dynamic_network();

    std::cout << "[Mrflow] Network generated\n";
}

