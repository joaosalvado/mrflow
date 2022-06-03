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
    //fr_->printAbstractPath(flow_solution); //TODO: comment me

    solution = flow_solution;

    return solved;
}

void mrflow::planner::MrFlowPlanner::findTransition(
        const std::vector<int> &x_prev,
        const std::vector<int> &x_next,
        int &P_in, int &P_out) const{
    std::vector<int> polygon_transitions(P); // +1 in polygon, -1 out of poly, 0 nothing happens

    //X_tilde describes the number of robots per polygon [1 0 2] 1 robot in P1, 0 in P2 and 2 in P3
    for(int p_id = 0; p_id < P; ++p_id){
        polygon_transitions[p_id] = x_next[p_id] - x_prev[p_id];
    }
    int p_id = 0;
    std::for_each(polygon_transitions.begin(),
                  polygon_transitions.end(),
                  [&P_in, &P_out, &p_id](int diff){
                      if(diff ==  1) P_in = p_id;
                      if(diff == -1) P_out = p_id;
                      ++p_id;
                  }
    );
}


std::vector<std::vector<int>>
mrflow::planner::MrFlowPlanner::dummyLabelledPath(
        std::vector<std::vector<int>> single_move_unlabeled_path){
    auto labeled_path = std::vector<std::vector<int>>();

    int path_length = single_move_unlabeled_path.size();
     auto convertLabeled = [this](std::vector<std::deque<int>> labeled_set){
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
    auto start_label = std::vector<std::deque<int>>(P, std::deque<int>());
    int robot_id = 0;
    for(int p_id = 0; p_id < P; ++p_id){
        for( int numRobots = 0; numRobots < start_unlabeled[p_id]; ++numRobots){
            start_label[p_id].push_back(robot_id++);
        }
    }

    // Label entire path
    labeled_path.push_back(convertLabeled(start_label));
    auto curr_label = start_label;
    for(int path_seq_id = 1; path_seq_id < path_length; ++path_seq_id){
        auto new_label = curr_label; // copy
        // E.g. [ 0 1 2] 1 robot in poly 1 , 2 robots in poly 2
       auto prev_unlabeld = single_move_unlabeled_path[path_seq_id-1];
       // E.g. [ 1 1 1] 1 robot in poly 0 , 1 robot in poly 1, 1 robot in poly 2
       auto next_unlabled = single_move_unlabeled_path[path_seq_id];
       // Compute polygon robot is coming out and going in, e.g. pin=0, pout=2
       int pin, pout;
       findTransition(prev_unlabeld, next_unlabled, pin, pout);
       // Remove a robot from pout and added it to pin
       int robot_transitioning = new_label[pout].front();
       new_label[pout].pop_front();
       new_label[pin].push_back(robot_transitioning);
       // Store new config
       labeled_path.push_back((convertLabeled(new_label)));

       // Prepare next iter
       curr_label = new_label;
    }

    // Remove duplicated polygon
    auto labeled_path_nodups = std::vector<std::vector<int>>(R, std::vector<int>());
    // Set start label
    for(int r = 0; r < R; ++r){
        labeled_path_nodups[r].push_back(labeled_path[0][r]);
    }
    for(int path_seq_id = 1; path_seq_id < path_length; ++path_seq_id){
        for(int r = 0; r < R; ++r){
            if(labeled_path_nodups[r].back() != labeled_path[path_seq_id][r]){
                labeled_path_nodups[r].push_back(labeled_path[path_seq_id][r]);
            }
        }
    }
    return labeled_path_nodups;
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


bool mrflow::planner::MrFlowPlanner::solve_concatpath(
        std::vector<int> &start,
        std::vector<int> &goal,
        std::vector<std::vector<int>> &solution){
    this->solve(start, goal, solution);
    solution = this->dummyLabelledPath(solution);
    return true;
}
