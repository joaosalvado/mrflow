#include "base/FlowRobots.hpp"

std::vector<std::vector<int>> mrflow::base::FlowRobots::getAbstractPath(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn)
{
    std::vector<std::vector<int>> path(mn->T + 1, std::vector<int>(mn->sn.P));
    for (int t_k = 0; t_k < mn->T+1; ++t_k)
    {
        for (int p_id = 0; p_id < mn->sn.P; ++p_id)
        {
            const auto &arc_id = mn->arc_ids_configs[t_k][p_id];
            //path[t_k][p_id] = mn->dn.flow->operator[](mn->dn.G->arcFromId(arc_id));
             path[t_k][p_id] = mn->cs->flow( mn->dn.G->arcFromId(arc_id) );
        }
    }
    return path;
}

void mrflow::base::FlowRobots::printAbstractPath(std::vector<std::vector<int>> path)
{
    std::cout << "<Path>" <<std::endl;
    for (auto state : path)
    {
        std::cout << "[ ";
        for (auto nR : state)
        {
            std::cout << " " << nR;
        }
        std::cout << "  ]" << std::endl;
    }
     std::cout << "~<Path>" <<std::endl;
}

std::vector<std::vector<int>> mrflow::base::FlowRobots::getTimeCompressedSolution(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn)
{
    std::vector<std::vector<int>> sol;
    sol.push_back(std::vector<int>(mn->sn.P));
    for (int t = 0; t < mn->T + 1; ++t)
    {
        bool diff = false;
        for (int p_id = 0; p_id < mn->sn.P; ++p_id)
        {
            auto arc_id = mn->arc_ids_configs[t][p_id];
            //auto flow = mn->dn.flow->operator[](mn->dn.G->arcFromId(arc_id));
            auto flow = mn->cs->flow( mn->dn.G->arcFromId(arc_id) );
            if (t == 0)
            {
                sol[0][p_id] = flow;
            }
            else
            {
                //Prev
                auto &prev = sol[sol.size() - 1][p_id];
                auto &curr = flow;
                if (curr != prev)
                {
                    diff = true;
                    break;
                }
            }
        }

        if (diff)
        {
            sol.push_back(std::vector<int>(mn->sn.P));
            for (int p_id = 0; p_id < mn->sn.P; ++p_id)
            {
                auto arc_id = mn->arc_ids_configs[t][p_id];
                //auto flow = mn->dn.flow->operator[](mn->dn.G->arcFromId(arc_id));
                auto flow = mn->cs->flow( mn->dn.G->arcFromId(arc_id) );
                sol[sol.size() - 1][p_id] = flow;
            }
        }
    }

    return sol;
}

std::set<PinPoutR> mrflow::base::FlowRobots::getOneStepTransitions(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
    int t_k)
{
    std::set<PinPoutR> transitions_one_step;
    //[Transition of abstract states]
    //note: abstract state represensts the amount of robots per polygon
    //      e.g. [1 0 2] 1 Robot in Pol1, 0 in Pol2 and 2 in Pol3
    const auto &abstract_state_prev = mn->getAbstractStateSolution(t_k - 1);
    const auto &abstract_state_next = mn->getAbstractStateSolution(t_k);

    //[No transitions]
    if (abstract_state_prev == abstract_state_next)
        return transitions_one_step;

    //[Transitions per Pout polygon]
    for (int p_id = 0; p_id < mn->sn.P; ++p_id)
    {
        // There are robost coming in p_id
        //Lets find where they come from
        mrflow::base::arcId arc_id = mn->arc_ids_configs[t_k][p_id];
        //Get arc and source node
        const auto &arcL2L3 = mn->dn.G->arcFromId(arc_id);
        const auto &nodeL2 = mn->dn.G->source(arcL2L3);
        //Surplus robots come from these olygons
        const auto &pin_pout_R_set_one_pin = this->fromIn(mn, nodeL2);
        for (auto &pin_pout_r_tuple : pin_pout_R_set_one_pin)
            transitions_one_step.insert(pin_pout_r_tuple);
    }

    return transitions_one_step;
}


std::vector<std::set<PinPoutR>> mrflow::base::FlowRobots::getTransitions(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn)
{
    std::vector<std::set<PinPoutR>> transitions;
    for (auto t_k = 1; t_k < mn->T + 1; ++t_k)
    {
        transitions.push_back(this->getOneStepTransitions(mn, t_k));
    }

    return transitions;
}

void mrflow::base::FlowRobots::printTransitions(const std::vector<std::set<PinPoutR>> &trans)
{
    std::cout << "[FlowRobots] Transtions" << std::endl;
    for (auto t_k = 0; t_k < trans.size(); ++t_k)
    {
        std::cout << " [t_k = " << t_k << "]" << std::endl;
        for (auto &set : trans[t_k])
        {
            std::cout << "\t< "
                      << std::get<Trans::PIN>(set) << " , "
                      << std::get<Trans::POUT>(set) << " , "
                      << std::get<Trans::R>(set) << " >"
                      << std::endl;
        }
        std::cout << "~[t_k = " << t_k + 1 << "]" << std::endl;
    }
    std::cout << "~[FlowRobots] Transtions" << std::endl;
}

std::vector<std::set<PinPoutR>> mrflow::base::FlowRobots::getTimeCompressedTransitions(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn)
{
    std::vector<std::set<PinPoutR>> compressed_transitions;
    auto &&transitions = this->getTransitions(mn);
    for (auto &trans_set : transitions)
    {
        if (!trans_set.empty())
        {
            compressed_transitions.push_back(trans_set);
        }
    }
    return compressed_transitions;
}





void mrflow::base::FlowRobots::toSingleMove(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
    std::vector<std::vector<int>> &single_sol)
{
    //[Abstract start state]
    single_sol.emplace_back(mn->getAbstractStateSolution(0));
    for (int t_k = 1; t_k < mn->T + 1; ++t_k) //Solution is T + 1 length
    {
        //[Transition of abstract states]
        //note: abstract state represensts the amount of robots per polygon
        //      e.g. [1 0 2] 1 Robot in Pol1, 0 in Pol2 and 2 in Pol3
        const auto &abstract_state_prev = mn->getAbstractStateSolution(t_k - 1);
        const auto &abstract_state_next = mn->getAbstractStateSolution(t_k);

        //[Remove Dups]
        // If vectors are the same dont add abstract state
        if (abstract_state_prev == abstract_state_next)
            continue;

        //[Single Movements]
        for (int p_id = 0; p_id < mn->sn.P; ++p_id)
        {
            //There are robost coming in p_id
            //Lets find where they come from
            arcId arc_id = mn->arc_ids_configs[t_k][p_id];
            //Get arc and source node
            const auto &arcL2L3 = mn->dn.G->arcFromId(arc_id);
            const auto &nodeL2 = mn->dn.G->source(arcL2L3);
            //Surplus robots come from these olygons
            std::list<mrflow::base::polygonId> polygons;
            this->fromIn(mn, nodeL2, polygons);

            //Single robot transitions
            for (auto p_out : polygons)
            {
                //Last state copy
                auto abstract_state_last = single_sol.back();
                --abstract_state_last[p_out];
                ++abstract_state_last[p_id];
                single_sol.emplace_back(std::move(abstract_state_last));
            }
        }
    }
}





void mrflow::base::FlowRobots::fromIn(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
    const ListDigraph::Node &nodeL2,
    std::list<mrflow::base::polygonId> &polygons)
{
    const auto &arcsL1L2 = mn->dn.G->inArcs(nodeL2);
    const auto &nodeL2_id = mn->dn.G->id(nodeL2);
    const auto &polL2 = mn->node_to_pol[nodeL2_id];
    for (const auto &arcL1L2 : arcsL1L2)
    {
        const auto &nodeL1 = mn->dn.G->source(arcL1L2);
        const auto &nodeL1_id = mn->dn.G->id(nodeL1);
        const auto &polL1 = mn->node_to_pol[nodeL1_id];

        //auto &nR = mn->dn.flow->operator[](arcL1L2);
        const auto &dummy_nR = mn->cs->flow( arcL1L2 );
        int nR = dummy_nR;

        if (nR > 0 && polL1 != polL2){
            while(nR--) polygons.push_back(polL1);
        }
            
    }
}


std::set<PinPoutR> mrflow::base::FlowRobots::fromIn(
    std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
    ListDigraph::Node nodeL2)
{
    std::set<PinPoutR> pin_pout_R;
    const auto &arcsL1L2 = mn->dn.G->inArcs(nodeL2);
    const auto &nodeL2_id = mn->dn.G->id(nodeL2);
    const auto &polL2 = mn->node_to_pol[nodeL2_id];
    for (const auto &arcL1L2 : arcsL1L2)
    {
        const auto &nodeL1 = mn->dn.G->source(arcL1L2);
        const auto &nodeL1_id = mn->dn.G->id(nodeL1);
        const auto &polL1 = mn->node_to_pol[nodeL1_id];

        //const auto &nR = mn->dn.flow->operator[](arcL1L2);
        const auto &nR = mn->cs->flow(arcL1L2);

        if (nR > 0 && polL1 != polL2)
            pin_pout_R.insert(std::make_tuple(polL2, polL1, nR));
    }

    return pin_pout_R;
}