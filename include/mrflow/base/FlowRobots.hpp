#ifndef FLOW_ROBOTS
#define FLOW_ROBOTS

#include <iostream>
#include <vector>
#include <tuple>
#include "MultirobotNetwork.hpp"

typedef std::tuple<int, int, int> PinPoutR;
enum Trans
{
    PIN,
    POUT,
    R
};

namespace mrflow::base
{
    class FlowRobots
    {

    public:
        FlowRobots(){};
        ~FlowRobots(){};

        /**
		 * @brief Get the Abstract Path object
		 * 
		 * @return 
		 */
        std::vector<std::vector<int>> getAbstractPath(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn);

        /**
		 * @brief Print given abstract path
		 * 
		 * @param path 
		 */
        void printAbstractPath(std::vector<std::vector<int>> path);

        /**
		 * @brief Get the Time Compressed Solution object
		 * Removes duplicated consecutive states
		 * 
		 * @param sol 
		 */
        std::vector<std::vector<int>> getTimeCompressedSolution(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn);

        std::set<PinPoutR> getOneStepTransitions(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
            int t_k);

        std::vector<std::set<PinPoutR>> getTransitions(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn);

        std::vector<std::set<PinPoutR>> getTimeCompressedTransitions(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn);

        void toSingleMove(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
            std::vector<std::vector<int>> &single_sol);

        void printTransitions(const std::vector<std::set<PinPoutR>> &trans);

        /**
		 * @brief Return set of tuple <Pin, Pout, R> for the robots leaving POut and coming Pin
		 * Note: nodeL2 corresponds to a polygon
		 * 
		 * @param nodeL2 
		 * @return std::set<polygonId> 
		 */
        std::set<PinPoutR> fromIn(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
            ListDigraph::Node nodeL2); //TODO: change to list to allow dups or multiset

        void fromIn(
            std::shared_ptr<mrflow::base::MultirobotNetwork> &mn,
            const ListDigraph::Node &nodeL2,
            std::list<mrflow::base::polygonId> &polygons);

    private:
    };
} // namespace mrflow

#endif