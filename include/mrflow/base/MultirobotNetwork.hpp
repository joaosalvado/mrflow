
#pragma once
#include <iostream>
#include <lemon/list_graph.h>
#include <lemon/capacity_scaling.h>
#include <lemon/graph_to_eps.h>
#include <lemon/math.h>
#include <map>
#include <memory>
#include <limits>
#include <unordered_map>
#include <set>



using namespace lemon;
using namespace std;

namespace mrflow::base
{
	typedef int arcId;
	typedef int nodeId;
	typedef std::size_t polygonId;

	class MultirobotNetwork
	{
		//typedef std::vector<std::vector<int>> PolygonConnectivityGraph;

		typedef struct static_network
		{
			std::vector<std::vector<int>> env_;
			std::vector<int> c_;				 //capacities
			std::vector<std::vector<double>> w_; //weights /cost
			std::vector<double> cs_;			 //stay cost
			int R, P;
		} static_network;

		typedef dim2::Point<int> Point;

		typedef struct dynamic_network
		{
			ListDigraph *G;
			ListDigraph::ArcMap<double> *CostMap;
			ListDigraph::ArcMap<int> *CapacityMap;
			std::vector<int>* CapacityMapArcIds;
			ListDigraph::ArcMap<int> *flow;
			ListDigraph::Node s; //single-source
			ListDigraph::Node t; //single-sink
			std::vector<ListDigraph::Arc> startConfig;
			std::vector<ListDigraph::Arc> goalConfig;
			//Ploting
			bool plot;
			ListDigraph::NodeMap<Point> *coords;
			ListDigraph::NodeMap<double> *sizes;
			ListDigraph::NodeMap<int> *colors;
			ListDigraph::NodeMap<int> *shapes;
			ListDigraph::ArcMap<int> *ecolors;
			ListDigraph::ArcMap<int> *widths;
			ListDigraph::NodeMap<string> *labels;
		} dynamic_network;

	private:
		friend class FlowRobots;
		CapacityScaling<ListDigraph> *cs;
		static_network sn;
		dynamic_network dn;
		std::vector<std::vector<arcId>> arc_ids_configs;   //Matrix of arc_ids, in time and polygons
		std::unordered_map<nodeId, polygonId> node_to_pol; //Convertion of node to pol
		int T;
		int T0;
		//Persistant map 
		ListDigraph::ArcMap<int> *_arc_idf;
		/**
		 * @brief Create a static network object
		 * 
		 * @param env	polygons connectivity	
		 * @param c		polygons max capacity		
		 * @param w 	transition cost matrix	
		 * @param cs	polygons cost stay 
		 * @param R 	aumount of robots
		 */
		void create_static_network(std::vector<std::vector<int>> env, std::vector<int> c,
								   std::vector<std::vector<double>> w, std::vector<double> cs, int R);

		/**
		 * @brief One step is representend by three layer of nodes.
		 * Each layer is a static network, or we can see it as the cfree.
		 * Edges between layer 1 and layer 2 model robot transitions between polygons (allow robots to stay also)
		 * Edges between layer 2 and layer 3 model polygons max capaxity costraints
		 * 
		 * @param three_layers 
		 */
		void one_step_dynamic_network(std::vector<std::vector<ListDigraph::Node> *> &three_layers);

		/**
		 * @brief Compute the maximum number of polygons transitions. Which is:
		 * 								T = P + R -1
		 * Amount of polygons plus amount of robots minus one.
		 * 
		 * @return int 
		 */
		int compute_T();

		/**
		 * @brief Modify dynamic network to setup it up for querying.
		 * This will change previous 0 capacities to the amount of robots here given
		 * 
		 * @param start_Kconfig 
		 * @param goal_Kconfig 
		 */
		void setup_query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig);
		void setup_query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig, int T);

		/**
		 * @brief Re-set edge capacities modified in setup_query to zero
		 * 
		 * @param start_Kconfig 
		 * @param goal_Kconfig 
		 */
		void clean_query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig);
		void clean_query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig, int T);
		/**
		 * @brief Get Abstract state at discrete time t_k from previously computed solution
		 * 
		 * @param t_k	discrete time
		 * @return std::vector<int> 
		 */
		std::vector<int> getAbstractStateSolution(int t_k);

		/**
		 * @brief Get the Flow / num of robots
		 * 
		 * @param arc_id 
		 * @return int 
		 */
		int getFlow(arcId arc_id) { return this->dn.flow->operator[](this->dn.G->arcFromId(arc_id)); }

		bool foundSolution(const std::vector<int> &goal_Kconfig);

	public:
		/**
		 * @brief Create a dynamic network object
		 * 
		 */
		void create_dynamic_network();

		/**
		 * @brief Query the dynamic netwok
		 * 
		 * @param start_Kconfig 
		 * @param goal_Kconfig 
		 * @return true 		success
		 * @return false 		failure
		 */
		bool query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig);
		bool query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig, int T);
		/**
		 * @brief Plot dynamic network and solution flow
		 * 
		 */
		void plot();

		/**
		 * @brief Solution of query
		 * 
		 */
		void printSolution();

		/**
		 * @brief Get the Num Nodes object
		 * 
		 * @return int 
		 */
		int getNumNodes() const
		{
			int num_nodes = 0;
			for (ListDigraph::NodeIt n(*(dn.G)); n != INVALID; ++n)
				num_nodes++;
			return num_nodes;
		}

		int getNumEdges() const
		{
			int num_edges = 0;
			for (ListDigraph::ArcIt a(*(dn.G)); a != INVALID; ++a)
				num_edges++;
			return num_edges;
		}

		int getMaxT() const{return this->T0;}

		MultirobotNetwork();
		MultirobotNetwork(std::vector<std::vector<int>> env, std::vector<int> c,
						  std::vector<std::vector<double>> w, std::vector<double> cs, int R);
		virtual ~MultirobotNetwork();
	};

} // namespace mrflow