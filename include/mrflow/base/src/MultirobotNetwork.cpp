#include "base/MultirobotNetwork.hpp"

using namespace mrflow::base;

MultirobotNetwork::MultirobotNetwork(std::vector<std::vector<int>> env, std::vector<int> c,
                                     std::vector<std::vector<double>> w, std::vector<double> cs, int R) : MultirobotNetwork()
{
  create_static_network(env, c, w, cs, R);
}

bool MultirobotNetwork::query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig)
{
  setup_query(start_Kconfig, goal_Kconfig);

  //Solving the problem using the capacity scaling Algorithm
  CapacityScaling<ListDigraph> cs(*(dn.G));
  // First run
  std::vector<int> *a;
  cs.upperMap(a).costMap(*(dn.CostMap)).stSupply(dn.s, dn.t, sn.R).run();
  //cs.flowMap(flow);
  cs.flowMap(*(dn.flow));

  //Clean previous query
  this->clean_query(start_Kconfig, goal_Kconfig);
  //this->printSolution(); //TODO: comment me
  if (cs.INFEASIBLE)
    std::cerr << "[Mrflow] Infeasible cs" << std::endl;
  if (!cs.OPTIMAL)
    std::cerr << "[Mrflow] Sub-optimal cs" << std::endl;

  this->plot();
  return cs.OPTIMAL;
}

bool MultirobotNetwork::query(
    const std::vector<int> &start_Kconfig,
    const std::vector<int> &goal_Kconfig,
    int Tl)
{
  this->T = Tl;
  setup_query(start_Kconfig, goal_Kconfig, Tl);

  cs->run();
  //cs->flowMap(*(dn.flow));

  //this->plot();
  bool solution_found = foundSolution(goal_Kconfig);
  this->clean_query(start_Kconfig, goal_Kconfig, Tl);
  //this->printSolution(); //TODO: comment me
  if (cs->INFEASIBLE)
  {
    std::cerr << "[Mrflow] Infeasible cs" << std::endl;
    return false;
  }
  if (!cs->OPTIMAL)
  {
    std::cerr << "[Mrflow] Sub-optimal cs" << std::endl;
    if (solution_found)
      return true;
    return false;
  }

  return solution_found;;
}

bool MultirobotNetwork::foundSolution(const std::vector<int> &goal_Kconfig)
{
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    //if (goal_Kconfig[p_id] != dn.flow->operator[](dn.goalConfig[p_id]))
    const auto &arc_goal = dn.goalConfig[p_id];
     if (goal_Kconfig[p_id] != cs->flow(arc_goal) )
    {
      return false;
    }
  }
  return true;
}

MultirobotNetwork::MultirobotNetwork()
{
  //Create polygon expanded time network

  dn.G = new ListDigraph();
  dn.CostMap = new ListDigraph::ArcMap<double>(*(dn.G));
  dn.CapacityMap = new ListDigraph::ArcMap<int>(*(dn.G));
  dn.flow = new ListDigraph::ArcMap<int>(*(dn.G));
  dn.CapacityMapArcIds = new std::vector<int>();
  //Plotting graph
  dn.plot = true;
  dn.coords = new ListDigraph::NodeMap<Point>(*(dn.G));
  dn.sizes = new ListDigraph::NodeMap<double>(*(dn.G));
  dn.colors = new ListDigraph::NodeMap<int>(*(dn.G));
  dn.shapes = new ListDigraph::NodeMap<int>(*(dn.G));
  dn.ecolors = new ListDigraph::ArcMap<int>(*(dn.G));
  dn.widths = new ListDigraph::ArcMap<int>(*(dn.G));
  dn.labels = new ListDigraph::NodeMap<string>(*(dn.G));
  _arc_idf = new ListDigraph::ArcMap<int>(*(dn.G));
}

MultirobotNetwork::~MultirobotNetwork()
{
  delete dn.G;
  delete dn.CostMap;
  delete dn.CapacityMap;
  delete dn.coords;
  delete dn.sizes;
  delete dn.colors;
  delete dn.shapes;
  delete dn.ecolors;
  delete dn.widths;
  delete dn.labels;
  delete dn.CapacityMapArcIds;
}

void MultirobotNetwork::create_static_network(std::vector<std::vector<int>> env, std::vector<int> c,
                                              std::vector<std::vector<double>> w, std::vector<double> cs, int R)
{
  //env - describes the polygon connectity graph of the environment
  //c - Capacity of polygons refers to the max number of robots per polygon
  //w - is the matrix node X node which depicts the distance between polygon centroids
  //cs - is a "fake" cost to make the robots move concurrently
  sn.env_ = env;
  sn.c_ = c;
  sn.w_ = w;
  sn.cs_ = cs;
  sn.R = R;
  sn.P = sn.env_.size();

  this->T = compute_T();
  this->T0 = this->T;
  this->arc_ids_configs = std::vector<std::vector<arcId>>(this->T + 1, std::vector<arcId>(sn.P));
}

void MultirobotNetwork::create_dynamic_network()
{
  //Source and Sink nodes
  dn.s = dn.G->addNode(); //single-source
  dn.t = dn.G->addNode(); //single-sink
  if (dn.plot)
  {
    dn.coords->operator[](dn.s) = Point(5, 5);
    dn.coords->operator[](dn.t) = Point(20 + (T + 2) * 10, 5);
  }

  //1) Source nodes and edges
  std::vector<ListDigraph::Node> *first_layer_1 = new std::vector<ListDigraph::Node>();
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    //1.1) Generate S+ nodes one for each polygon and first layer 1
    //     , generate arc between nodes of thhe same polygon with cost and capacity zero
    //First layer 1
    const auto &p_node_1 = dn.G->addNode();
    first_layer_1->push_back(p_node_1);
    //Set S+
    const auto &u = dn.G->addNode();
    //Add arc between u and p_node_1
    const auto &arc_init = dn.G->addArc(u, p_node_1);
    dn.CapacityMap->operator[](arc_init) = 0; //to be changed during query phase
    dn.CostMap->operator[](arc_init) = 0.0f;
    dn.startConfig.push_back(arc_init); //to get arcs in query

    //1.2) Transfrom Multi-source network into single-source, connect s to S+
    //     , usually capacity is setted to inf but we will set to R number of robots
    //     , it will be probably more numerically stable
    const auto &arc_single_source = dn.G->addArc(dn.s, u);
    dn.CapacityMap->operator[](arc_single_source) = sn.R;
    dn.CostMap->operator[](arc_single_source) = 0.0f;

    if (dn.plot)
    {
      dn.coords->operator[](u) = Point(10, (p_id)*5);
      dn.coords->operator[](p_node_1) = Point(15, (p_id)*5);
      dn.labels->operator[](u) = std::to_string(p_id);
      dn.labels->operator[](p_node_1) = std::to_string(p_id);
    }
  }

  //2) Sink nodes and edges
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    //2.1) Add arc between vl and vll with zero cost and capacity
    //     , this will be utilized in the query phase to assign goal config
    const auto &vl = dn.G->addNode();
    //Set S-
    const auto &vll = dn.G->addNode();
    //Add arc between vl and vll to be used in query phase
    const auto &arc_goal = dn.G->addArc(vl, vll);
    dn.CapacityMap->operator[](arc_goal) = 0; //to be changed in the query phase
    dn.CostMap->operator[](arc_goal) = 0.0f;
    dn.goalConfig.push_back(arc_goal); //to get arcs in query

    //2.2) Transform multiple sink into single sink network
    const auto &arc_single_sink = dn.G->addArc(vll, dn.t);
    dn.CapacityMap->operator[](arc_single_sink) = sn.R;
    dn.CostMap->operator[](arc_single_sink) = 0.0f;

    if (dn.plot)
    {
      dn.coords->operator[](vl) = Point(20 + (T)*10, (p_id + sn.P) * 6);
      dn.coords->operator[](vll) = Point(20 + (T + 1) * 10, (p_id + sn.P) * 6);
      dn.labels->operator[](vl) = std::to_string(p_id);
      dn.labels->operator[](vll) = std::to_string(p_id);
    }
  }

  //3) Generate the time expanded dynamic network
  //Idea: repeat static network (polygons) 3 times (three layers)
  //      , connect it appropriatelly such that distance between polygons
  //      and max number of robot per polygo is accounted for.
  //      . Generate 3*P*T nodes, repeat this three layer T times
  std::vector<std::vector<ListDigraph::Node> *> three_layers(3);

  for (int t = 0; t < T; ++t)
  {
    if (t == 0)
    { //copy first layer
      three_layers[0] = first_layer_1;
      three_layers[1] = new std::vector<ListDigraph::Node>();
      three_layers[2] = new std::vector<ListDigraph::Node>();
    }
    else
    { //copy prev third layer to next first layer
      delete three_layers[0];
      three_layers[0] = three_layers[2];
      delete three_layers[1];
      three_layers[1] = new std::vector<ListDigraph::Node>();
      three_layers[2] = new std::vector<ListDigraph::Node>();
    }
    one_step_dynamic_network(three_layers);

    //Plot data
    if (dn.plot)
    {
      for (int p_id = 0; p_id < sn.P; ++p_id)
      {
        const auto &nodeL2 = three_layers[1]->operator[](p_id);
        const auto &nodeL3 = three_layers[2]->operator[](p_id);
        dn.coords->operator[](nodeL2) = Point(20 + (t)*10, (p_id)*5);
        dn.coords->operator[](nodeL3) = Point(25 + (t)*10, (p_id)*5);
        dn.labels->operator[](nodeL2) = std::to_string(p_id);
        dn.labels->operator[](nodeL3) = std::to_string(p_id);
      }
    }

    //(Solution Retrival) Layer 3 arcs
    //Store pointers to the to be computed flow of the arcs incoming on layer[2]
    //Note: Nodes on layer[2] have a single incoming arc, thus the flow correspods
    //      to the robots in the correspoding node, i.e. each node corresponds to a polygon
    for (int p_id = 0; p_id < sn.P; ++p_id)
    {
      const auto &nodeL3 = three_layers[2]->operator[](p_id);
      ListDigraph::Arc arc_in_nodeL3;
      //Get arc coming in
      dn.G->firstIn(arc_in_nodeL3, nodeL3);
      //t+1 because start config was added before
      this->arc_ids_configs[t + 1][p_id] = dn.G->id(arc_in_nodeL3);
    }

    //(Solution Retrival) Layer 1 and 2 map Nodes to Polygons
    for (int p_id = 0; p_id < sn.P; ++p_id)
    {
      //Layer1
      const auto &nodeL1 = three_layers[0]->operator[](p_id);
      nodeId nodeL1_id = dn.G->id(nodeL1);
      this->node_to_pol.insert({nodeL1_id, p_id});
      //Layer2
      const auto &nodeL2 = three_layers[1]->operator[](p_id);
      nodeId nodeL2_id = dn.G->id(nodeL2);
      this->node_to_pol.insert({nodeL2_id, p_id});
    }
  }

  // //4) End Condition - Connect last three_layers[2] to the sink nodes
  // //   Connect nodes in layer three to respective (polygon) nodes vl
  // for (int p_id = 0; p_id < sn.P; p_id++)
  // {
  //   const auto &nodeL3 = three_layers[2]->operator[](p_id); //node polygon p_id in layer 1
  //   //Get vl form arc vl->vll and connect l3 node to vl
  //   const auto &vl = dn.G->source(dn.goalConfig[p_id]);
  //   const auto arc_L3_vl = dn.G->addArc(nodeL3, vl);
  //   //Add capacity and cost to arc
  //   dn.CapacityMap->operator[](arc_L3_vl) = sn.c_[p_id];
  //   dn.CostMap->operator[](arc_L3_vl) = 0.0f;
  // }

  //Free last three layers
  std::for_each(three_layers.begin(), three_layers.end(), [&](std::vector<ListDigraph::Node> *v) {
    delete v;
  });

  //Initialize Capacity Scaling algorithm with graph and property maps

  //Count num of arcs/edges
  int num_arcs = 0;
  int num_nodes = 0;
  for (ListDigraph::ArcIt a(*(dn.G)); a != INVALID; ++a)
  {
    num_arcs++;
  }
  for (ListDigraph::NodeIt a(*(dn.G)); a != INVALID; ++a)
  {
    num_nodes++;
  }
  (*(dn.CapacityMapArcIds)).resize(2 * (num_nodes + num_arcs));
  for (int i = 0; i < 2 * (num_nodes + num_arcs); ++i)
  {
    dn.CapacityMapArcIds->operator[](i) = std::numeric_limits<int>::max();
  }  
  int j = 0;
  for (ListDigraph::NodeIt n(*(dn.G)); n != INVALID; ++n)
  {
    for (ListDigraph::OutArcIt a(*(dn.G), n); a != INVALID; ++a, ++j)
    {
      _arc_idf->operator[](a) = j;
    }
    for (ListDigraph::InArcIt a(*(dn.G), n); a != INVALID; ++a, ++j)
    {
    }
    ++j;
  }
  for (ListDigraph::ArcIt arc(*(dn.G)); arc != INVALID; ++arc)
  {
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc)) = dn.CapacityMap->operator[](arc);
  }
  cs = new CapacityScaling<ListDigraph>(*(dn.G));
  cs->upperMap(dn.CapacityMapArcIds).costMap(*(dn.CostMap)).stSupply(dn.s, dn.t, sn.R);
}

void MultirobotNetwork::one_step_dynamic_network(
    std::vector<std::vector<ListDigraph::Node> *> &three_layers)
{
  //Populate Layers with nodes
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    auto p_node_2 = dn.G->addNode();
    three_layers[1]->push_back(p_node_2);
    auto p_node_3 = dn.G->addNode();
    three_layers[2]->push_back(p_node_3);
  }

  //1) Arcs between layer 1 and 2
  //1.1) Stay
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    const auto &nodeL1 = three_layers[0]->operator[](p_id); //node polygon p_id in layer 1
    const auto &nodeL2 = three_layers[1]->operator[](p_id); //node polygon p_id in layer 2
    const auto &arc12 = dn.G->addArc(nodeL1, nodeL2);
    //Add capacity and cost to arc
    dn.CapacityMap->operator[](arc12) = sn.c_[p_id];
    dn.CostMap->operator[](arc12) = sn.cs_[p_id];
  }

  //1.2) Exchange polygons if connected
  for (int p_id1 = 0; p_id1 < sn.P; p_id1++)
  {
    for (auto p_id2 : sn.env_[p_id1])
    {
      //p_id1 and p_id2 are connected
      const auto &nodeL1 = three_layers[0]->operator[](p_id1);
      const auto &nodeL2 = three_layers[1]->operator[](p_id2);
      const auto &arc12 = dn.G->addArc(nodeL1, nodeL2);
      //Add capacity and cost to arc
      dn.CapacityMap->operator[](arc12) = sn.c_[p_id1];
      dn.CostMap->operator[](arc12) = sn.w_[p_id1][p_id2];
    }
  }

  //2) Arcs between layer 2 and 3
  //Note: ensure max amount of robot per polygon
  //      the same as the "stay" in arc from layer
  //      1 to layer 2, but with zero cost
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    const auto &nodeL2 = three_layers[1]->operator[](p_id); //node polygon p_id in layer 1
    const auto &nodeL3 = three_layers[2]->operator[](p_id); //node polygon p_id in layer 2
    const auto &arc23 = dn.G->addArc(nodeL2, nodeL3);
    //Add capacity and cost to arc
    dn.CapacityMap->operator[](arc23) = sn.c_[p_id];
    dn.CostMap->operator[](arc23) = 0.0f;
  }

  // //3) Sink Condition
  // //   Connect nodes in layer three to respective (polygon) nodes vl
  // for(int p_id = 0; p_id < sn.P; p_id++){
  //   const auto &nodeL3 = three_layers[2]->operator[](p_id);//node polygon p_id in layer 1
  //   //Get vl form arc vl->vll and connect l3 node to vl
  //   const auto &vl = dn.G->source( dn.goalConfig[p_id] );
  //   const auto arc_L3_vl = dn.G->addArc(nodeL3, vl);
  //   //Add capacity and cost to arc
  //   dn.CapacityMap->operator[](arc_L3_vl) = sn.c_[p_id];
  //   dn.CostMap->operator[](arc_L3_vl) = 0.0f;
  // }

  //3) Sink Condition --- dummy edges with zero cost and zero capacity
  //   Connect nodes in layer two to respective (polygon) nodes vl
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    const auto &nodeL2 = three_layers[1]->operator[](p_id); //node polygon p_id in layer 1
    //Get vl form arc vl->vll and connect l3 node to vl
    const auto &vl = dn.G->source(dn.goalConfig[p_id]);
    const auto arc_L2_vl = dn.G->addArc(nodeL2, vl);
    //Add capacity and cost to arc
    dn.CapacityMap->operator[](arc_L2_vl) = 0; // sn.c_[p_id];
    dn.CostMap->operator[](arc_L2_vl) = 0.0f;
  }
}

void MultirobotNetwork::setup_query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig)
{
  //Assign capacities of source and sink edges in order to depict start and goal configuration
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    //1) Assign capacities for source arcs, arc between set S+ and first layer
    const auto &arc_source = dn.startConfig[p_id];
    dn.CapacityMap->operator[](arc_source) = start_Kconfig[p_id];
    //2) Assign capacities for sink arcs, arc between set of vl nodes and S- (or vll nodes)
    const auto &arc_sink = dn.goalConfig[p_id];
    dn.CapacityMap->operator[](arc_sink) = goal_Kconfig[p_id];

    //Update persistant map
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_source)) = dn.CapacityMap->operator[](arc_source);
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_sink)) = dn.CapacityMap->operator[](arc_sink);
  }

  //Add start state to solution
  for (int p_id = 0; p_id < sn.P; ++p_id)
  {
    const auto &arc_source = dn.startConfig[p_id];
    this->arc_ids_configs[0][p_id] = dn.G->id(arc_source);
  }
}
void MultirobotNetwork::setup_query(
    const std::vector<int> &start_Kconfig,
    const std::vector<int> &goal_Kconfig,
    int Tl)
{
  setup_query(start_Kconfig, goal_Kconfig);

  // //   Connect nodes in layer three at expansion Tl to respective (polygon) nodes vl
  // for(int p_id = 0; p_id < sn.P; p_id++){
  //   int arc_L2_l3_id = this->arc_ids_configs[Tl][p_id];
  //   const auto &arc_L2_l3 = dn.G->arcFromId(arc_L2_l3_id);
  //   const auto &nodeL3 = dn.G->target(arc_L2_l3);
  //   //Get vl form arc vl->vll and connect l3 node to vl
  //   const auto &vl = dn.G->source( dn.goalConfig[p_id] );
  //   const auto arc_L3_vl = dn.G->addArc(nodeL3, vl);
  //   //Add capacity and cost to arc
  //   dn.CapacityMap->operator[](arc_L3_vl) = sn.c_[p_id];
  //   dn.CostMap->operator[](arc_L3_vl) = 0.0f;
  // }

  //Modify capacities layer 3 at Tl and close water afterwards
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    int arc_L2_l3_id = this->arc_ids_configs[Tl][p_id];
    const auto &arc_L2_l3 = dn.G->arcFromId(arc_L2_l3_id);
    const auto &nodeL2 = dn.G->source(arc_L2_l3);
    ListDigraph::Arc arc_L2_vl;
    dn.G->firstOut(arc_L2_vl, nodeL2);
    if (arc_L2_vl == arc_L2_l3)
    { //there are only 2 arcs
      dn.G->nextOut(arc_L2_vl);
    }
    //Open l2 --> vl
    dn.CapacityMap->operator[](arc_L2_vl) = sn.c_[p_id];
    //Close l2 --> l3
    dn.CapacityMap->operator[](arc_L2_l3) = 0;

    //Update tthe other map too
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_L2_vl)) = dn.CapacityMap->operator[](arc_L2_vl);
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_L2_l3)) = dn.CapacityMap->operator[](arc_L2_l3);
  }
}

void MultirobotNetwork::clean_query(const std::vector<int> &start_Kconfig, const std::vector<int> &goal_Kconfig)
{
  //Assign capacities of source and sink edges in order to depict start and goal configuration
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    //1) Assign capacities for source arcs, arc between set S+ and first layer
    const auto &arc_source = dn.startConfig[p_id];
    dn.CapacityMap->operator[](arc_source) = 0;
    //2) Assign capacities for sink arcs, arc between set of vl nodes and S- (or vll nodes)
    const auto &arc_sink = dn.goalConfig[p_id];
    dn.CapacityMap->operator[](arc_sink) = 0;

    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_source)) = dn.CapacityMap->operator[](arc_source);
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_sink)) = dn.CapacityMap->operator[](arc_sink);
  }
}
void MultirobotNetwork::clean_query(
    const std::vector<int> &start_Kconfig,
    const std::vector<int> &goal_Kconfig,
    int Tl)
{
  clean_query(start_Kconfig, goal_Kconfig);

  // //Delete edges from layer 3 expansion Tl and vl nodes
  // for(int p_id = 0; p_id < sn.P; p_id ++){
  //   const auto &vl = dn.G->source( dn.goalConfig[p_id] );
  //   ListDigraph::Arc arc_L3_vl;
  //   dn.G->firstIn(arc_L3_vl, vl);//There is onl one arc incoming on vl
  //   dn.G->erase(arc_L3_vl);
  // }
  //Modify capacities layer 3 at Tl and close water afterwards
  for (int p_id = 0; p_id < sn.P; p_id++)
  {
    int arc_L2_l3_id = this->arc_ids_configs[Tl][p_id];
    const auto &arc_L2_l3 = dn.G->arcFromId(arc_L2_l3_id);
    const auto &nodeL2 = dn.G->source(arc_L2_l3);
    ListDigraph::Arc arc_L2_vl;
    dn.G->firstOut(arc_L2_vl, nodeL2);
    if (arc_L2_vl == arc_L2_l3)
    { //there are only 2 arcs
      dn.G->nextOut(arc_L2_vl);
    }
    //Open l2 --> l3
    dn.CapacityMap->operator[](arc_L2_l3) = sn.c_[p_id];
    //Close l2 --> vl
    dn.CapacityMap->operator[](arc_L2_vl) = 0;

    //Update tthe other map too
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_L2_vl)) = dn.CapacityMap->operator[](arc_L2_vl);
    dn.CapacityMapArcIds->operator[](_arc_idf->operator[](arc_L2_l3)) = dn.CapacityMap->operator[](arc_L2_l3);
  }
}

int MultirobotNetwork::compute_T()
{
  //T = R + l - 1
  // R: number_of_robots
  // l: max_polygon_transitions_in_static_network
  int R = sn.R;
  int l = sn.P;

  int T = R + l - 2;
  return T;
}

void MultirobotNetwork::plot()
{
  cs->flowMap(*(dn.flow));
  //Print Graph
  for (ListDigraph::NodeIt n(*(dn.G)); n != INVALID; ++n)
  {
    if (dn.G->id(n) == 0)
    {
      dn.labels->operator[](n) = "S";
      dn.sizes->operator[](n) = 1;
      dn.colors->operator[](n) = 4;
      dn.shapes->operator[](n) = 1;
      continue;
    }
    if (dn.G->id(n) == 1)
    {
      dn.labels->operator[](n) = "T";
      dn.sizes->operator[](n) = 1;
      dn.colors->operator[](n) = 4;
      dn.shapes->operator[](n) = 1;
      continue;
    }
    dn.sizes->operator[](n) = 1;
    dn.colors->operator[](n) = 2;
    dn.shapes->operator[](n) = 0;
  }
  for (ListDigraph::NodeIt n(*(dn.G)); n != INVALID; ++n)
  {
    if (dn.G->id(n) >= 2 && dn.G->id(n) <= sn.P * 4 + 1)
    {
      dn.sizes->operator[](n) = 1;
      dn.colors->operator[](n) = 5;
      dn.shapes->operator[](n) = 2;
      for (int p_id = 0; p_id < sn.P; ++p_id)
      {
        if (dn.G->id(n) == (p_id + 1) * 2)
        {
          dn.sizes->operator[](n) = 1;
          dn.colors->operator[](n) = 2;
          dn.shapes->operator[](n) = 0;
        }
      }
    }
  }

  for (ListDigraph::ArcIt a(*(dn.G)); a != INVALID; ++a)
  {
    if (dn.flow->operator[](a) == 0)
    {
      dn.ecolors->operator[](a) = 0;
      dn.widths->operator[](a) = 1;
    }
  }

  for (ListDigraph::ArcIt a(*(dn.G)); a != INVALID; ++a)
  {
    if (dn.flow->operator[](a) != 0)
    {
      dn.ecolors->operator[](a) = 1;
      dn.widths->operator[](a) = dn.flow->operator[](a);
    }
  }

  Palette palette;
  Palette paletteW(true);
  IdMap<ListDigraph, ListDigraph::Node> id(*(dn.G));
  if (dn.plot)
  {
    graphToEps(*(dn.G), "mrflow2.eps").
        //scale(9).
        coords(*(dn.coords))
            .title("Sample .eps figure (with arrowheads)")
            .copyright("(C) 2003-2007 LEMON Project")
            .absoluteNodeSizes()
            .absoluteEdgeWidths()
            .nodeColors(composeMap(palette, *(dn.colors)))
            .coords(*(dn.coords))
            .nodeScale(1)
            .nodeSizes(*(dn.sizes))
            .nodeShapes(*(dn.shapes))
            .edgeColors(composeMap(palette, *(dn.ecolors)))
            .edgeWidthScale(.4)
            .edgeWidths(*(dn.widths))
            .nodeTexts(*(dn.labels))
            .nodeTextSize(2)
            .drawArrows()
            .arrowWidth(0.2)
            .arrowLength(0.2)
            .run();
  }
}

void MultirobotNetwork::printSolution()
{
  if (this->arc_ids_configs.empty())
  {
    std::cerr << "[MRFLOW] No Solution" << std::endl;
    return;
  }

  for (auto vec : this->arc_ids_configs)
  {
    std::cout << "[ ";
    for (auto arc_id : vec)
    {
      //std::cout << " " << dn.flow->operator[](dn.G->arcFromId(arc_id)) << " ";
      std::cout << " " << cs->flow(dn.G->arcFromId(arc_id)) << " ";
    }
    std::cout << "  ]" << std::endl;
  }
}

std::vector<int> MultirobotNetwork::getAbstractStateSolution(int t_k)
{
  std::vector<int> abstract_state = std::vector<int>(sn.P);
  for (int p_id = 0; p_id < this->sn.P; ++p_id)
  {
    //Get arc id of poligon p_id at discrete time k
    auto arc_id = arc_ids_configs[t_k][p_id];
    //Flow = amount of robots in Poligon p_id at discrete time k
    //auto nR = dn.flow->operator[](dn.G->arcFromId(arc_id));
    auto nR = cs->flow(dn.G->arcFromId(arc_id));
    //Populate abstarct state
    abstract_state[p_id] = nR;
  }
  return abstract_state;
}
