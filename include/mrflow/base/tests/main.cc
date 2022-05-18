#include <iostream>
#include <map>
#include <memory>

#include "base/MultirobotNetwork.hpp"
#include "base/FlowRobots.hpp"


#include <iostream>


//using namespace cv;

using namespace lemon;
using namespace std;

typedef std::vector<std::vector<int>> PolygonConnectivityGraph;

int main()
{
  std::vector<int> start{2, 1, 0};
  std::vector<int> goal{0, 1, 2};

  int R = 3; //Number of Robots
  int P = 3; //Number of Polygons
  double cost_stay_in_polygon = 0.1;

  //Three Polygon Example
  //TODO: generate matrix or check what I did in configuration free space to testif two polygons are connected
  PolygonConnectivityGraph env;
  env.resize(3);
  env[0].push_back(1);
  env[1].push_back(0);
  env[1].push_back(2);
  env[2].push_back(1);

  std::vector<int> pol_capacity(3);
  pol_capacity[0] = 2;
  pol_capacity[1] = 1;
  pol_capacity[2] = 2;

  std::vector<std::vector<double>> cost_matrix(P, std::vector<double>(P, 0));
  cost_matrix[0][1] = 1.1;
  cost_matrix[1][0] = 1.1;
  cost_matrix[2][1] = 1.1;
  cost_matrix[1][2] = 1.1;

  std::vector<double> cost_stay(P, cost_stay_in_polygon);

  //***************************************************************************************************
  //***************************************************************************************************
  //***************************************************************************************************
  //***************************************************************************************************
  //***************************************************************************************************

  auto mn = std::make_shared<mrflow::base::MultirobotNetwork>(env, pol_capacity, cost_matrix, cost_stay, R);
  mn->create_dynamic_network();

  bool a = mn->query(start, goal,1);

   bool b1 =  mn->query(start, goal,2);

    mn->query(start, goal,3);

    mn->query(start, goal,3);

  //mn->printSolution();
  //mn->getTimeCompressedSolution(path);
  //mn->printAbstractPath(path);
  //mn->printSolution();

  //mrflow::FlowRobots fr;
  // auto path = fr.getTimeCompressedSolution(mn);
  // fr.printAbstractPath(path);
  // auto &&transitions = fr.getTransitions(mn);
  // fr.printTransitions(transitions);
  // auto &&compressed_transitions = fr.getTimeCompressedTransitions(mn);
  // fr.printTransitions(compressed_transitions); 


  mrflow::FlowRobots fr;
  auto solution_path = fr.getAbstractPath(mn);
   fr.printAbstractPath(solution_path);
  std::vector<std::vector<int>> single_move_compressed_path;
  fr.toSingleMove(mn, single_move_compressed_path);
  fr.printAbstractPath(single_move_compressed_path);

  


  
}
