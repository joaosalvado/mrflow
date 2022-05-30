//
// Created by ohmy on 2022-05-30.
//

#ifndef MRFLOW_MRPATH_H
#define MRFLOW_MRPATH_H

#include <vector>

class MrPath {
    std::vector<std::vector<int>> polygons_sequence;
    struct Ellipse{
        double a; // minor-axis
        double b; // major-axis
        double cx; // center x
        double cy; // center y
    };
    std::vector<Ellipse> obstacles;
    MrPath(std::vector<std::vector<int>> polygons_sequence_){
        polygons_sequence = polygons_sequence_;
    }

};


#endif //MRFLOW_MRPATH_H
