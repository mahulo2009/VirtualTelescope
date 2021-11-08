//
// Created by mhuertas on 08/11/2021.
//

#ifndef VIRTUALTELESCOPE_POINTMODEL_H
#define VIRTUALTELESCOPE_POINTMODEL_H

#include <string>
#include <vector>

class PointModel {

public:

    struct PointModelTerm {
        std::string  name;
        double value;
    };

    void read(const std::string &path);
    void print();
    void fetch();

private:

    enum PointModelTokeType { caption, method, term,end };

    void processToken_(std::string &token);
    void processTerm_(std::string &token);

    PointModelTokeType pointModelTokeType_;

    std::vector<PointModelTerm> terms_;
};

std::ostream &operator<<(std::ostream &os, const PointModel::PointModelTerm  &t);

#endif //VIRTUALTELESCOPE_POINTMODEL_H
