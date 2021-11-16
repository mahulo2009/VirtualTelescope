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

    int maxTerms;
    int ntRoom;
    int model[100];
    double coefValues[100];
    int numLocalTerms;
    int numExplTerms;
    int numTerms;
    char coefNames[100][9];
    char coefFormat[100][9];

    double guidingA;
    double guidingB;

private:

    enum PointModelTokeType { caption, method, term,end };

    void processToken_(std::string &token);
    void processTerm_(std::string &token);

    PointModelTokeType pointModelTokeType_;

    std::vector<PointModelTerm> terms_;
};

std::ostream &operator<<(std::ostream &os, const PointModel::PointModelTerm  &t);

#endif //VIRTUALTELESCOPE_POINTMODEL_H
