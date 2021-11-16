//
// Created by mhuertas on 08/11/2021.
//
#include "PointModel.h"

#include <fstream>
#include <iostream>
#include <regex>
#include <algorithm>
#include <tpt.h>
#include <tcs.h>


void PointModel::read(const std::string &path) {

    maxTerms=100;
    ntRoom=100;
    guidingA=guidingB=0.0;

    pointModelTokeType_  = caption;

    std::ifstream file(path);
    std::string str;
    while (std::getline(file, str)) {
        processToken_(str);
    }
}

void PointModel::processTerm_(std::string &token) {

    token.erase(
            token.begin(),
            std::find_if(
                    token.begin(),
                    token.end(),
                    [](unsigned char ch) { return !std::isspace(ch);}
            )
    );

    std::regex regex{R"(\s+)"}; // split on space and comma
    std::sregex_token_iterator it{token.begin(), token.end(), regex, -1};
    std::sregex_token_iterator end;

    if (std::distance(it,end)!=3) {
        throw std::invalid_argument("");
    }
    PointModelTerm term = {*it,std::stod(*(++it))* AS2R};
    terms_.push_back(term);
}

void PointModel::processToken_(std::string &token) {

    switch (pointModelTokeType_)
    {
        case caption:
            pointModelTokeType_= method;
            break;
        case method:
            pointModelTokeType_= term;
            break;
        case term:
            if (token=="END")
                pointModelTokeType_= end;
            else
                processTerm_(token);
            break;
        case end:
            break;
    }
}

void PointModel::print() {
    for (auto term : terms_)
        std::cout << term << std::endl;
}

void PointModel::fetch() {

    int res = tptMinit (maxTerms, ntRoom, model,
                        &numLocalTerms, &numExplTerms, &numTerms, coefNames);

    for (PointModelTerm &  term : terms_)
    {
        res = tcsAddtrm (maxTerms, ntRoom, const_cast<char*>(term.name.c_str()), term.value,
                         &numTerms, coefNames,coefFormat, model,coefValues);

        //std::cout << "rest " << res << std::endl;
    }
/*
    std::cout << "numTerms " << numTerms << std::endl;
    for (int i=0;i< numTerms;i++) {
        std::cout << model[i] << " " << coefNames[i]  <<" " <<coefValues[i] << std::endl;
    }
    */
}

std::ostream &operator<<(std::ostream &os,const PointModel::PointModelTerm  &t) {
    return os << t.name << " " << t.value;
}
