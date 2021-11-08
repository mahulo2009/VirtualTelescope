//
// Created by mhuertas on 06/11/2021.
//
#include "PointModel.h"


int main()
{
    PointModel pmp;
    pmp.read("../../pointmodel.dat");
    pmp.print();
    pmp.fetch();
    return 0;
}
