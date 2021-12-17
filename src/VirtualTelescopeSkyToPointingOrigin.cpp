//
// Created by mhuertas on 17/12/2021.
//

#include "VirtualTelescopeSkyToPointingOrigin.h"
#include <iostream>

vt::main::VirtualTelescopeSkyToPointingOrigin::VirtualTelescopeSkyToPointingOrigin(
        const vt::main::TelescopeContext &telescopeContext) : VirtualTelescope(telescopeContext) {}

void vt::main::VirtualTelescopeSkyToPointingOrigin::fastUpdate() {

    int retval;

    tcsVTxy (
            target_.coordinates[0],
            target_.coordinates[1],
            const_cast<double (*)[3]>(mediumCtx_.mountSPM1),
            target_.frame,
            sin(sideralTime_),
            cos(sideralTime_),
            const_cast<double (*)[3]>(mediumCtx_.mountSPM2),
            pointOrig_.focalStation,
            mountDemand_.demandedRma_,
            mountDemand_.demandedRollTarget_,
            mountDemand_.demandedPitchTarget_,
            mediumCtx_.ia,
            mediumCtx_.ib,
            mediumCtx_.np,
            mediumCtx_.xt,
            mediumCtx_.yt,
            mediumCtx_.zt,
            guidingCorrectionA,
            guidingCorrectionB,
            &poX,
            &poY,
            &retval
    );
}

void vt::main::VirtualTelescopeSkyToPointingOrigin::print() {
    std::cout << "VirtualTelescopeSkyToEncoder: " << std::endl;
    //std::cout << "timestamp\t\t=\t" <<  taiMjd << " MJD(TAI)" <<std::endl; todo
    std::cout << "sidereal time\t=\t" <<  getSideralTime() << " radians" <<std::endl;
    std::cout << "POX\t\t=\t" <<  poX << " millimeters" <<std::endl;
    std::cout << "POY\t\t=\t" <<  poY << " millimeters" <<std::endl;

    std::cout << std::endl;

}
