//
// Created by mhuertas on 17/12/2021.
//

#include "VirtualTelescopeEncoderToSky.h"
#include "slalib.h"
#include <iostream>
#include <sstream>


vt::main::VirtualTelescopeEncoderToSky::VirtualTelescopeEncoderToSky(const vt::main::TelescopeContext &telescopeContext)
        : VirtualTelescope(telescopeContext) {}


void vt::main::VirtualTelescopeEncoderToSky::fastUpdate()  {

    //make the transformation
    tcsVTsky (
            mountDemand_.demandedRollTarget_,
            mountDemand_.demandedPitchTarget_,
            pointOrig_.focalStation,
            mountDemand_.demandedRma_,
            pointOrig_.coordinates[0],
            pointOrig_.coordinates[1],
            const_cast<double (*)[3]>(mediumCtx_.mountSPM1_i),
            target_.frame,
            sin(sideralTime_),
            cos(sideralTime_),
            const_cast<double (*)[3]>(mediumCtx_.mountSPM2_i),
            mediumCtx_.ia,
            mediumCtx_.ib,
            mediumCtx_.np,
            mediumCtx_.xt,
            mediumCtx_.yt,
            mediumCtx_.zt,
            guidingCorrectionA,
            guidingCorrectionB,
            &sky_longitude,
            &sky_latitude
    );

    sky_longitude = slaDranrm(sky_longitude);

}

void vt::main::VirtualTelescopeEncoderToSky::print() {

    char sign;
    int conv[4];
    std::ostringstream o0;
    std::ostringstream o1;
    std::ostringstream o2;

    //ST
    slaCr2tf (4,getSideralTime(), &sign, conv);
    o0<<sign<<conv[0]<<" "<<conv[1]<<" "<<conv[2]<<"."<<conv[3];

    //RA
    slaDr2tf (4, sky_longitude, &sign, conv);
    o1<<conv[0]<<" "<<conv[1]<<" "<<conv[2]<<"."<<conv[3];
    o1<<" ";

    //Dec
    slaDr2af (4,sky_latitude, &sign, conv);
    o2<<sign<<conv[0]<<" "<<conv[1]<<" "<<conv[2]<<"."<<conv[3];

    std::cout << "VirtualTelescopeEncoderToSky: " << std::endl;
    std::cout << "RA: "<< sky_longitude << std::endl;
    std::cout << "DEC: "<< sky_latitude << std::endl;
    //std::cout << "timestamp\t\t=\t" <<  taiMjd << " MJD(TAI)" <<std::endl; todo
    std::cout << "sidereal time\t=\t" <<  o0.str() << " HH:mm:ss" <<std::endl;
    std::cout << "RA demand\t\t=\t" <<  o1.str() << " HH:mm:ss" <<std::endl;
    std::cout << "DEC demand\t\t=\t" <<  o2.str() << " DD:mm:ss" <<std::endl;
    std::cout << std::endl;

}

