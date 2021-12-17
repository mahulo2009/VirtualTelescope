//
// Created by mhuertas on 17/12/2021.
//

#include "VirtualTelescopeSkyToEncoder.h"
#include <iostream>

vt::main::VirtualTelescopeSkyToEncoder::VirtualTelescopeSkyToEncoder(const vt::main::TelescopeContext &telescopeContext)
        : VirtualTelescope(telescopeContext) {}


void vt::main::VirtualTelescopeSkyToEncoder::fastUpdate()  {

    int max_iterations=100; //todo

    //std::cout << "----------------------------VirtualTelescope::vtSkyToEnc" << std::endl;

    double aimvect[3];
    double newroll1, newpitch1;
    double newroll2, newpitch2;

    int nIter=0;
    bool solutionConverged = false;
    double newroll=0, newpitch=0, newrma=0;

    while (!solutionConverged && nIter<max_iterations)
    {
        int retval;
        tcsTrack (
                //Input
                target_.coordinates[0],                                            //target "roll" coordinate
                target_.coordinates[1],                                           //target "pitch" coordinate
                const_cast<double (*)[3]>(mediumCtx_.mountSPM1),     //SPM #1, mount
                target_.frame,                                       //reference frame for the target
                sin(sideralTime_),                                   //sine of sidereal time
                cos(sideralTime_),                                   //cosine of sidereal time
                const_cast<double (*)[3]>(mediumCtx_.mountSPM2),     //SPM #2, mount
                pointOrig_.focalStation,                             //rotator location
                newrma,                                              //predicted rotator mechanical angle
                newroll,                                             //predicted roll
                newpitch,                                            //predicted pitch
                pointOrig_.coordinates[0],                                                //pointing origin x (in focal lengths) //todo scale
                pointOrig_.coordinates[1],                                                //pointing origin y (in focal lengths) //todo scale
                mediumCtx_.ia,                                       //roll zero point
                mediumCtx_.ib,                                       //pitch zero point
                mediumCtx_.np,                                       //mount axes nonperpendicularity
                mediumCtx_.xt,                                       //telescope vector, x-component
                mediumCtx_.yt,                                       //telescope vector, y-component
                mediumCtx_.zt,                                       //telescope vector, z-component
                guidingCorrectionA,                             //guiding correction, collimation
                guidingCorrectionB,                             //guiding correction, pitch
                0,                                                   //radius of "no go" region
                //Output
                &aimvect[0],                                         //AIM x-coordinate
                &aimvect[1],                                         //AIM y-coordinate
                &aimvect[2],                                         //AIM z-coordinate
                &newroll1,                                           //roll coordinate, first solution
                &newpitch1,                                          //pitch coordinate, first solution
                &newroll2,                                           //roll coordinate, second solution
                &newpitch2,                                          //pitch coordinate, second solution
                &retval
        );

        newroll =  newroll1;
        newpitch = newpitch1;

        tcsRotator (
                //Input
                aimvect[0],                                             //AIM x-coordinate
                aimvect[1],                                             //AIM y-coordinate
                aimvect[2],                                             //AIM z-coordinate
                pointOrig_.focalStation,                                //rotator location
                newrma,                                                 //predicted rotator demand
                0,                                                      //FALSE/TRUE = above/below pole
                newroll,                                                //demand roll
                newpitch,                                               //demand pitch
                pointOrig_.coordinates[0],                                                    //pointing-origin x (in focal lengths) todo scale
                pointOrig_.coordinates[1],                                                  //pointing-origin y (in focal lengths) todo scale
                mediumCtx_.ia,                                          //roll zero point
                mediumCtx_.ib,                                          //pitch zero point
                mediumCtx_.np,                                          //nonperpendicularity
                mediumCtx_.xt,                                          //telescope vector, x-component
                mediumCtx_.yt,                                          //telescope vector, y-component
                mediumCtx_.zt,                                          //telescope vector, z-component
                guidingCorrectionA,                                //guiding adjustment, collimation
                guidingCorrectionB,                                //guiding adjustment, pitch
                0.0,                                                    //sine of Instrument Alignment Angle
                1.0,                                                    //cosine of Instrument Alignment Angle
                const_cast<double (*)[3]>(mediumCtx_.rotatorSPM1_i),    //inverse SPM #1 for the rotator
                pointOrig_.frame,                                       //rotator tracking frame ID
                sin(sideralTime_),                                      //sine of sidereal time
                cos(sideralTime_),                                      //cosine of sidereal time
                const_cast<double (*)[3]>(mediumCtx_.rotatorSPM2_i),    //inverse SPM #2 for the rotator
                pointOrig_.ipa,                                         //requested Instrument Position Angle
                //Output
                &newrma,                                                //required rotator mechanical angle
                &retval                                                 //status: 0=OK
        );

        nIter++;
        if (fabs(newroll-encoderRoll)<1e-10 && fabs(newpitch-encoderPitch)<1e-10 && fabs(newrma-encoderRma)<1e-10)
            solutionConverged = true;
        else
        {
            encoderRoll = newroll;
            encoderPitch = newpitch;
            encoderRma = newrma;
        }
    }

    /*
    std::cout << "enc_roll " << enc_roll  << std::endl;
    std::cout << "enc_pitch " << enc_pitch   << std::endl;
    std::cout << "enc_rma " << enc_rma  << std::endl;

    std::cout << "----------------------------VirtualTelescope::vtSkyToEnc END" << std::endl;
    */

}

void vt::main::VirtualTelescopeSkyToEncoder::print() {

    std::cout << "VirtualTelescopeSkyToEncoder: " << std::endl;
    //std::cout << "timestamp\t\t=\t" <<  taiMjd << " MJD(TAI)" <<std::endl; todo
    std::cout << "sidereal time\t=\t" <<  getSideralTime() << " radians" <<std::endl;
    std::cout << "Az demand\t\t=\t" <<  (M_PI-encoderRoll)*(180/M_PI) << " degrees" <<std::endl;
    std::cout << "Elv demand\t\t=\t" <<  encoderPitch*(180/M_PI) << " degrees" <<std::endl;
    std::cout << "rotator demand\t=\t" <<  encoderRma*(180/M_PI) << " degrees" <<std::endl;
    std::cout << std::endl;
}

