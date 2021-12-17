//
// Created by mhuertas on 06/11/2021.
//

#include "VirtualTelescope.h"

#include <iostream>
#include <iomanip>
#include "slalib.h"

vt::main::VirtualTelescope::VirtualTelescope(const TelescopeContext & telescopeContext) :
                                                    telescopeContext_(telescopeContext)
{
}


void vt::main::VirtualTelescope::init() {

    //std::cout << "----------------------------VirtualTelescope::init" << std::endl;
    //std::cout << std::setprecision(15) << std::endl;

    //call to the tcsInit2 function
    tcsInit2 (
              //Input
              telescopeContext_.geographicalParameters.meanLon,     //site mean east longitude [radians]
              telescopeContext_.geographicalParameters.meanLat,     //site mean geodetic latitude [radians]
              telescopeContext_.polarMotionParameters.polarMotionX, //polar-motion x angle [radians]
              telescopeContext_.polarMotionParameters.polarMotionY, //polar-motion y angle [radians]
              telescopeContext_.geographicalParameters.height,      //site elevation, meters above sea-level. [meter]
              telescopeContext_.opticalParameters.mountType,        //mount type altazimutal.
              0,                //0 for altazimutal
              0,                //0 for altazimutal
              0,                //0 for altazimutal
              //Output
              azElToNominalMount,     //rotation matrix, [Az,El] to nominal mount.
              &trueGeographicalParameters_.trueLon,                       //telescope longitude true [radians]
              &trueGeographicalParameters_.trueLat,                       //telescope latitude true [radians]
              &trueGeographicalParameters_.axisDistanceAu,                //distance from spin axis [AU]
              &trueGeographicalParameters_.equatorDistanceAu,             //distance from equator [AU]
              &trueGeographicalParameters_.axisDistanceKm,                //distance from spin axis [km]
              &trueGeographicalParameters_.equatorDistanceKm,             //distance from equator [km]
              &trueGeographicalParameters_.diurnalAberration);            //diurnal aberration [radians]

    /*
    std::cout << "telescopeParams_.azElToNominalMount[0][0] " << telescopeParams_.azElToNominalMount[0][0] << std::endl;
    std::cout << "geoData_.trueLon " << geoData_.trueLon << std::endl;
    std::cout << "geoData_.trueLat " << geoData_.trueLat<< std::endl;
    std::cout << "geoData_.axisDistanceAu "<< geoData_.axisDistanceAu<< std::endl;
    std::cout << "geoData_.equatorDistanceAu " << geoData_.equatorDistanceAu<< std::endl;
    std::cout << "geoData_.axisDistanceKm "<< geoData_.axisDistanceKm<< std::endl;
    std::cout << "geoData_.equatorDistanceKm " << geoData_.equatorDistanceKm<< std::endl;
    std::cout << "geoData_.diurnalAberration " << geoData_.diurnalAberration<< std::endl;
    std::cout << "----------------------------VirtualTelescope::init END" << std::endl;
    */
}

void vt::main::VirtualTelescope::slowUpdate() {

    /*
    std::cout << "----------------------------VirtualTelescope::slowUpdate" << std::endl;
    std::cout << std::setprecision(15) << std::endl;
    std::cout << "iers_.ut1ToUtc " << iers_.ut1ToUtc << std::endl;
    std::cout << "iers_.taiToUtc " << iers_.taiToUtc << std::endl;
    std::cout << "iers_.ttToTai " << iers_.ttToTai << std::endl;
    */

    tcsSlow (
            //Input
            taiMjd_,                        //time (TAI Modified Julian Data, JD-2000000.5) [seconds] //todo review this value differnt in maedium (comment)
            telescopeContext_.geographicalParameters.height,                //site elevation, meters above sea-level [meter].
            telescopeContext_.timeTransformParameters.ut1ToUtc,                 //current UT1-UTC [day]
            telescopeContext_.timeTransformParameters.taiToUtc,                 //TAI-UTC [day]
            telescopeContext_.timeTransformParameters.ttToTai,                  //TT-TAI [day]
            weatherData_.temperature,           //ambient temperature [K]
            weatherData_.pressure,              //pressure [mb=hPa]
            weatherData_.humidity,              //relative humidity [0-1]
            weatherData_.troposphereLapse,      //tropospheric lapse rate [K per meter]
            telescopeContext_.opticalParameters.refWaveLen,    //reference wavelength [micrometers]
            trueGeographicalParameters_.trueLon,               //telescope longitude true
            trueGeographicalParameters_.trueLat,               //telescope latitude true
            //Output
            &slowCtx_.refTAI,               //reference epoch [TAI MDJ]
            &slowCtx_.refLAST,              //LAST at reference epoch [radians]
            &slowCtx_.refTT,                //TT at reference epoch [MJD]
            &slowCtx_.refTTJ,               //TT at reference epoch [Julian Epoch]
            slowCtx_.amprms,                //target independent MAP parameters
            &slowCtx_.refrA,                //target refraction constant
            &slowCtx_.refrB                 //tan refraction constant
    );

    //todo for the moment we use this.

    double delta = taiMjd_ - slowCtx_.refTAI;
    sideralTime_ = slowCtx_.refLAST + delta*STRPD;

    /*
    std::cout << "slowCtx_.refTAI " << slowCtx_.refTAI << std::endl;
    std::cout << "slowCtx_.refLAST " << slowCtx_.refLAST << std::endl;
    std::cout << "slowCtx_.refTT " << slowCtx_.refTT << std::endl;
    std::cout << "slowCtx_.refTTJ " << slowCtx_.refTTJ << std::endl;
    std::cout << "slowCtx_.refrA " << slowCtx_.refrA << std::endl;
    std::cout << "slowCtx_.refrB " << slowCtx_.refrB << std::endl;
    std::cout << "----------------------------VirtualTelescope::slowUpdate END" << std::endl;
    */

}

void vt::main::VirtualTelescope::mediumUpdate() {

    /*
    std::cout << "----------------------------VirtualTelescope::mediumUpdate" << std::endl;
    std::cout << std::setprecision(15) << std::endl;
    std::cout << "taiMjd_ " << taiMjd_<< std::endl;
    std::cout << "mountType " << telescopeParams_.mountType << std::endl;
    */
    double aux[3] = {0.0,0.0,0.0};

    tcsMedium (
            //Input
            //Current Time
            taiMjd_,                                //time (TAI Julian Data, JD-2400000.5) [seconds] //todo review this value differnt in maedium (comment)
            //Pointing Model
            telescopeContext_.pointModelParameters.maxTerms,                //maximum number of terms in model
            telescopeContext_.pointModelParameters.model,                   //term number fo current model (0 = end)
            telescopeContext_.pointModelParameters.coefValues,              //coefficient values
            telescopeContext_.pointModelParameters.numLocalTerms,           //number of local terms
            telescopeContext_.pointModelParameters.numExplTerms,            //number of terms implemented explicitly (local + standard)
            telescopeContext_.pointModelParameters.numTerms,                //number of terms available currently (local + standard + generic)
            telescopeContext_.pointModelParameters.coefNames,               //coefficient names (local + standard + generic)
            telescopeContext_.pointModelParameters.coefFormat,              //format of generic terms added to coeffn
            //Mount
            telescopeContext_.opticalParameters.mountType,             //mount type
            azElToNominalMount,    //rotation matrix [Az/El] to nominal mount
            mountDemand_.demandedRollTarget_,   //demanded mount roll (radians, right handed)
            mountDemand_.demandedPitchTarget_,  //demanded mount pitch (radians)
            false,              //TRUE = "below the pole"
            aux,                   //auxiliary readings
            //Frames
            target_.frame,                          //mount frame type
            target_.equinox,                        //mount frame equinox
            target_.waveLen,                        //mount wavelength
            pointOrig_.frame,                       //rotator orientation frame type
            pointOrig_.equinox,                     //rotator orientation frame equinox
            pointOrig_.waveLen,                     //rotator orientation wavelength
            //Target
            target_.coordinates ,                    //target coordinates
            //Time-scales
            slowCtx_.refTAI,                        //raw clock time at reference epoch
            slowCtx_.refLAST,                       //LAST at reference epoch (radians)
            slowCtx_.refTTJ,                        //TT at reference epoch (Julian epoch)
            //Refraction
            weatherData_.temperature,                   //ambient temperature [K]
            weatherData_.pressure,                      //pressure [mb=hPa]
            weatherData_.humidity,                      //relative humidity [0-1]
            weatherData_.troposphereLapse,              //tropospheric lapse rate [K per meter]
            telescopeContext_.opticalParameters.refWaveLen,            //reference wavelength [micrometers]
            slowCtx_.refrA,                         //tan refraction constant
            slowCtx_.refrB,                         //tan refraction constant
            NULL,                                   //optional refraction function
            //Site
            telescopeContext_.geographicalParameters.height,                        //telescope height above sea level (meters)
            trueGeographicalParameters_.trueLat,                       //telescope true latitude (true)
            trueGeographicalParameters_.diurnalAberration,             //diurnal aberration (radians)
            //Mean to apparent
            slowCtx_.amprms,                        //target-independent MAP parameters
            //Output
            &mediumCtx_.ia,                         //roll zero point
            &mediumCtx_.ib,                         //pitch zero point
            &mediumCtx_.np,                         //mount axes nonperpendicularity
            &mediumCtx_.xt,                         //telescope vector, x-component
            &mediumCtx_.yt,                         //telescope vector, y-component
            &mediumCtx_.zt,                         //telescope vector, z-component
            mediumCtx_.azElToMount,                 //rotation matrix, [Az,El] to mount
            mediumCtx_.mountSPM1,                   //SPM #1, mount
            mediumCtx_.mountSPM1_i,                 //inverse SPM #1, mount
            mediumCtx_.mountSPM2,                   //SPM #2, mount
            mediumCtx_.mountSPM2_i,                 //inverse SPM #2, mount
            mediumCtx_.rotatorSPM1,                 //SPM #1, rotator
            mediumCtx_.rotatorSPM1_i,               //inverse SPM #1, rotator
            mediumCtx_.rotatorSPM2,                 //SPM #2, rotator
            mediumCtx_.rotatorSPM2_i                //inverse SPM #2, rotator
    );

    /*
    std::cout << mediumCtx_.ib<< std::endl;
    std::cout << mediumCtx_.np<< std::endl;
    std::cout << mediumCtx_.xt<< std::endl;
    std::cout << mediumCtx_.yt<< std::endl;
    std::cout << mediumCtx_.zt<< std::endl;
    std::cout << mediumCtx_.azElToMount[0][0]<< std::endl;
    std::cout << mediumCtx_.mountSPM1[0][0]<< std::endl;
    std::cout << mediumCtx_.mountSPM1_i[0][0]<< std::endl;
    std::cout << mediumCtx_.mountSPM2[0][0]<< std::endl;
    std::cout << mediumCtx_.mountSPM2_i[0][0]<< std::endl;
    std::cout << mediumCtx_.rotatorSPM1[0][0]<< std::endl;
    std::cout << mediumCtx_.rotatorSPM1_i[0][0]<< std::endl;
    std::cout << mediumCtx_.rotatorSPM2[0][0]<< std::endl;
    std::cout << mediumCtx_.rotatorSPM2_i[0][0]<< std::endl;

    std::cout << "----------------------------VirtualTelescope::slowUpdate END" << std::endl;
    */
}

void vt::main::VirtualTelescope::updateWeatherData(const WeatherData &weatherData) {
    weatherData_ = weatherData;
}

void vt::main::VirtualTelescope::updateMountDemand(const vt::main::MountDemand &mountDemand) {
    mountDemand_ = mountDemand;
}

void vt::main::VirtualTelescope::updateTaiMjd(double taiMjd) {
    taiMjd_ = taiMjd;
}

void vt::main::VirtualTelescope::updateTarget(const vt::main::Target &target) {
    target_ = target;
}

void vt::main::VirtualTelescope::updatePointOriging(const vt::main::Target &pointOrig) {
    pointOrig_ = pointOrig;
}

double vt::main::VirtualTelescope::getSideralTime() const {
    return sideralTime_;
}
