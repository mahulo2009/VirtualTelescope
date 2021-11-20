//
// Created by mhuertas on 06/11/2021.
//

#include "VirtualTelescope.h"

#include <iostream>
#include "slalib.h"

vt::main::VirtualTelescope::VirtualTelescope(const GeoDataParams &geoData,
                                             const TelescopeParams & params):
        geoData_(geoData),
        telescopeParams_(params)
{
}

void vt::main::VirtualTelescope::init() {

    //call to the tcsInit2 function
    int ret=tcsInit2 (
              //Input
              geoData_.meanLon,                        //site mean east longitude [radians]
              geoData_.meanLat,                        //site mean geodetic latitude [radians]
              iers_.polarMotionX,                      //polar-motion x angle [radians]
              iers_.polarMotionY,                      //polar-motion y angle [radians]
              geoData_.height,                         //site elevation, meters above sea-level. [meter]
              telescopeParams_.mountType,              //mount type altazimutal.
              telescopeParams_.gimbalZ,                //0 for altazimutal
              telescopeParams_.gimbalY,                //0 for altazimutal
              telescopeParams_.gimbalX,                //0 for altazimutal
              //Output
              telescopeParams_.azElToNominalMount,     //rotation matrix, [Az,El] to nominal mount.
              &geoData_.trueLon,                       //telescope longitude true [radians]
              &geoData_.trueLat,                       //telescope latitude true [radians]
              &geoData_.axisDistanceAu,                //distance from spin axis [AU]
              &geoData_.equatorDistanceAu,             //distance from equator [AU]
              &geoData_.axisDistanceKm,                //distance from spin axis [km]
              &geoData_.equatorDistanceKm,             //distance from equator [km]
              &geoData_.diurnalAberration);            //diurnal aberration [radians]

    std::cout << "----------------------------VirtualTelescope::init" << std::endl;

    std::cout << "oK "<< ret << std::endl;

    for (auto &row : telescopeParams_.azElToNominalMount) {
        for (auto x : row) {
            std::cout <<  x << "\t";
        }
        std::cout << std::endl;
    }

    std::cout << "geoData_.trueLon " << geoData_.trueLon << std::endl;
    std::cout << "geoData_.trueLat " << geoData_.trueLat<< std::endl;
    std::cout << "geoData_.axisDistanceAu "<< geoData_.axisDistanceAu<< std::endl;
    std::cout << "geoData_.axisDistanceKm "<< geoData_.axisDistanceKm<< std::endl;
    std::cout << "geoData_.equatorDistanceAu " << geoData_.equatorDistanceAu<< std::endl;
    std::cout << "geoData_.equatorDistanceKm " << geoData_.equatorDistanceKm<< std::endl;
    std::cout << "geoData_.diurnalAberration " << geoData_.diurnalAberration<< std::endl;
    std::cout << "----------------------------VirtualTelescope::init END" << std::endl;

}

void vt::main::VirtualTelescope::slowUpdate() {

    std::cout << "----------------------------" << std::endl;
    std::cout << "iers_.ut1ToUtc " << iers_.ut1ToUtc << std::endl;
    std::cout << "iers_.taiToUtc " << iers_.taiToUtc << std::endl;
    std::cout << "iers_.ttToTai " << iers_.ttToTai << std::endl;
    std::cout << "----------------------------" << std::endl;

    tcsSlow (
            //Input
            taiMjd_,                        //time (TAI Modified Julian Data, JD-2000000.5) [seconds] //todo review this value differnt in maedium (comment)
            geoData_.height,                //site elevation, meters above sea-level [meter].
            iers_.ut1ToUtc,                 //current UT1-UTC [day]
            iers_.taiToUtc,                 //TAI-UTC [day]
            iers_.ttToTai,                  //TT-TAI [day]
            weather_.temperature,           //ambient temperature [K]
            weather_.pressure,              //pressure [mb=hPa]
            weather_.humidity,              //relative humidity [0-1]
            weather_.troposphereLapse,      //tropospheric lapse rate [K per meter]
            telescopeParams_.refWaveLen,    //reference wavelength [micrometers]
            geoData_.trueLon,               //telescope longitude true
            geoData_.trueLat,               //telescope latitude true
            //Output
            &slowCtx_.refTAI,               //reference epoch [TAI MDJ]
            &slowCtx_.refLAST,              //LAST at reference epoch [radians]
            &slowCtx_.refTT,                //TT at reference epoch [MJD]
            &slowCtx_.refTTJ,               //TT at reference epoch [Julian Epoch]
            slowCtx_.amprms,                //target independent MAP parameters
            &slowCtx_.refrA,                //target refraction constant
            &slowCtx_.refrB                 //tan refraction constant
    );

    std::cout << "----------------------------" << std::endl;
    std::cout << "slowCtx_.refTAI " << slowCtx_.refTAI << std::endl;
    std::cout << "slowCtx_.refLAST " << slowCtx_.refLAST << std::endl;
    std::cout << "slowCtx_.refTT " << slowCtx_.refTT << std::endl;
    std::cout << "slowCtx_.refTTJ " << slowCtx_.refTTJ << std::endl;
    std::cout << "slowCtx_.refrA " << slowCtx_.refrA << std::endl;
    std::cout << "slowCtx_.refrB " << slowCtx_.refrB << std::endl;
    std::cout << "----------------------------" << std::endl;


}

void vt::main::VirtualTelescope::mediumUpdate() {

    std::cout << "----------------------------mediumUpdate" << std::endl;
    std::cout << "pointingModel_.coefValues " << pointingModel_.coefValues[0] << std::endl;

    //todo
    double targetCoordenates_[] = {0, 1.55334} ;

    int ret= tcsMedium (
            //Input
            //Current Time
            taiMjd_,                                //time (TAI Julian Data, JD-2400000.5) [seconds] //todo review this value differnt in maedium (comment)
            //Pointing Model
            pointingModel_.maxTerms,                //maximum number of terms in model
            pointingModel_.model,                   //term number fo current model (0 = end)
            pointingModel_.coefValues,              //coefficient values
            pointingModel_.numLocalTerms,           //number of local terms
            pointingModel_.numExplTerms,            //number of terms implemented explicitly (local + standard)
            pointingModel_.numTerms,                //number of terms available currently (local + standard + generic)
            pointingModel_.coefNames,               //coefficient names (local + standard + generic)
            pointingModel_.coefFormat,              //format of generic terms added to coeffn
            //Mount
            telescopeParams_.mountType,             //mount type
            telescopeParams_.azElToNominalMount,    //rotation matrix [Az/El] to nominal mount
            telescopeStatus_.demandedRollTarget_,   //demanded mount roll (radians, right handed)
            telescopeStatus_.demandedPitchTarget_,  //demanded mount pitch (radians)
            telescopeParams_.belowPole,             //TRUE = "below the pole"
            telescopeParams_.aux,                   //auxiliary readings
            //Frames
            target_.frame,                          //mount frame type
            target_.equinox,                        //mount frame equinox
            target_.waveLen,                        //mount wavelength
            pointOrig_.frame,                       //rotator orientation frame type
            pointOrig_.equinox,                     //rotator orientation frame equinox
            pointOrig_.waveLen,                     //rotator orientation wavelength
            //Target
            targetCoordenates_ ,                    //target coordinates
            //Time-scales
            slowCtx_.refTAI,                        //raw clock time at reference epoch
            slowCtx_.refLAST,                       //LAST at reference epoch (radians)
            slowCtx_.refTTJ,                        //TT at reference epoch (Julian epoch)
            //Refraction
            weather_.temperature,                   //ambient temperature [K]
            weather_.pressure,                      //pressure [mb=hPa]
            weather_.humidity,                      //relative humidity [0-1]
            weather_.troposphereLapse,              //tropospheric lapse rate [K per meter]
            telescopeParams_.refWaveLen,            //reference wavelength [micrometers]
            slowCtx_.refrA,                         //tan refraction constant
            slowCtx_.refrB,                         //tan refraction constant
            NULL,                                   //optional refraction function
            //Site
            geoData_.height,                        //telescope height above sea level (meters)
            geoData_.trueLat,                       //telescope true latitude (true)
            geoData_.diurnalAberration,             //diurnal aberration (radians)
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

    std::cout << "oK "<< ret << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout << "telescopeStatus_.demandedRollTarget_ " << telescopeStatus_.demandedRollTarget_ << std::endl;
    std::cout << "telescopeStatus_.demandedPitchTarget_ " << telescopeStatus_.demandedPitchTarget_ << std::endl;

    std::cout << "mediumCtx_.ia " << mediumCtx_.ia << std::endl;
    std::cout << "----------------------------" << std::endl;

}

void vt::main::VirtualTelescope::vtSkyToEnc (double sky_roll,
                                             double sky_pitch,
                                             double po_x,
                                             double po_y,
                                             double& enc_roll,
                                             double& enc_pitch,
                                             double& enc_rma,
                                             int max_iterations) const {

    double aimvect[3];
    double newroll1, newpitch1;
    double newroll2, newpitch2;

    int nIter=0;
    bool solutionConverged = false;
    double newroll, newpitch, newrma;

    while (!solutionConverged && nIter<max_iterations)
    {
        int retval;
        tcsTrack (
                //Input
                sky_roll,                                            //target "roll" coordinate
                sky_pitch,                                           //target "pitch" coordinate
                const_cast<double (*)[3]>(mediumCtx_.mountSPM1),     //SPM #1, mount
                target_.frame,                                       //reference frame for the target
                sin(sideralTime_),                                   //sine of sidereal time
                cos(sideralTime_),                                   //cosine of sidereal time
                const_cast<double (*)[3]>(mediumCtx_.mountSPM2),     //SPM #2, mount
                pointOrig_.focalStation,                             //rotator location
                newrma,                                              //predicted rotator mechanical angle
                newroll,                                             //predicted roll
                newpitch,                                            //predicted pitch
                po_x,                                                //pointing origin x (in focal lengths) //todo scale
                po_y,                                                //pointing origin y (in focal lengths) //todo scale
                mediumCtx_.ia,                                       //roll zero point
                mediumCtx_.ib,                                       //pitch zero point
                mediumCtx_.np,                                       //mount axes nonperpendicularity
                mediumCtx_.xt,                                       //telescope vector, x-component
                mediumCtx_.yt,                                       //telescope vector, y-component
                mediumCtx_.zt,                                       //telescope vector, z-component
                pointingModel_.guidingA,                             //guiding correction, collimation
                pointingModel_.guidingB,                             //guiding correction, pitch
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

        newroll = telescopeParams_.belowPole ? newroll2 : newroll1;
        newpitch = telescopeParams_.belowPole ? newpitch2 : newpitch1;

        tcsRotator (
                aimvect[0],                 //AIM x-coordinate
                aimvect[1],                 //AIM y-coordinate
                aimvect[2],                 //AIM z-coordinate
                pointOrig_.focalStation,    //rotator location
                newrma,
                0, //pole avoidance, set to 0 because we want to convert coordinates, not generate demands
                enc_roll,
                enc_pitch,
                po_x,
                po_y, //todo scale
                mediumCtx_.ia,
                mediumCtx_.ib,
                mediumCtx_.np,
                mediumCtx_.xt,
                mediumCtx_.yt,
                mediumCtx_.zt,
                pointingModel_.guidingA,
                pointingModel_.guidingB,
                0.0,
                1.0, //we handle the instrument alignment angle in the DoF class
                const_cast<double (*)[3]>(mediumCtx_.rotatorSPM1_i),
                pointOrig_.frame,
                sin(sideralTime_), cos(sideralTime_),
                const_cast<double (*)[3]>(mediumCtx_.rotatorSPM2_i),
                pointOrig_.ipa,
                &enc_rma,
                &retval
        );

        nIter++;
        if (fabs(newroll-enc_roll)<1e-10 && fabs(newpitch-enc_pitch)<1e-10 && fabs(newrma-enc_rma)<1e-10)
            solutionConverged = true;
        else
        {
            enc_roll = newroll;
            enc_pitch = newpitch;
            enc_rma = newrma;
        }
    }

    std::cout << "solutionConverged " << solutionConverged << std::endl;

}

/*
void vt::main::VirtualTelescope::vtEncToSky(double mount_roll, double mount_pitch, double rma,
                                            double po_x, double po_y,
                                            double tai,
                                            double &sky_longitude, double &sky_latitude) const {

    double scaled_poX, scaled_poY;

    //convert pointing origin from millimeters to radians in the tangent plane
    convertFocalPlaneMmToRad_ (po_x, po_y,
                               scaled_poX, scaled_poY);

    //correct for the focal plane distortion
    removeFocalPlaneDistortion_ (scaled_poX, scaled_poY, scaled_poX, scaled_poY);


    //make the transformation
    tcsVTsky (
            mount_roll,
            mount_pitch,
            pointOrig_.focalStation,
            rma,
            scaled_poX, scaled_poY,
            const_cast<double (*)[3]>(mediumCtx_.mountSPM1_i),
            target_.frame,
            sin(sideralTime_), cos(sideralTime_),
            const_cast<double (*)[3]>(mediumCtx_.mountSPM2_i),
            mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
            mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
            0.0,0.0,
            &sky_longitude,
            &sky_latitude
    );
}

void vt::main::VirtualTelescope::vtSkyToPointOrig(double sky_longitude, double sky_latitude, double mount_roll,
                                                  double mount_pitch, double rma, double tai, double &po_x,
                                                  double &po_y) const {

    double scaled_poX, scaled_poY;

    int retval;
    tcsVTxy (
            sky_longitude,
            sky_latitude,
            const_cast<double (*)[3]>(mediumCtx_.mountSPM1),
            target_.frame,
            sin(sideralTime_), cos(sideralTime_),
            const_cast<double (*)[3]>(mediumCtx_.mountSPM2),
            pointOrig_.focalStation,
            rma,
            mount_roll,
            mount_pitch,
            mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
            mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
            pointingModel_.guidingA, pointingModel_.guidingB,
            &scaled_poX,
            &scaled_poY,
            &retval
    );
}
*/

void
vt::main::VirtualTelescope::convertFocalPlaneMmToRad_(double mm_x, double mm_y, double &rad_x, double &rad_y) const {
    rad_x = mm_x / telescopeParams_.plate_scale;
    rad_y = mm_y / telescopeParams_.plate_scale;
}

void
vt::main::VirtualTelescope::removeFocalPlaneDistortion_(double x0, double y0, double &x, double &y) const {
    x = x0;
    y = y0;
    slaUnpcd (telescopeParams_.distortionCoefficient, &x, &y);
}

void vt::main::VirtualTelescope::setIers(const vt::main::IERS &iers) {
    iers_ = iers;
}

void vt::main::VirtualTelescope::setWeather(const vt::main::WeatherStation &weather) {
    weather_ = weather;
}

void vt::main::VirtualTelescope::setPointingModel(const PointModel &pointingModel) {
    pointingModel_ = pointingModel;
}

void vt::main::VirtualTelescope::setTelescopeStatus(const vt::main::TelescopeStatus &telescopeStatus) {
    telescopeStatus_ = telescopeStatus;
}

void vt::main::VirtualTelescope::setTaiMjd(double taiMjd) {
    taiMjd_ = taiMjd;
}

void vt::main::VirtualTelescope::setTarget(const vt::main::Target &target) {
    target_ = target;
}

void vt::main::VirtualTelescope::setPointOrig(const vt::main::Target &pointOrig) {
    pointOrig_ = pointOrig;
}

void vt::main::VirtualTelescope::setSideralTime(double sideralTime) {
    sideralTime_ = sideralTime;
}
