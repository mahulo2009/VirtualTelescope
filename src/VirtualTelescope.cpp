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
    tcsInit2 (geoData_.meanLon, geoData_.meanLat,
              iers_.polarMotionX, iers_.polarMotionY,
              geoData_.height,
              telescopeParams_.mountType,
              telescopeParams_.gimbalZ, telescopeParams_.gimbalY, telescopeParams_.gimbalX,
              telescopeParams_.azElToNominalMount,
              &geoData_.trueLon, &geoData_.trueLat,
              &geoData_.axisDistanceAu, &geoData_.equatorDistanceAu,
              &geoData_.axisDistanceKm, &geoData_.equatorDistanceKm,
              &geoData_.diurnalAberration);

    std::cout << "----------------------------" << std::endl;
    std::cout << "geoData_.trueLon " << geoData_.trueLon << std::endl;
    std::cout << "geoData_.trueLat " << geoData_.trueLat<< std::endl;
    std::cout << "geoData_.axisDistanceAu "<< geoData_.axisDistanceAu<< std::endl;
    std::cout << "geoData_.axisDistanceKm "<< geoData_.axisDistanceKm<< std::endl;
    std::cout << "geoData_.equatorDistanceAu " << geoData_.equatorDistanceAu<< std::endl;
    std::cout << "geoData_.equatorDistanceKm " << geoData_.equatorDistanceKm<< std::endl;
    std::cout << "geoData_.diurnalAberration " << geoData_.diurnalAberration<< std::endl;
    std::cout << "----------------------------" << std::endl;

}

void vt::main::VirtualTelescope::slowUpdate() {

    std::cout << "----------------------------" << std::endl;
    std::cout << "iers_.ut1ToUtc " << iers_.ut1ToUtc << std::endl;
    std::cout << "iers_.taiToUtc " << iers_.taiToUtc << std::endl;
    std::cout << "iers_.ttToTai " << iers_.ttToTai << std::endl;
    std::cout << "----------------------------" << std::endl;

    tcsSlow (
            //input parameters
            taiMjd_,  //atomic time
            geoData_.height, //height above sea level
            iers_.ut1ToUtc, //IERS bulletin, UT1-UTC
            iers_.taiToUtc, //idem, TAI-UTC
            iers_.ttToTai,  //idem, TT-TAI
            weather_.temperature, //refraction data
            weather_.pressure, //refraction data
            weather_.humidity, //refraction data
            weather_.troposphereLapse, //refraction data, tropospheric lapse rate
            telescopeParams_.refWaveLen, //telescope reference wave length (in general, not current target)
            geoData_.trueLon, //true site longitude, corrected from polar motion
            geoData_.trueLat, //true site latitude, corrected from polar motion
            //output parameters
            &slowCtx_.refTAI, //TAI at reference epoch, for interpolations
            &slowCtx_.refLAST, //local aparent sidereal time at refTAI
            &slowCtx_.refTT, //Terrestrial Time at refTAI
            &slowCtx_.refTTJ, //Julian TT at refTAI
            slowCtx_.amprms, //target independent astrometric transformation values
            &slowCtx_.refrA, //refraction model A coefficient
            &slowCtx_.refrB  //refraction model B cofficient
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
    tcsMedium (
            //input parameters
            taiMjd_, //atomic time
            pointingModel_.maxTerms,  //pointing model, max terms
            pointingModel_.model,  //idem, model coefficient list
            pointingModel_.coefValues, //idem, coefficient values
            pointingModel_.numLocalTerms, //idem, number of local terms in the model
            pointingModel_.numExplTerms,  //idem, number of explicit terms in the model
            pointingModel_.numTerms,      //idem, total number of terms
            pointingModel_.coefNames, //idem, coefficient names
            pointingModel_.coefFormat, //idem, coefficient format
            telescopeParams_.mountType, //mount params, mount type
            telescopeParams_.azElToNominalMount, //idem, nominal rotation matrix from Az/El to mount
            telescopeStatus_.demandedRollTarget_, //idem, current roll demand
            telescopeStatus_.demandedPitchTarget_, //idem, current pitch demand
            telescopeParams_.belowPole, //idem, below/above pole selector
            telescopeParams_.aux, //idem, auxiliary readings (??)
            target_.frame,//target, reference frame info
            target_.equinox, //target
            target_.waveLen, //target
            pointOrig_.frame, //rotator orientation frame
            pointOrig_.equinox,
            pointOrig_.waveLen,
            targetCoordenates_ ,
            slowCtx_.refTAI, //reference time
            slowCtx_.refLAST, //local sidereal time at reference time
            slowCtx_.refTTJ, //Julian TT at reference time
            weather_.temperature, //refraction data
            weather_.pressure, //refraction data
            weather_.humidity, //refraction data
            weather_.troposphereLapse, //refraction data, tropospheric lapse rate
            telescopeParams_.refWaveLen, //generic reference wavelength (not the target's one)
            slowCtx_.refrA, //refraction data, A coeff
            slowCtx_.refrB, //refraction data, B coeff
            NULL, //NULL, because we use default refraction model
            geoData_.height, //site data
            geoData_.trueLat,
            geoData_.diurnalAberration,
            slowCtx_.amprms, //target independent transformation values
            //output
            &mediumCtx_.ia, //corrections to pointing model
            &mediumCtx_.ib,
            &mediumCtx_.np,
            &mediumCtx_.xt, //telescope vector, 3 components
            &mediumCtx_.yt,
            &mediumCtx_.zt,
            mediumCtx_.azElToMount, //true azimuth/elevation to mount rotation matrix
            mediumCtx_.mountSPM1, //Sky Patch matrices and inverses
            mediumCtx_.mountSPM1_i,
            mediumCtx_.mountSPM2,
            mediumCtx_.mountSPM2_i,
            mediumCtx_.rotatorSPM1,
            mediumCtx_.rotatorSPM1_i,
            mediumCtx_.rotatorSPM2,
            mediumCtx_.rotatorSPM2_i
    );

    std::cout << "----------------------------" << std::endl;
    std::cout << "telescopeStatus_.demandedRollTarget_ " << telescopeStatus_.demandedRollTarget_ << std::endl;
    std::cout << "telescopeStatus_.demandedPitchTarget_ " << telescopeStatus_.demandedPitchTarget_ << std::endl;

    std::cout << "mediumCtx_.ia " << mediumCtx_.ia << std::endl;
    std::cout << "----------------------------" << std::endl;

}

void vt::main::VirtualTelescope::vtSkyToEnc (double sky_roll, double sky_pitch,
                                             double po_x, double po_y,
                                             double& enc_roll, double& enc_pitch, double& enc_rma,
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
                sky_roll,
                sky_pitch,
                const_cast<double (*)[3]>(mediumCtx_.mountSPM1),
                target_.frame,
                sin(sideralTime_), cos(sideralTime_),
                const_cast<double (*)[3]>(mediumCtx_.mountSPM2),
                pointOrig_.focalStation,
                newrma,
                newroll,
                newpitch,
                po_x, po_y, //todo scale
                mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
                mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
                pointingModel_.guidingA, pointingModel_.guidingB,
                0, //pole avoidance, set to 0 because we want to convert coordinates, not generate demands
                &aimvect[0], &aimvect[1], &aimvect[2],
                &newroll1, &newpitch1, //first solution pair
                &newroll2, &newpitch2, //second solution pair
                &retval
        );

        newroll = telescopeParams_.belowPole ? newroll2 : newroll1;
        newpitch = telescopeParams_.belowPole ? newpitch2 : newpitch1;

        tcsRotator (
                aimvect[0], aimvect[1], aimvect[2],
                pointOrig_.focalStation,
                newrma,
                0, //pole avoidance, set to 0 because we want to convert coordinates, not generate demands
                enc_roll, enc_pitch,
                po_x, po_y, //todo scale
                mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
                mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
                pointingModel_.guidingA, pointingModel_.guidingB,
                0.0, 1.0, //we handle the instrument alignment angle in the DoF class
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
