//
// Created by mhuertas on 06/11/2021.
//

#include "VirtualTelescope.h"

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
}

void vt::main::VirtualTelescope::slowUpdate() {

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
}

void vt::main::VirtualTelescope::mediumUpdate() {

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
            new double[2] , //todo const_cast<double*>(target_->coords().c_ptr()), //target coordinates, tcspk needs double[2]
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
}

void vt::main::VirtualTelescope::vtSkyToEnc(double sky_roll, double sky_pitch,
                                            double po_x, double po_y,
                                            double pred_roll, double pred_pitch, double pred_rma,
                                            double tai,
                                            double& enc_roll, double& enc_pitch, double& enc_rma) const {

    double aimvect[3];
    double newroll1, newpitch1;
    double newroll2, newpitch2;
    int retval;

    tcsTrack (
            sky_roll,
            sky_pitch,
            mediumCtx_.mountSPM1,
            target_.frame,
            //todo sin_st, cos_st,
            0.0,0.0,
            mediumCtx_.mountSPM2,
            pointOrig_->focalStation(),
            pred_rma,
            pred_roll,
            pred_pitch,
            //todo scaled_poX, scaled_poY,
            0.0,0.0,
            mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
            mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
            //todo pointModel_->guidingA, pointModel_->guidingB,
            0.0,0.0,
            0, //pole avoidance, set to 0 because we want to convert coordinates, not generate demands
            &aimvect[0], &aimvect[1], &aimvect[2],
            &newroll1, &newpitch1, //first solution pair
            &newroll2, &newpitch2, //second solution pair
            &retval
    );

    tcsRotator (
            aimvect[0], aimvect[1], aimvect[2],
            pointOrig_->focalStation(),
            pred_rma,
            0, //pole avoidance, set to 0 because we want to convert coordinates, not generate demands
            enc_roll, enc_pitch,
            //todo scaled_poX, scaled_poY,
            0.0,0.0,
            mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
            mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
            //todo pointModel_->guidingA, pointModel_->guidingB,
            0.0,0.0,
            0.0, 1.0, //we handle the instrument alignment angle in the DoF class
            mediumCtx_.rotatorSPM1_i,
            pointOrig_.frame,
            //todo sin_st, cos_st,
            //sin_st, cos_st,
            mediumCtx_.rotatorSPM2_i,
            pointOrig_->IPA(),
            &enc_rma,
            &retval
    );
}

void vt::main::VirtualTelescope::vtEncToSky(double mount_roll, double mount_pitch, double rma,
                                            double po_x, double po_y,
                                            double tai,
                                            double &sky_longitude, double &sky_latitude) const {

    //make the transformation
    tcsVTsky (
            mount_roll,
            mount_pitch,
            pointOrig_->focalStation(),
            rma,
            //todo scaled_poX, scaled_poY,
            0.0,0.0,
            mediumCtx_.mountSPM1_i,
            target_.frame,
            //todo sin_st,cos_st,
            0.0,0.0,
            mediumCtx_.mountSPM2_i,
            mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
            mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
            pointModel_->guidingA, pointModel_->guidingB,
            &sky_longitude,
            &sky_latitude
    );
}

void vt::main::VirtualTelescope::vtSkyToPointOrig(double sky_longitude, double sky_latitude, double mount_roll,
                                                  double mount_pitch, double rma, double tai, double &po_x,
                                                  double &po_y) const {

    int retval;
    double scaled_poX, scaled_poY, distorted_poX, distorted_poY;

    tcsVTxy (
            sky_longitude,
            sky_latitude,
            mediumCtx_->mountSPM1,
            target_->frame(),
            //todo sin_st,cos_st,
            0.0,0.0,
            mediumCtx_.mountSPM2,
            pointOrig_->focalStation(),
            rma,
            mount_roll,
            mount_pitch,
            mediumCtx_.ia, mediumCtx_.ib, mediumCtx_.np,
            mediumCtx_.xt, mediumCtx_.yt, mediumCtx_.zt,
            pointModel_->guidingA, pointModel_->guidingB,
            &scaled_poX,
            &scaled_poY,
            &retval
    );
}
