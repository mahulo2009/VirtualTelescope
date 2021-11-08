//
// Created by mhuertas on 06/11/2021.
//

#include "VirtualTelescope.h"

vt::main::VirtualTelescope::VirtualTelescope(const GeoData &geoData,
                                             const TelescopeParams & params):
        geoData_(geoData),
        telescopeParams_(params)
{
    slowCtx_ = new TargetIndependentContext;
    aux_mediumCtx_ = new TargetDependentContext;
    pointingModel_ = new PointingModel;
}

void vt::main::VirtualTelescope::init(double tai) {

    //call to the tcsInit2 function
    tcsInit2 (geoData_.meanLon, geoData_.meanLat,
              //iers_->polarMotionX(taiMjd), iers_->polarMotionY(taiMjd), todo
              0,0,
              geoData_.height,
              telescopeParams_.mountType,
              telescopeParams_.gimbalZ, telescopeParams_.gimbalY, telescopeParams_.gimbalX,
              telescopeParams_.azElToNominalMount,
              &geoData_.trueLon, &geoData_.trueLat,
              &geoData_.axisDistanceAu, &geoData_.equatorDistanceAu,
              &geoData_.axisDistanceKm, &geoData_.equatorDistanceKm,
              &geoData_.diurnalAberration);
}

void vt::main::VirtualTelescope::slowUpdate(double tai,double taiMjd) {

    tcsSlow (
            //input parameters
            taiMjd,  //atomic time
            geoData_.height, //height above sea level
            //iers_->ut1_utc(taiMjd), //IERS bulletin, UT1-UTC
            //iers_->tai_utc(), //idem, TAI-UTC
            //iers_->tt_tai(),  //idem, TT-TAI
            //weather_->temperature(),
            //weather_->pressure(),
            //weather_->humidity(),
            //weather_->troposphereLapse(), //tropospheric lapse rate, K per meter
            0, 0, 0, 0, 0, 0, 0, //todo
            telescopeParams_.refWaveLen, //telescope reference wave length (in general, not current target)
            geoData_.trueLon, //true site longitude, corrected from polar motion
            geoData_.trueLat, //true site latitude, corrected from polar motion
            //output parameters
            &slowCtx_->refTAI, //TAI at reference epoch, for interpolations
            &slowCtx_->refLAST, //local aparent sidereal time at refTAI
            &slowCtx_->refTT, //Terrestrial Time at refTAI
            &slowCtx_->refTTJ, //Julian TT at refTAI
            slowCtx_->amprms, //target independent astrometric transformation values
            &slowCtx_->refrA, //refraction model A coefficient
            &slowCtx_->refrB  //refraction model B cofficient
    );

}

void vt::main::VirtualTelescope::mediumUpdate(double tai,double taiMjd) {

    tcsMedium (
            //input parameters
            taiMjd, //atomic time
            pointingModel_->maxTerms,  //pointing model, max terms
            pointingModel_->model.c_ptr(),  //idem, model coefficient list
            pointingModel_->coefValues.c_ptr(), //idem, coefficient values
            pointingModel_->numLocalTerms, //idem, number of local terms in the model
            pointingModel_->numExplTerms,  //idem, number of explicit terms in the model
            pointingModel_->numTerms,      //idem, total number of terms
            (char (*)[9])pointingModel_->coefNames.c_ptr(), //idem, coefficient names
            (char (*)[9])pointingModel_->coefFormat.c_ptr(), //idem, coefficient format
            telescopeParams_.mountType, //mount params, mount type
            telescopeParams_.azElToNominalMount, //idem, nominal rotation matrix from Az/El to mount
            telescopeStatus_->demandedRollTarget(tai), //idem, current roll demand
            telescopeStatus_->demandedPitchTarget(tai), //idem, current pitch demand
            telescopeParams_->belowPole, //idem, below/above pole selector
            telescopeParams_->aux, //idem, auxiliary readings (??)
            target_->frame(), //target, reference frame info
            target_->equinox(), //target
            target_->waveLen(), //target
            pointOrig_->frame(), //rotator orientation frame
            pointOrig_->equinox(),
            pointOrig_->waveLen(),
            const_cast<double*>(target_->coords().c_ptr()), //target coordinates, tcspk needs double[2]
            slowCtx_->refTAI, //reference time
            slowCtx_->refLAST, //local sidereal time at reference time
            slowCtx_->refTTJ, //Julian TT at reference time
            //weather_->temperature(), //refraction data
            //weather_->pressure(), //refraction data
            //weather_->humidity(), //refraction data
            //weather_->troposphereLapse(), //refraction data, tropospheric lapse rate
            0,0,0,0,
            telescopeParams_.refWaveLen, //generic reference wavelength (not the target's one)
            slowCtx_->refrA, //refraction data, A coeff
            slowCtx_->refrB, //refraction data, B coeff
            NULL, //NULL, because we use default refraction model
            geoData_.height, //site data
            geoData_.trueLat,
            geoData_.diurnalAberration,
            slowCtx_->amprms, //target independent transformation values
            //output
            &aux_mediumCtx_->ia, //corrections to pointing model
            &aux_mediumCtx_->ib,
            &aux_mediumCtx_->np,
            &aux_mediumCtx_->xt, //telescope vector, 3 components
            &aux_mediumCtx_->yt,
            &aux_mediumCtx_->zt,
            aux_mediumCtx_->azElToMount, //true azimuth/elevation to mount rotation matrix
            aux_mediumCtx_->mountSPM1, //Sky Patch matrices and inverses
            aux_mediumCtx_->mountSPM1_i,
            aux_mediumCtx_->mountSPM2,
            aux_mediumCtx_->mountSPM2_i,
            aux_mediumCtx_->rotatorSPM1,
            aux_mediumCtx_->rotatorSPM1_i,
            aux_mediumCtx_->rotatorSPM2,
            aux_mediumCtx_->rotatorSPM2_i
    );
}