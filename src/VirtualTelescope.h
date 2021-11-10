//
// Created by mhuertas on 06/11/2021.
//

#ifndef VIRTUALTELESCOPE_VIRTUALTELESCOPE_H
#define VIRTUALTELESCOPE_VIRTUALTELESCOPE_H

//todo
#define AMPRMS_SZ  21

#include "tcs.h"

namespace  vt {
    namespace  main {

        struct GeoDataParams
        {
            //Input
            double meanLon;
            double meanLat;
            double height;
            //Output
            double trueLon;
            double trueLat;
            double axisDistanceAu;
            double axisDistanceKm;
            double equatorDistanceAu;
            double equatorDistanceKm;
            double diurnalAberration;
        };

        struct TelescopeParams {
            //Input
            double focal_len;
            double plate_scale;
            double distortion;
            double refWaveLen;
            MTYPE mountType;
            double gimbalX;
            double gimbalY;
            double gimbalZ;
            bool belowPole=false;
            double aux[3]; //todo initilize
            //Output
            double azElToNominalMount[3][3];
        };

        struct TargetIndependentContext
        {
            double refTAI;
            double refLAST;
            double refTT;
            double refTTJ;
            double amprms[AMPRMS_SZ];
            double refrA;
            double refrB;
        };

        struct TargetDependentContext
        {
            double refTAI;
            double ia;
            double ib;
            double np;
            double xt;
            double yt;
            double zt;
            double azElToMount[3][3];
            double mountSPM1[3][3];
            double mountSPM1_i[3][3];
            double mountSPM2[3][3];
            double mountSPM2_i[3][3];
            double rotatorSPM1[3][3];
            double rotatorSPM1_i[3][3];
            double rotatorSPM2[3][3];
            double rotatorSPM2_i[3][3];
        };

        struct WeatherStation {
            double temperature; //todo setter.
            double pressure; //todo setter.
            double humidity; //todo setter.
            double troposphereLapse; //todo setter.
        };

        struct IERS {
            double polarMotionX; //todo setter.
            double polarMotionY; //todo setter.
            double ut1ToUtc; //todo setter.
            double taiToUtc; //todo setter.
            double ttToTai; //todo setter
        };

        struct PointModel
        {
            int maxTerms;
            int ntRoom;
            int model[100];
            double coefValues[100];
            int numLocalTerms;
            int numExplTerms;
            int numTerms;
            char coefNames[100][9];
            char coefFormat[100][9];
        };

        struct Target {
            FRAMETYPE frame;
            double equinox;
            double waveLen;
        };

        struct TelescopeStatus {
            double demandedRollTarget_;
            double demandedPitchTarget_;
        };

        class VirtualTelescope {

            public:

                VirtualTelescope(const GeoDataParams &geoData, const TelescopeParams & params);

                virtual void init ();
                virtual void slowUpdate();
                virtual void mediumUpdate();

                void vtSkyToEnc (double sky_roll, double sky_pitch,
                                 double po_x, double po_y,
                                 double pred_roll, double pred_pitch, double pred_rma,
                                 double tai,
                                 double& enc_roll, double& enc_pitch, double& enc_rma) const;

                void vtEncToSky (double mount_roll, double mount_pitch, double rma,
                                               double po_x, double po_y,
                                               double tai,
                                               double& sky_longitude, double& sky_latitude) const;

                void vtSkyToPointOrig (double sky_longitude, double sky_latitude,
                                                     double mount_roll, double mount_pitch, double rma,
                                                     double tai,
                                                     double& po_x, double& po_y) const;

            private:
                //Params
                GeoDataParams geoData_;
                TelescopeParams telescopeParams_;
                //Context
                TargetIndependentContext slowCtx_;
                TargetDependentContext mediumCtx_;
                //Variables
                double taiMjd_;
                IERS iers_;
                WeatherStation weather_;
                PointModel pointingModel_;
                Target target_,pointOrig_;
                TelescopeStatus telescopeStatus_;
        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPE_H