//
// Created by mhuertas on 06/11/2021.
//

#ifndef VIRTUALTELESCOPE_VIRTUALTELESCOPE_H
#define VIRTUALTELESCOPE_VIRTUALTELESCOPE_H

//todo
#define AMPRMS_SZ  21

#include "tcs.h"
#include "PointModel.h"

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
            double distortionCoefficient;
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
            double temperature;
            double pressure;
            double humidity;
            double troposphereLapse;
        };

        struct IERS {
            double polarMotionX;
            double polarMotionY;
            double ut1ToUtc;
            double taiToUtc;
            double ttToTai;
        };

        struct Target {
            FRAMETYPE frame;
            double equinox;
            double waveLen;
            ROTLOC focalStation;
            double ipa;
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

                void setIers(const IERS &iers);
                void setWeather(const WeatherStation &weather);
                void setPointingModel(const PointModel &pointingModel);
                void setTelescopeStatus(const TelescopeStatus &telescopeStatus);
                void setTaiMjd(double taiMjd);
                void setTarget(const Target &target);
                void setPointOrig(const Target &pointOrig);
                void targetCoordenates(double targetCoordenates[2]);

                void vtSkyToEnc (double sky_roll, double sky_pitch,
                                    double po_x, double po_y,
                                    double& enc_roll, double& enc_pitch, double& enc_rma,
                                    int max_iterations) const;

                void vtEncToSky (double mount_roll,
                                     double mount_pitch,
                                     double rma,
                                     double po_x,
                                     double po_y,
                                     double& sky_longitude,
                                     double& sky_latitude) const;

                void vtSkyToPointOrig (double sky_longitude,
                                           double sky_latitude,
                                           double mount_roll,
                                           double mount_pitch,
                                           double rma,
                                           double& po_x,
                                           double& po_y) const;



        private:

            //Params
            GeoDataParams geoData_;
            TelescopeParams telescopeParams_;

            //Context
            TargetIndependentContext slowCtx_;
            TargetDependentContext mediumCtx_;

            //Variables
            double taiMjd_;
            double sideralTime_;
        public:
            double getSideralTime() const;

        private:

            IERS iers_;
            WeatherStation weather_;
            PointModel pointingModel_;
            Target target_,pointOrig_;
            TelescopeStatus telescopeStatus_;

            double targetCoordenates_[2];

        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPE_H