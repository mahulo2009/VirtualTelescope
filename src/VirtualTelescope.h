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
/*
        struct PointModel
        {
            int maxTerms;
            int ntRoom;
            int model[100];
            int numLocalTerms;
            int numExplTerms;
            int numTerms;
            char coefNames[100][9];
            char coefFormat[100][9];
            double coefValues[100];
            double guidingA;
            double guidingB;
        };
*/
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

                void vtSkyToEnc (double sky_roll, double sky_pitch,
                                    double po_x, double po_y,
                                    double& enc_roll, double& enc_pitch, double& enc_rma,
                                    int max_iterations) const;

            /*

            void vtSkyToEnc (double sky_roll, double sky_pitch,
                             double po_x, double po_y,
                             double pred_roll, double pred_pitch, double pred_rma,
                             double tai,
                             double& enc_roll, double& enc_pitch, double& enc_rma) const;


            void vtSkyToPointOrig (double sky_longitude, double sky_latitude,
                                                 double mount_roll, double mount_pitch, double rma,
                                                 double tai,
                                                 double& po_x, double& po_y) const;
                                                 */

        private:


            void convertFocalPlaneMmToRad_ (double mm_x, double mm_y,
                                                          double& rad_x, double& rad_y) const;

            void removeFocalPlaneDistortion_ (double x0, double y0,
                                                            double& x, double& y) const;
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
            void setSideralTime(double sideralTime);

        private:
            IERS iers_;
            WeatherStation weather_;
            PointModel pointingModel_;
            Target target_,pointOrig_;
        public:
            void setTarget(const Target &target);

            void setPointOrig(const Target &pointOrig);

        private:
            TelescopeStatus telescopeStatus_;
        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPE_H