//
// Created by mhuertas on 06/11/2021.
//

#ifndef VIRTUALTELESCOPE_VIRTUALTELESCOPE_H
#define VIRTUALTELESCOPE_VIRTUALTELESCOPE_H

//todo
#define AMPRMS_SZ  21

#include "TelescopeContext.h"

namespace  vt {

    namespace  main {

        struct TrueGeographicalParameters {
            double trueLon;
            double trueLat;
            double axisDistanceAu;
            double axisDistanceKm;
            double equatorDistanceAu;
            double equatorDistanceKm;
            double diurnalAberration;
        };

        struct WeatherData {
            double temperature;
            double pressure;
            double humidity;
            double troposphereLapse;
        };

        struct MountDemand {
            double demandedRollTarget_;
            double demandedPitchTarget_;
            double demandedRma_;
        };

        struct Target {
            FRAMETYPE frame;
            double equinox;
            double waveLen;
            ROTLOC focalStation;
            double ipa;
            double coordinates[2];
        };

        class VirtualTelescope {

            public:

                VirtualTelescope(const TelescopeContext & telescopeContext);

                virtual void init ();

                virtual void slowUpdate();

                virtual void mediumUpdate();

                virtual void fastUpdate() = 0;

                virtual void print() = 0;

                void updateTaiMjd(double taiMjd);

                void updateWeatherData(const WeatherData &weatherData);

                void updateMountDemand(const MountDemand &mountDemand);

                void updateTarget(const Target &target);

                void updatePointOriging(const Target &pointOrig);

                double getSideralTime() const;

        private:

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

        private:

            TelescopeContext telescopeContext_;

            TrueGeographicalParameters trueGeographicalParameters_;

            double azElToNominalMount[3][3];

            double taiMjd_;

            WeatherData weatherData_;

            TargetIndependentContext slowCtx_;

        protected:

            MountDemand mountDemand_;

            double guidingCorrectionA;

            double guidingCorrectionB;

            Target target_,pointOrig_;

            TargetDependentContext mediumCtx_;

            double sideralTime_;

        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPE_H