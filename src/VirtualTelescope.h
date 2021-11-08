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

        struct GeoData
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

        struct PointingModel
        {
            static const int nameLength = 9;
            static const int DEF_MAXT = 100;
            static const int DEF_NTROOM = 100;
            int maxTerms;
            int ntRoom;
            //Array1D<int> model; todo
            int numLocalTerms;
            int numExplTerms;
            int numTerms;
            //Array2D<char> coefNames; todo
            //Array2D<char> coefFormat; todo
            //Array1D<double> coefValues; todo
            double guidingA;
            double guidingB;
        };


        class VirtualTelescope {
        public:
            VirtualTelescope(const GeoData &geoData, const TelescopeParams & params);

            virtual void init (const double tai);
            virtual void slowUpdate(double tai,double taiMjd);
            virtual void mediumUpdate(double tai,double taiMjd);

        private:

            GeoData geoData_;
            TelescopeParams telescopeParams_;
            TargetIndependentContext *slowCtx_;
            TargetDependentContext *aux_mediumCtx_;
            PointingModel *pointingModel_;
        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPE_H