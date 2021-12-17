//
// Created by mhuertas on 17/12/2021.
//

#ifndef VIRTUALTELESCOPE_TELESCOPECONTEXT_H
#define VIRTUALTELESCOPE_TELESCOPECONTEXT_H

//todo
#define AMPRMS_SZ  21

#include "tcs.h"
#include "PointModel.h"

namespace  vt {

    namespace  main {

        struct GeographicalParameters {
            double meanLon;
            double meanLat;
            double height;
        };

        struct OpticalParameters {
            double focal_len;
            double plate_scale;
            double distortionCoefficient;
            double refWaveLen;
            MTYPE mountType;
        };

        struct PolarMotionParameters {
            double polarMotionX;
            double polarMotionY;
        };

        struct TimeTransformParameters {
            double ut1ToUtc;
            double taiToUtc;
            double ttToTai;
        };

        struct PointModelParameters {
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

        struct TelescopeContext {
            GeographicalParameters geographicalParameters;
            OpticalParameters opticalParameters;
            PolarMotionParameters polarMotionParameters;
            TimeTransformParameters timeTransformParameters;
            PointModelParameters pointModelParameters;
        };

    }
}


#endif //VIRTUALTELESCOPE_TELESCOPECONTEXT_H
