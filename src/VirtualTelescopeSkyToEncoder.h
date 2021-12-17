//
// Created by mhuertas on 17/12/2021.
//

#ifndef VIRTUALTELESCOPE_VIRTUALTELESCOPESKYTOENCODER_H
#define VIRTUALTELESCOPE_VIRTUALTELESCOPESKYTOENCODER_H

#include "VirtualTelescope.h"

namespace  vt {

    namespace main {

        class VirtualTelescopeSkyToEncoder : public VirtualTelescope {

        public:

            VirtualTelescopeSkyToEncoder(const TelescopeContext &telescopeContext);

            virtual void fastUpdate();

            virtual void print();

        private:

            double encoderRoll;

            double encoderPitch;

            double encoderRma;

        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPESKYTOENCODER_H
