//
// Created by mhuertas on 17/12/2021.
//

#ifndef VIRTUALTELESCOPE_VIRTUALTELESCOPEENCODERTOSKY_H
#define VIRTUALTELESCOPE_VIRTUALTELESCOPEENCODERTOSKY_H

#include "VirtualTelescope.h"
namespace  vt {

    namespace main {

        class VirtualTelescopeEncoderToSky : public VirtualTelescope {

        public:
            VirtualTelescopeEncoderToSky(const TelescopeContext &telescopeContext);

            virtual void fastUpdate();

            virtual void print();

        private:

            double sky_longitude;

            double sky_latitude;

        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPEENCODERTOSKY_H
