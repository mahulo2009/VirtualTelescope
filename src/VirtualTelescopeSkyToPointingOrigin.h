//
// Created by mhuertas on 17/12/2021.
//

#ifndef VIRTUALTELESCOPE_VIRTUALTELESCOPESKYTOPOINTINGORIGIN_H
#define VIRTUALTELESCOPE_VIRTUALTELESCOPESKYTOPOINTINGORIGIN_H

#include "VirtualTelescope.h"

namespace  vt {

    namespace main {

        class VirtualTelescopeSkyToPointingOrigin : public VirtualTelescope {

        public:

            VirtualTelescopeSkyToPointingOrigin(const TelescopeContext &telescopeContext);

            virtual void fastUpdate();

            virtual void print();

        private:

            double poX;

            double poY;

        };
    }
}

#endif //VIRTUALTELESCOPE_VIRTUALTELESCOPESKYTOPOINTINGORIGIN_H
