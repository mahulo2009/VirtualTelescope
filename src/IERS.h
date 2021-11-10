//
// Created by mhuertas on 09/11/2021.
//

#ifndef VIRTUALTELESCOPE_IERS_H
#define VIRTUALTELESCOPE_IERS_H

//todo IERS as a service

class IERS {

    public:

        double polarMotionX(const double& mjd) const;
        double polarMotionY(const double& mjd) const;
        double ut1_utc(const double& mjd) const;
        double tai_utc(void) const;
        double tt_tai(void) const;
};


#endif //VIRTUALTELESCOPE_IERS_H
