//
// Created by mhuertas on 06/11/2021.
//
#include "VirtualTelescope.h"

#define GCS_AS2R  4.8481368110953599358991410235794797595635330237270e-6
#define Day2Second 86400.0
#define Second2Day 1.1574074074074073e-05
#define MJD_1970 40587.0

#include <iostream>
#include "PointModel.h"
#include <math.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <iomanip>

vt::main::GeoDataParams loadGeoParams()
{
    //Set Geo Data
    vt::main::GeoDataParams geoDataParams;
    geoDataParams.meanLon=-0.3122447361322777670267214489285834133625030517578125;
    geoDataParams.meanLat=0.5018798499051132511183936912857461720705032348632812;
    geoDataParams.height=2267.83;

    return geoDataParams;
}

vt::main::TelescopeParams loadTelescopeParams()
{
    //Set Telescope Params
    vt::main::TelescopeParams telescopeParams;
    telescopeParams.focal_len=170000.0;
    telescopeParams.plate_scale=169887.94501735844196107771603192;
    telescopeParams.distortionCoefficient=-382.4338085509681051168;
    telescopeParams.refWaveLen=0.5;
    telescopeParams.mountType=ALTAZ;
    telescopeParams.gimbalX=0.0;
    telescopeParams.gimbalY=0.0;
    telescopeParams.gimbalZ=0.0;
    telescopeParams.belowPole=false;
    telescopeParams.aux[0]=telescopeParams.aux[1]=telescopeParams.aux[2]=0.0;

    return telescopeParams;
}

void setIersParams(vt::main::VirtualTelescope &vt)
{
    //Set IERS data
    vt.setIers({0.1213 * 4.84814e-6 ,
                0.2464 * 4.84814e-6,
                -0.10581 * Second2Day, //TODO THIS VALUES NOT OK REVIEW
                37.0/Day2Second,
                32.184/Day2Second});
}

void setPointingModelParams(vt::main::VirtualTelescope &vt) {
    //Set Pointing model data
    PointModel pmp;
    pmp.read("../../pointmodel.dat");
    //pmp.print();
    pmp.fetch();
    vt.setPointingModel(pmp);
}

void updateWeather(vt::main::VirtualTelescope &vt) {
    //Set Weather data
    vt.setWeather({273,
                   1000.0,
                   0.0,
                   0.0065});
}

void updateTargetParams(vt::main::VirtualTelescope &vt) {
    vt.setTarget({FK5,
                  2000.0,
                  0.626,
                  NASMYTH_L});

    vt.setPointOrig({FK5,
                     2000.0,
                     0.626,
                     NASMYTH_L});
}


int main()
{

    vt::main::GeoDataParams geoDataParams= loadGeoParams();
    vt::main::TelescopeParams telescopeParams =loadTelescopeParams();
    vt::main::VirtualTelescope vt(geoDataParams,telescopeParams);
    setIersParams(vt);
    setPointingModelParams(vt);
    updateWeather(vt);
    updateTargetParams(vt);

    auto now = std::chrono::system_clock::now();
    time_t tai = std::chrono::system_clock::to_time_t( now );
    double taiMjd = tai * (Second2Day)  + MJD_1970;
    //Set Time data
    vt.setTaiMjd(taiMjd);


    //vt.setSideralTime(5.216);
    //Set target data
    vt.setTelescopeStatus({0,
                                        0}); //todo check how to obtain these values

    //init------------------
    vt.init();

    double cood[] = {5.0,1.0};
    vt.targetCoordenates(cood);

    std::cout << std::setprecision(15) << std::endl;
    for (int i=0;i<60;i++)
    {
        auto now = std::chrono::system_clock::now();
        time_t tai = std::chrono::system_clock::to_time_t( now );
        double taiMjd = tai * (Second2Day) + 37 *(Second2Day) + MJD_1970;
        //Set Time data
        vt.setTaiMjd(taiMjd);

        //slow------------------
        vt.slowUpdate();

        //medium------------------
        vt.mediumUpdate();

        //sky to enc------------------
        double enc_roll, enc_pitch,  enc_rma;
        vt.vtSkyToEnc (cood[0],cood[1],
                       0, 0,
                       enc_roll, enc_pitch, enc_rma,
                       100);

        std::cout << "timestamp\t\t=\t" <<  taiMjd << " MJD(TAI)" <<std::endl;
        std::cout << "sidereal time\t=\t" <<  vt.getSideralTime() << " radians" <<std::endl;
        std::cout << "Az demand\t\t=\t" <<  (M_PI-enc_roll)*(180/M_PI) << " degrees" <<std::endl;
        std::cout << "Elv demand\t\t=\t" <<  enc_pitch*(180/M_PI) << " degrees" <<std::endl;
        std::cout << "rotator demand\t=\t" <<  enc_rma*(180/M_PI) << " degrees" <<std::endl;
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
