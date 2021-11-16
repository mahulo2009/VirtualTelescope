//
// Created by mhuertas on 06/11/2021.
//
#include "VirtualTelescope.h"

#define GCS_AS2R  4.8481368110953599358991410235794797595635330237270e-6
#define Day2Second 86400.0
#define MJD_1970 40587.0

#include <iostream>
#include "PointModel.h"
#include <math.h>
#include <ctime>
#include <chrono>

int main()
{
    vt::main::GeoDataParams geoDataParams;
    geoDataParams.meanLon=-0.3122447361322777670267214489285834133625030517578125;
    geoDataParams.meanLat=0.5018798499051132511183936912857461720705032348632812;
    geoDataParams.height=2267.83;

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

    vt::main::VirtualTelescope vt(geoDataParams,telescopeParams);
    vt.setIers({8.62145e-321,
                2.77171e-321,
                -1.34082e-313, //TODO THIS VALUES NOT OK REVIEW
                37.0/Day2Second,
                32.184/Day2Second});
    //------------------
    vt.init();

    vt.setWeather({273,
                   1000.0,
                   0.0,
                   0.0065});

    vt.setTaiMjd(59533.7);

    //------------------
    vt.slowUpdate();


    PointModel pmp;
    pmp.read("../../pointmodel.dat");
    pmp.print();
    pmp.fetch();
    vt.setPointingModel(pmp);

    vt.setTelescopeStatus({3.14159,
                           0});

    vt.setTarget({FK5,
                  2000.0,
                  0.5,
                  NASMYTH_L});

    vt.setPointOrig({FK5,
                     2000.0,
                     0.5,
                     NASMYTH_L});

    //------------------
    vt.mediumUpdate();



    using std::chrono::system_clock;
    system_clock::time_point now = system_clock::now();
    std::time_t tai = system_clock::to_time_t ( now );
    std::cout << "today is: " << tai << std::endl;

    vt.setTaiMjd(tai*(1/Day2Second)+MJD_1970);
    vt.setSideralTime(4*(360/24.0)*(M_PI/180.0));





    double enc_roll, enc_pitch,  enc_rma;
    vt.vtSkyToEnc (4*(360/24.0)*(M_PI/180.0),60*(M_PI/180.0) ,
                     0, 0,
                     enc_roll, enc_pitch, enc_rma,
                     10);

    std::cout << "enc_roll " << enc_roll << std::endl;
    std::cout << "enc_pitch " << enc_pitch << std::endl;
    std::cout << "enc_rma " << enc_rma << std::endl;

    return 0;
}
