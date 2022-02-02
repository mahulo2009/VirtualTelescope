//
// Created by mhuertas on 06/11/2021.
//

#include "VirtualTelescopeSkyToEncoder.h"
#include "VirtualTelescopeEncoderToSky.h"
#include "VirtualTelescopeSkyToPointingOrigin.h"
#include <vector>

#define Second2Day 1.1574074074074073e-05
#define MJD_1970 40587.0

#include "PointModel.h"
#include <math.h>
#include <ctime>
#include <chrono>

vt::main::GeographicalParameters loadGeoParams()
{
    vt::main::GeographicalParameters geographicalParameters;
    geographicalParameters.meanLon=-0.3122447361322777670267214489285834133625030517578125;
    geographicalParameters.meanLat=0.5018798499051132511183936912857461720705032348632812;
    geographicalParameters.height=2267.83;

    return geographicalParameters;
}

vt::main::OpticalParameters loadOpticalParameters()
{
    //Set Telescope Params
    vt::main::OpticalParameters opticalParameters;
    opticalParameters.focal_len=170000.0;
    opticalParameters.plate_scale=169887.94501735844196107771603192;
    opticalParameters.distortionCoefficient=-382.4338085509681051168;
    opticalParameters.refWaveLen=0.5;
    opticalParameters.mountType=ALTAZ;

    return opticalParameters;
}

vt::main::PolarMotionParameters loadPolarMotionParameters()
{
    vt::main::PolarMotionParameters polarMotionParameters;
    polarMotionParameters.polarMotionX=0.1213 * 4.84814e-6;
    polarMotionParameters.polarMotionY=0.2464 * 4.84814e-6;

    return polarMotionParameters;
}

vt::main::TimeTransformParameters loadTimeTransformParameters()
{
    vt::main::TimeTransformParameters timeTransformParameters;
    timeTransformParameters.ut1ToUtc=-0.10581 * Second2Day; //todo review this values
    timeTransformParameters.taiToUtc=37.0 * Second2Day;
    timeTransformParameters.ttToTai=32.184 * Second2Day;

    return timeTransformParameters;
}


vt::main::PointModelParameters loadPointModelParameters()
{
    vt::main::PointModelParameters pointModelParameters;

    PointModel pmp;
    pmp.read("../../pointmodel.dat");
    pmp.fetch();
    //pmp.print();

    pointModelParameters.maxTerms=pmp.maxTerms;
    pointModelParameters.ntRoom=pmp.ntRoom;
    std::copy(std::begin(pmp.model), std::end(pmp.model), std::begin(pointModelParameters.model));
    std::copy(std::begin(pmp.coefValues), std::end(pmp.coefValues), std::begin(pointModelParameters.coefValues));
    pointModelParameters.numLocalTerms=pmp.numLocalTerms;
    pointModelParameters.numExplTerms=pmp.numExplTerms;
    pointModelParameters.numTerms=pmp.numTerms;
    std::copy(&pmp.coefNames[0][0], &pmp.coefNames[0][0]+100*9, &pointModelParameters.coefNames[0][0]);
    std::copy(&pmp.coefFormat[0][0], &pmp.coefFormat[0][0]+100*9, &pointModelParameters.coefFormat[0][0]);

    return pointModelParameters;
}


void updateWeather(vt::main::VirtualTelescope &vt) {
    //Set Weather data
    vt.updateWeatherData({273,
                          1000.0,
                          0.0,
                          0.0065});
}

int main()
{
    vt::main::TelescopeContext telescopeContext;
    telescopeContext.geographicalParameters=loadGeoParams();
    telescopeContext.opticalParameters=loadOpticalParameters();
    telescopeContext.polarMotionParameters=loadPolarMotionParameters();
    telescopeContext.timeTransformParameters=loadTimeTransformParameters();
    telescopeContext.pointModelParameters=loadPointModelParameters();

    vt::main::VirtualTelescope * virtualTelescopeSkyToEncoder = new vt::main::VirtualTelescopeSkyToEncoder(telescopeContext);
    vt::main::VirtualTelescope * virtualTelescopeEncoderToSky = new vt::main::VirtualTelescopeEncoderToSky(telescopeContext);
    vt::main::VirtualTelescope * virtualTelescopeSkyToPointingOrigin = new vt::main::VirtualTelescopeSkyToPointingOrigin(telescopeContext);

    //Sky to Encoder
    virtualTelescopeSkyToEncoder->
        updateMountDemand({0,
                                        0,
                                        0});
    virtualTelescopeSkyToEncoder->
            updatePointOriging({FK5,
                                2000.0,
                                0.626,
                                NASMYTH_L,
                                0.0,{0,0}});

    virtualTelescopeSkyToEncoder->
            updateTarget({FK5,
                          2000.0,
                          0.626,
                          NASMYTH_L,
                          0.0,{5.964,1.0}});

    //Encoder to Sky
    virtualTelescopeEncoderToSky->
        updateMountDemand({
                                    M_PI+(35)*(M_PI/180.0),
                                    (43)*(M_PI/180.0),
                                    (118)*(M_PI/180.0) } );
    virtualTelescopeEncoderToSky->
            updatePointOriging({FK5,
                                2000.0,
                                0.626,
                                NASMYTH_L,
                                0.0,{0,0}});

    virtualTelescopeEncoderToSky->
            updateTarget({FK5,
                          2000.0,
                          0.626,
                          NASMYTH_L,
                          0.0,{0.0,0.0}});


    //Sky to Poiting Origin
    virtualTelescopeSkyToPointingOrigin->
            updateMountDemand({
                                      M_PI+(35)*(M_PI/180.0),
                                      (43)*(M_PI/180.0),
                                      (118)*(M_PI/180.0) } );
    virtualTelescopeSkyToPointingOrigin->
            updatePointOriging({FK5,
                                2000.0,
                                0.626,
                                NASMYTH_L,
                                0.0,{0,0}});

    virtualTelescopeSkyToPointingOrigin->
            updateTarget({FK5,
                          2000.0,
                          0.626,
                          NASMYTH_L,
                          0.0,{5.964,1.0}});


    std::vector<vt::main::VirtualTelescope*> virtualTelescopes;
    virtualTelescopes.push_back(virtualTelescopeSkyToEncoder);
    virtualTelescopes.push_back(virtualTelescopeEncoderToSky);
    virtualTelescopes.push_back(virtualTelescopeSkyToPointingOrigin);

    auto now = std::chrono::system_clock::now();
    time_t tai = std::chrono::system_clock::to_time_t( now );
    double taiMjd = tai * (Second2Day) + 37 *(Second2Day) + MJD_1970;

    for (auto vt : virtualTelescopes)
    {
        updateWeather(*vt);
        vt->updateTaiMjd(taiMjd);
        vt->init();
    }

    for (auto vt : virtualTelescopes)
    {
        vt->init();
        vt->slowUpdate();
        vt->mediumUpdate();
        vt->fastUpdate();
        vt->print();
    }

    return 0;
}
