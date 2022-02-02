//
// Created by mhuertas on 06/11/2021.
#include "gtest/gtest.h"

#include "VirtualTelescope.h"
#include "VirtualTelescopeSkyToEncoder.h"
#include "PointModel.h"
#include "VirtualTelescopeEncoderToSky.h"
#include "VirtualTelescopeSkyToPointingOrigin.h"

#define Second2Day 1.1574074074074073e-05

class VirtualTelescopeTest : public ::testing::Test {
public:
    VirtualTelescopeTest()
    {
        vt::main::GeographicalParameters geographicalParameters;
        geographicalParameters.meanLon=-0.3122447361322777670267214489285834133625030517578125;
        geographicalParameters.meanLat=0.5018798499051132511183936912857461720705032348632812;
        geographicalParameters.height=2267.83;

        vt::main::OpticalParameters opticalParameters;
        opticalParameters.focal_len=170000.0;
        opticalParameters.plate_scale=169887.94501735844196107771603192;
        opticalParameters.distortionCoefficient=-382.4338085509681051168;
        opticalParameters.refWaveLen=0.5;
        opticalParameters.mountType=ALTAZ;

        vt::main::PolarMotionParameters polarMotionParameters;
        polarMotionParameters.polarMotionX=0.1213 * 4.84814e-6;
        polarMotionParameters.polarMotionY=0.2464 * 4.84814e-6;

        vt::main::TimeTransformParameters timeTransformParameters;
        timeTransformParameters.ut1ToUtc=-0.10581 * Second2Day; //todo review this values
        timeTransformParameters.taiToUtc=37.0 * Second2Day;
        timeTransformParameters.ttToTai=32.184 * Second2Day;

        PointModel pmp;
        pmp.read("../pointmodel.dat"); //todo if the file is not found throw error
        pmp.fetch();


        vt::main::PointModelParameters pointModelParameters;
        pointModelParameters.maxTerms=pmp.maxTerms;
        pointModelParameters.ntRoom=pmp.ntRoom;
        std::copy(std::begin(pmp.model), std::end(pmp.model), std::begin(pointModelParameters.model));
        std::copy(std::begin(pmp.coefValues), std::end(pmp.coefValues), std::begin(pointModelParameters.coefValues));
        pointModelParameters.numLocalTerms=pmp.numLocalTerms;
        pointModelParameters.numExplTerms=pmp.numExplTerms;
        pointModelParameters.numTerms=pmp.numTerms;
        std::copy(&pmp.coefNames[0][0], &pmp.coefNames[0][0]+100*9, &pointModelParameters.coefNames[0][0]);
        std::copy(&pmp.coefFormat[0][0], &pmp.coefFormat[0][0]+100*9, &pointModelParameters.coefFormat[0][0]);

        vt::main::TelescopeContext telescopeContext;
        telescopeContext.geographicalParameters=geographicalParameters;
        telescopeContext.opticalParameters=opticalParameters;
        telescopeContext.polarMotionParameters=polarMotionParameters;
        telescopeContext.timeTransformParameters=timeTransformParameters;
        telescopeContext.pointModelParameters=pointModelParameters;

        virtualTelescopeSkyToEncoder = new vt::main::VirtualTelescopeSkyToEncoder(telescopeContext);
        virtualTelescopeEncoderToSky = new vt::main::VirtualTelescopeEncoderToSky(telescopeContext);
        virtualTelescopeSkyToPointingOrigin = new vt::main::VirtualTelescopeSkyToPointingOrigin(telescopeContext);

        virtualTelescopeSkyToEncoder->
            updateWeatherData({273,1000.0,0.0,0.0065});
        virtualTelescopeSkyToEncoder->
                updateTaiMjd(59565.660451388889);

        virtualTelescopeEncoderToSky->
                updateWeatherData({273,1000.0,0.0,0.0065});
        virtualTelescopeEncoderToSky->
                updateTaiMjd(59565.660451388889);

        virtualTelescopeSkyToPointingOrigin->
                updateWeatherData({273,1000.0,0.0,0.0065});
        virtualTelescopeSkyToPointingOrigin->
                updateTaiMjd(59565.660451388889);

    }

    vt::main::VirtualTelescope * virtualTelescopeSkyToEncoder;
    vt::main::VirtualTelescope * virtualTelescopeEncoderToSky;
    vt::main::VirtualTelescope * virtualTelescopeSkyToPointingOrigin;
};

TEST_F(VirtualTelescopeTest,SKY2ENCODER) {

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
                          0.0,{5.344,1.0}});


    virtualTelescopeSkyToEncoder->init();
    virtualTelescopeSkyToEncoder->slowUpdate();
    virtualTelescopeSkyToEncoder->mediumUpdate();
    virtualTelescopeSkyToEncoder->fastUpdate();
    virtualTelescopeSkyToEncoder->print();

    ASSERT_DOUBLE_EQ(1.0, 2.0);
}

TEST_F(VirtualTelescopeTest,ENCODER2SKY) {

    virtualTelescopeEncoderToSky->
            updateMountDemand({M_PI-0.026,
                               1.072,
                               1.065});

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


    virtualTelescopeEncoderToSky->init();
    virtualTelescopeEncoderToSky->slowUpdate();
    virtualTelescopeEncoderToSky->mediumUpdate();
    virtualTelescopeEncoderToSky->fastUpdate();
    virtualTelescopeEncoderToSky->print();

    ASSERT_DOUBLE_EQ(1.0, 2.0);
}

TEST_F(VirtualTelescopeTest,SKY2POINTORIGIN) {
    virtualTelescopeSkyToPointingOrigin->
            updateMountDemand({0,
                               1.0,
                               1.0});

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
                          0.0,{5.34437,1.0}});


    virtualTelescopeSkyToPointingOrigin->init();
    virtualTelescopeSkyToPointingOrigin->slowUpdate();
    virtualTelescopeSkyToPointingOrigin->mediumUpdate();
    virtualTelescopeSkyToPointingOrigin->fastUpdate();
    virtualTelescopeSkyToPointingOrigin->print();

    ASSERT_DOUBLE_EQ(1.0, 2.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}