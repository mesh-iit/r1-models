
#include <iDynTree/Axis.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/JointState.h>
#include <iDynTree/Indices.h>
#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Sensors.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

#include <yarp/os/Property.h>
#include <cmath>
#include <cstdlib>

bool isNotR1Mk3Model(const std::string& modelPath)
{
    return (modelPath.find("Gazebo") != std::string::npos);
}

inline bool checkDoubleAreEqual(const double & val1,
                                const double & val2,
                                const double tol)
{
    if( std::fabs(val1-val2) > tol )
    {
        return false;
    }

    return true;
}


template<typename VectorType1, typename VectorType2>
bool checkVectorAreEqual(const VectorType1 & dir1,
                         const VectorType2 & dir2,
                         const double tol)
{
    if( dir1.size() != dir2.size() )
    {
        return false;
    }

    for(int i=0; i < dir1.size(); i++)
    {
        if( std::fabs(dir1(i)-dir2(i)) > tol )
        {
            return false;
        }
    }
    return true;
}

template<typename MatrixType1, typename MatrixType2>
bool checkMatrixAreEqual(const MatrixType1 & mat1,
                         const MatrixType2 & mat2,
                         const double tol)
{
    if( mat1.rows() != mat2.rows() ||
        mat1.cols() != mat2.cols() )
    {
        return false;
    }

    for(int i=0; i < mat1.rows(); i++)
    {
        for(int j=0; j < mat1.cols(); j++ )
        {
            if( std::fabs(mat1(i,j)-mat2(i,j)) > tol )
            {
                return false;
            }
        }
    }
    return true;
}

bool checkTransformAreEqual(const iDynTree::Transform & t1,
                            const iDynTree::Transform & t2,
                            const double tol)
{
    return checkMatrixAreEqual(t1.getRotation(),t2.getRotation(),tol) &&
           checkVectorAreEqual(t1.getPosition(),t2.getPosition(),tol);
}


bool getAxisInRootLink(iDynTree::KinDynComputations & comp,
                                 const std::string jointName,
                                 iDynTree::Axis & axisInRootLink)
{
    iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("base_link");

    if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "r1-model-test error: impossible to find base_link in model" << std::endl;
        return false;
    }

    iDynTree::JointIndex jntIdx = comp.getRobotModel().getJointIndex(jointName);

    if( jntIdx == iDynTree::JOINT_INVALID_INDEX )
    {
        std::cerr << "r1-model-test error: impossible to find " << jointName << " in model" << std::endl;
        return false;
    }

    iDynTree::LinkIndex childLinkIdx = comp.getRobotModel().getJoint(jntIdx)->getSecondAttachedLink();

    // Check that the joint are actually revolute as all the joints in iCub
    const iDynTree::RevoluteJoint * revJoint = dynamic_cast<const iDynTree::RevoluteJoint *>(comp.getRobotModel().getJoint(jntIdx));

    if( !revJoint )
    {
        std::cerr << "r1-model-test error: " << jointName << " is not revolute " << std::endl;
        return false;
    }

    if( !revJoint->hasPosLimits() )
    {
        std::cerr << "r1-model-test error: " << jointName << " is a continous joint" << std::endl;
        return false;
    }

    axisInRootLink = comp.getRelativeTransform(rootLinkIdx,childLinkIdx)*(revJoint->getAxis(childLinkIdx));

    return true;
}

bool checkBaseLink(iDynTree::KinDynComputations & comp)
{
    iDynTree::LinkIndex base_linkIdx = comp.getFrameIndex("base_link");

    if( base_linkIdx == iDynTree::FRAME_INVALID_INDEX )
    {
        std::cerr << "r1-model-test error: impossible to find base_link in model" << std::endl;
        return false;
    }

    std::cerr << "r1-model-test : base_link test performed correctly " << std::endl;

    return true;
}

// TODO soles are not present, maybe we can do it for the wheels
// bool checkSolesAreParallelAndCorrectlyPlaced(iDynTree::KinDynComputations & comp)
// {
//     iDynTree::LinkIndex rootLinkIdx = comp.getFrameIndex("base_link");

//     if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
//     {
//         std::cerr << "r1-model-test error: impossible to find root_link in model" << std::endl;
//         return false;
//     }

//     iDynTree::LinkIndex l_sole = comp.getFrameIndex("l_sole");

//     if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
//     {
//         std::cerr << "r1-model-test error: impossible to find frame l_sole in model" << std::endl;
//         return false;
//     }

//     iDynTree::LinkIndex r_sole = comp.getFrameIndex("r_sole");

//     if( rootLinkIdx == iDynTree::FRAME_INVALID_INDEX )
//     {
//         std::cerr << "r1-model-test error: impossible to find frame r_sole in model" << std::endl;
//         return false;
//     }

//     iDynTree::Transform root_H_l_sole = comp.getRelativeTransform(rootLinkIdx,l_sole);
//     iDynTree::Transform root_H_r_sole = comp.getRelativeTransform(rootLinkIdx,r_sole);

//     iDynTree::Transform root_H_l_sole_expected(iDynTree::Rotation(1, 0, 0,
//                                                                   0, 1, 0,
//                                                                   0, 0, 1),
//                                                iDynTree::Position(0.04403,0.0744,-0.7793));
//     iDynTree::Transform root_H_r_sole_expected(iDynTree::Rotation(1, 0, 0,
//                                                                   0, 1, 0,
//                                                                   0, 0, 1),
//                                                iDynTree::Position(0.04403,-0.0744,-0.7793));


//     if (!checkTransformAreEqual(root_H_l_sole, root_H_l_sole_expected, 1e-5))
//     {
//         std::cerr << "r1-model-test : transform between root_H_l_sole is not the expected one, test failed." << std::endl;
//         std::cerr << "r1-model-test : root_H_l_sole :" << root_H_l_sole.toString() << std::endl;
//         std::cerr << "r1-model-test : root_H_l_sole_expected :" << root_H_l_sole_expected.toString() << std::endl;
//         return false;
//     }

//     if (!checkTransformAreEqual(root_H_r_sole, root_H_r_sole_expected, 1e-5))
//     {
//         std::cerr << "r1-model-test : transform between root_H_r_sole is not the expected one, test failed." << std::endl;
//         std::cerr << "r1-model-test : root_H_r_sole :" << root_H_r_sole.toString() << std::endl;
//         std::cerr << "r1-model-test : root_H_r_sole_expected :" << root_H_r_sole_expected.toString() << std::endl;
//         return false;
//     }

//     // height of the sole should be equal
//     double l_sole_height = root_H_l_sole.getPosition().getVal(2);
//     double r_sole_height = root_H_r_sole.getPosition().getVal(2);

//     if( !checkDoubleAreEqual(l_sole_height,r_sole_height,1e-5) )
//     {
//         std::cerr << "r1-model-test error: l_sole_height is " << l_sole_height << ", while r_sole_height is " << r_sole_height << " (diff : " << std::fabs(l_sole_height-r_sole_height) <<  " )"  << std::endl;
//         return false;
//     }

//     // x should also be equal
//     double l_sole_x = root_H_l_sole.getPosition().getVal(0);
//     double r_sole_x = root_H_r_sole.getPosition().getVal(0);

//     // The increased threshold is a workaround for https://github.com/robotology/ergocub-model-generator/issues/125
//     if( !checkDoubleAreEqual(l_sole_x,r_sole_x, 2e-4) )
//     {
//         std::cerr << "r1-model-test error: l_sole_x is " << l_sole_x << ", while r_sole_x is " << r_sole_x << " (diff : " << std::fabs(l_sole_x-r_sole_x) <<  " )"  << std::endl;
//         return false;
//     }

//     // y should be simmetric
//     double l_sole_y = root_H_l_sole.getPosition().getVal(1);
//     double r_sole_y = root_H_r_sole.getPosition().getVal(1);

//     // The increased threshold is a workaround for https://github.com/robotology/ergocub-model-generator/issues/125
//     if( !checkDoubleAreEqual(l_sole_y,-r_sole_y,1e-4) )
//     {
//         std::cerr << "r1-model-test error: l_sole_y is " << l_sole_y << ", while r_sole_y is " << r_sole_y << " while they should be simmetric (diff : " << std::fabs(l_sole_y+r_sole_y) <<  " )"  << std::endl;
//         return false;
//     }


//     std::cerr << "r1-model-test : sole are parallel test performed correctly " << std::endl;

//     return true;
// }



bool checkAxisDirections(iDynTree::KinDynComputations & comp, bool isNotR1Mk3Model)
{

    std::vector<std::string> axisNames;
    std::vector<iDynTree::Direction> expectedDirectionInRootLink;
    axisNames.push_back("mobile_base_l_wheel_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0,1,0));
    // axisNames.push_back("torso_heave_eq_joint"); //non revolute
    // expectedDirectionInRootLink.push_back(iDynTree::Direction(0,-1,0));
    axisNames.push_back("mobile_base_r_wheel_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0,1,0));
    axisNames.push_back("torso_roll_eq_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1,0,0));
    axisNames.push_back("torso_pitch_eq_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0,-1,0));
    axisNames.push_back("torso_yaw_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0,0,1));
    axisNames.push_back("l_shoulder_pitch");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(8.67743e-05,0.970296,0.24192));
    axisNames.push_back("r_shoulder_pitch");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-8.67743e-05,0.970296,-0.24192));
    axisNames.push_back("neck_pitch_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0,1,0));
    axisNames.push_back("neck_yaw_joint");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.173648,-1.18476e-14,0.984808));
    axisNames.push_back("r_shoulder_roll");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-1,1.38653e-05,0.000414302));
    axisNames.push_back("r_shoulder_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.000414302,-2.34529e-06,1));
    axisNames.push_back("r_elbow");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-1.38663e-05,-1,-2.33954e-06));
    axisNames.push_back("r_wrist_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.000414302,-2.34529e-06,1));
    axisNames.push_back("r_wrist_roll");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1,-1.38653e-05,-0.000414302));
    axisNames.push_back("r_wrist_pitch");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-1.38663e-05,-1,-2.33954e-06));
    axisNames.push_back("r_index_add");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1.38663e-05,1,2.33954e-06));
    axisNames.push_back("r_pinkie_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.995817,-0.0260909,-0.0875684));
    axisNames.push_back("r_thumb_add");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.224547,8.33839e-07,0.974463));
    axisNames.push_back("r_middle_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1,-1.38653e-05,-0.000414302));
    axisNames.push_back("r_ring_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.995817,-0.0260909,-0.0875684));
    axisNames.push_back("r_ring_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.995817,-0.0260909,-0.0875684));
    axisNames.push_back("r_middle_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1,-1.38653e-05,-0.000414302));
    axisNames.push_back("r_thumb_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.185415,0.969849,0.158162));
    axisNames.push_back("r_thumb_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.185415,0.969849,0.158162));
    axisNames.push_back("r_pinkie_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.995817,-0.0260909,-0.0875684));
    axisNames.push_back("r_index_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.956426,-1.39452e-05,0.291975));
    axisNames.push_back("r_index_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.956426,-1.39452e-05,0.291975));
    axisNames.push_back("l_shoulder_roll");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1,1.38653e-05,-0.000414302));
    axisNames.push_back("l_shoulder_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.000414302,-2.34529e-06,-1));
    axisNames.push_back("l_elbow");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1.38663e-05,-1,2.33954e-06));
    axisNames.push_back("l_wrist_yaw");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.0414261,-2.91196e-06,-0.999142));
    axisNames.push_back("l_wrist_roll");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.999142,-1.37575e-05,0.0414261));
    axisNames.push_back("l_wrist_pitch");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(1.38663e-05,-1,2.33954e-06));
    axisNames.push_back("l_thumb_add");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.184394,2.57429e-07,-0.982852));
    axisNames.push_back("l_pinkie_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.991388,-0.0260908,0.128335));
    axisNames.push_back("l_middle_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.999142,-1.37575e-05,0.0414261));
    axisNames.push_back("l_ring_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.991388,-0.0260908,0.128335));
    axisNames.push_back("l_index_add");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-1.38663e-05,1,-2.33954e-06));
    axisNames.push_back("l_index_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.967596,-1.40077e-05,-0.252505));
    axisNames.push_back("l_index_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.967596,-1.40077e-05,-0.252505));
    axisNames.push_back("l_ring_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.991388,-0.0260908,0.128335));
    axisNames.push_back("l_middle_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.999142,-1.37575e-05,0.0414261));
    axisNames.push_back("l_pinkie_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(-0.991388,-0.0260908,0.128335));
    axisNames.push_back("l_thumb_prox");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.178772,0.969848,-0.165634));
    axisNames.push_back("l_thumb_dist");
    expectedDirectionInRootLink.push_back(iDynTree::Direction(0.178772,0.969848,-0.165634));

    for(int i=0; i < axisNames.size(); i++)
    {
        std::string axisToCheck = axisNames[i];
        iDynTree::Axis axisInRootLink;
        iDynTree::Direction expectedDirection = expectedDirectionInRootLink[i];
        bool getAxisOk = getAxisInRootLink(comp,axisToCheck,axisInRootLink);

        if( !getAxisOk ) {
            return false;
        }

        if( !checkVectorAreEqual(axisInRootLink.getDirection(),expectedDirection,1e-5) )
        {
            std::cerr << "r1-model-test error:" << axisToCheck << " got direction of " << axisInRootLink.getDirection().toString()
                  << " instead of expected " << expectedDirection.toString() << "\n" << std::endl;
            return false;
        }
    }

    return true;
}

bool checkFTSensorsAreOddAndNotNull(iDynTree::ModelLoader & mdlLoader)
{
    int nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    if( nrOfFTSensors == 0 )
    {
        std::cerr << "r1-model-test error: no F/T sensor found in the model" << std::endl;
        return false;
    }

    if( nrOfFTSensors % 2 == 0 )
    {
        std::cerr << "r1-model-test : even number of F/T sensor found in the model" << std::endl;
        return false;
    }


    return true;
}

/**
 * All the iCub have a even and not null number of F/T sensors.
 */
bool checkFTSensorsAreEvenAndNotNull(iDynTree::ModelLoader & mdlLoader)
{
    int nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    if( nrOfFTSensors == 0 )
    {
        std::cerr << "r1-model-test error: no F/T sensor found in the model" << std::endl;
        return false;
    }

    if( nrOfFTSensors % 2 == 1 )
    {
        std::cerr << "r1-model-test : odd number of F/T sensor found in the model" << std::endl;
        return false;
    }


    return true;
}


bool checkFrameIsCorrectlyOriented(iDynTree::KinDynComputations & comp,
                                      const iDynTree::Rotation& expected,
                                      const std::string& frameName)
{
    if (!comp.model().isFrameNameUsed(frameName))
    {
        return true;
    }

    iDynTree::Rotation actual = comp.getRelativeTransform("base_link", frameName).getRotation();

    if (!checkMatrixAreEqual(expected, actual, 1e-3))
    {
        std::cerr << "r1-model-test : transform between base_link and " << frameName << " is not the expected one, test failed." << std::endl;
        std::cerr << "r1-model-test : Expected transform : " << expected.toString() << std::endl;
        std::cerr << "r1-model-test : Actual transform : " << actual.toString() << std::endl;
        return false;
    }

    return true;
}



bool checkFTSensorsAreCorrectlyOriented(iDynTree::KinDynComputations & comp)
{

    iDynTree::Rotation rootLink_R_sensorFrameLeftArmExpected =
        iDynTree::Rotation(1.38663e-05, -1, -0.000414302,
                            -1, -1.38653e-05, -2.34529e-06,
                            2.33954e-06, 0.000414302, -1);
    iDynTree::Rotation rootLink_R_sensorFrameRightArmExpected =
        iDynTree::Rotation(1.38663e-05, 1, -0.000414302,
                            1, -1.38653e-05, 2.34529e-06,
                            2.33954e-06, -0.000414302, -1);

    bool ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameLeftArmExpected, "l_arm_ft");
    ok = checkFrameIsCorrectlyOriented(comp, rootLink_R_sensorFrameRightArmExpected, "r_arm_ft") && ok;
    return ok;
}


bool Model_isFrameNameUsed(const iDynTree::Model& model, const std::string frameName)
{
    for(size_t i=0; i < model.getNrOfLinks(); i++ )
    {
        if( frameName == model.getLinkName(i) )
        {
            return true;
        }
    }

    for(size_t i=model.getNrOfLinks(); i < model.getNrOfFrames(); i++ )
    {

        if( frameName == model.getFrameName(i) )
        {
            return true;
        }
    }

    return false;
}

bool checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(const std::string& modelPath,
                                                                                        iDynTree::KinDynComputations& comp,
                                                                                        const iDynTree::SensorsList& sensors,
                                                                                        const std::string& sensorName)
{
    // As of mid 2023, for iCub 3 models the frame name is <prefix>_ft, while the sensor name is <prefix>_ft,
    // and the joint name is <prefix>_ft_sensor
    std::string frameName = sensorName;

    //std::cerr << comp.model().toString() << std::endl;

    // Check frame exist
    if (!comp.model().isFrameNameUsed(frameName))
    {
        std::cerr << "r1-model-test : model " << modelPath << " does not contain frame " << frameName << " as expected." << std::endl;
        return false;
    }

    // Check sensors exists
    std::ptrdiff_t sensorIdx;
    if (!sensors.getSensorIndex(iDynTree::SIX_AXIS_FORCE_TORQUE, sensorName, sensorIdx))
    {
        std::cerr << "r1-model-test : model " << modelPath << " does not contain FT sensor " << sensorName << " as expected." <<  std::endl;
        return false;
    }

    // Get base_H_link
    iDynTree::Transform base_H_frame = comp.getRelativeTransform("base_link", frameName);

    // Get base_H_sensor
    iDynTree::SixAxisForceTorqueSensor * sens
            = (::iDynTree::SixAxisForceTorqueSensor *) sensors.getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE, sensorIdx);
    if (!sens)
    {
        std::cerr << "r1-model-test : model " << modelPath << " error in reading sensor " << sensorName << "." <<  std::endl;
        return false;
    }

    std::string firstLinkName = sens->getFirstLinkName();
    iDynTree::Transform base_H_firstLink = comp.getRelativeTransform("base_link", firstLinkName);
    iDynTree::Transform firstLink_H_sensor;
    bool ok = sens->getLinkSensorTransform(sens->getFirstLinkIndex(), firstLink_H_sensor);

    if (!ok)
    {
        std::cerr << "r1-model-test : model " << modelPath << " error in reading transform of sensor " << sensorName << "." <<  std::endl;
        return false;
    }

    //std::cout<<firstLink_H_sensor.toString()<<std::endl;
    //std::cout<<firstLinkName<<" "<<sens->getSecondLinkName()<<std::endl;
    //std::cout<<base_H_firstLink.toString();

    iDynTree::Transform base_H_sensor = base_H_firstLink*firstLink_H_sensor;

    // Check that the two transfom are equal equal
    if (!checkTransformAreEqual(base_H_frame, base_H_sensor, 1e-5))
    {
        std::cerr << "r1-model-test : transform between base_H_frame and base_H_sensor for " << sensorName << " is not the expected one, test failed." << std::endl;
        std::cerr << "r1-model-test : base_H_frame :" << base_H_frame.toString() << std::endl;
        std::cerr << "r1-model-test : base_H_sensor :" << base_H_sensor.toString() << std::endl;
        return false;
    }

    // Beside checking that base_H_frame and base_H_sensor, we should also check that the translation
    // between the child link of FT joint and the FT frame is the zero vector, as as of mid 2023 the SDF
    // standard always assume that the 6D FT measured by the sensor is expressed in the origin of the child link frame
    // See https://github.com/gazebosim/sdformat/issues/130 for more details
    iDynTree::Traversal traversalWithURDFBase;
    comp.model().computeFullTreeTraversal(traversalWithURDFBase);

    iDynTree::LinkIndex childLinkIdx = traversalWithURDFBase.getChildLinkIndexFromJointIndex(comp.model(), sens->getParentJointIndex());
    std::string childLinkName = comp.model().getLinkName(childLinkIdx);

    iDynTree::Transform childLink_H_sensorFrame = comp.getRelativeTransform(childLinkName, frameName);

    iDynTree::Vector3 zeroVector;
    zeroVector.zero();

    if (!checkVectorAreEqual(childLink_H_sensorFrame.getPosition(), zeroVector, 1e-6))
    {
        std::cerr << "r1-model-test : translation between link "  << childLinkName <<  " and sensor " << sensorName << " is non-zero, test failed, see  https://github.com/gazebosim/sdformat/issues/130  for more details." << std::endl;
        std::cerr << "r1-model-test : childLink_H_sensorFrame.getPosition(): "  << childLink_H_sensorFrame.getPosition().toString() << std::endl;
        return false;
    }

    return true;
}

// Check FT sensors
// This is only possible with V3 as V3 models have FT frame exported models
// However, as of mid 2023 the V2 models do not need this check as the link explicitly
// are using the FT frames as frames of the corresponding link
bool checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(const std::string& modelPath)
{
    iDynTree::ModelLoader mdlLoader;

    // Open the model
    iDynTree::ModelParserOptions parserOptions;

    // By default iDynTree creates an additional frame with the same name of the sensor,
    // however in this case we have both the sensor and the urdf frame called <prefix>_ft,
    // and for this test we want to make sure that the <prefix>_ft additional frame is the
    // one in the URDF
    parserOptions.addSensorFramesAsAdditionalFrames = false;
    mdlLoader.setParsingOptions(parserOptions);

    mdlLoader.loadModelFromFile(modelPath);

    iDynTree::KinDynComputations comp;
    const bool modelLoaded = comp.loadRobotModel(mdlLoader.model());

    if (!modelLoaded)
    {
        std::cerr << "r1-model-test error: impossible to load model from " << modelLoaded << std::endl;
        return false;
    }

    iDynTree::Vector3 grav;
    grav.zero();
    iDynTree::JointPosDoubleArray qj(comp.getRobotModel());
    iDynTree::JointDOFsDoubleArray dqj(comp.getRobotModel());
    qj.zero();
    dqj.zero();

    comp.setRobotState(qj,dqj,grav);
    iDynTree::SensorsList sensors = mdlLoader.sensors();


    bool ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "l_arm_ft");
    ok = checkFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath, comp, sensors, "r_arm_ft") && ok;
    return ok;
}

int main(int argc, char ** argv)
{
    yarp::os::Property prop;

    // Parse command line options
    prop.fromCommand(argc,argv);

    // Skip upper body tests (to support testing iCubHeidelberg01 model)
    bool skipUpperBody = prop.check("skipUpperBody");

    // Get model name
    if( ! prop.find("model").isString() )
    {
        std::cerr << "r1-model-test error: model option not found" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string modelPath = prop.find("model").asString().c_str();

    iDynTree::ModelLoader mdlLoader;
    mdlLoader.loadModelFromFile(modelPath);

    // Open the model
    iDynTree::KinDynComputations comp;
    const bool modelLoaded = comp.loadRobotModel(mdlLoader.model());

    if( !modelLoaded )
    {
        std::cerr << "r1-model-test error: impossible to load model from " << modelLoaded << std::endl;
        return EXIT_FAILURE;
    }

    iDynTree::Vector3 grav;
    grav.zero();
    iDynTree::JointPosDoubleArray qj(comp.getRobotModel());
    iDynTree::JointDOFsDoubleArray dqj(comp.getRobotModel());
    qj.zero();
    dqj.zero();

    comp.setRobotState(qj,dqj,grav);

    // Check axis
    if( !checkAxisDirections(comp, isNotR1Mk3Model(modelPath)) )
    {
        return EXIT_FAILURE;
    }
    // Check if base_link exist, and check that is a frame attached to base_link and if its
    // transform is the idyn
    if( !checkBaseLink(comp) )
    {
        return EXIT_FAILURE;
    }

    // Now some test that test the sensors
    // The ft sensors orientation respect to the base_link are different to iCubV2 and they are under investigation.
    if( !checkFTSensorsAreEvenAndNotNull(mdlLoader) )
    {
        return EXIT_FAILURE;
    }

    if (!checkFTSensorsAreCorrectlyOriented(comp))
    {
       return EXIT_FAILURE;
    }

    if (!checkAllFTMeasurementFrameGivenBySensorTagsIsCoherentWithMeasurementFrameGivenByFrame(modelPath))
    {
        return EXIT_FAILURE;
    }


    std::cerr << "Check for model " << modelPath << " concluded correctly!" << std::endl;

    return EXIT_SUCCESS;
}
