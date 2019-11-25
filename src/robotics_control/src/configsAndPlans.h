//Configs:
// std::vector<double> idleConfig = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};

std::vector<double> preGraspSmall = {0.822716, -1.31612, 2.22471, -3.09909, -1.32535, 1.31554};
std::vector<double> preGraspMedium = {1.27992, -1.25995, 2.10726, -3.05872, -1.63359, 1.65229};
std::vector<double> preGraspLarge = {1.09172, -1.26745, 2.23029, -3.27299, -1.54429, 1.53417};

std::vector<double> graspSmall = {0.845758, -1.09288, 2.09879, -3.19561, -1.28939, 1.29291};
std::vector<double> graspMedium = {1.28511, -1.03589, 2.05326, -3.35281, -1.62399, 1.64745};
std::vector<double> graspLarge = {1.09723, -1.1352, 2.13748, -3.27136, -1.54863, 1.52398};

std::vector<double> aboveBoxA = {0.9804, -1.37721, 1.98231, -2.03515, -1.56245, -0.165955};
std::vector<double> aboveBoxB = {0.942007, -1.55605, 2.07771, -1.79518, -1.59887, -0.165931};
std::vector<double> aboveBoxC = {1.11376, -1.31014, 1.88295, -1.9344, -1.49911, -0.165979};
std::vector<double> aboveBoxD = {1.11423, -1.53884, 2.02828, -1.7641, -1.52779, -0.166015};

std::vector<double> aboveDiscard = {1.08083, -1.31036, 1.98195, -2.48933, -1.50092, 1.52787};

// std::vector<double> prePrePush = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
// std::vector<double> prePush = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
// std::vector<double> push = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};


namespace forwardPlans
{
    //Plans:
    moveit::planning_interface::MoveGroupInterface::Plan firstPlan;

    //Idle to pre-grasp:
    moveit::planning_interface::MoveGroupInterface::Plan idleToPreSmall;
    moveit::planning_interface::MoveGroupInterface::Plan idleToPreMedium;
    moveit::planning_interface::MoveGroupInterface::Plan idleToPreLarge;

    //Pre-grasp to grasp:
    moveit::planning_interface::MoveGroupInterface::Plan graspPreSmall;
    moveit::planning_interface::MoveGroupInterface::Plan graspPreMedium;
    moveit::planning_interface::MoveGroupInterface::Plan graspPreLarge;

    //Pre-grasp to boxes:
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxA;
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxB;
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxC;
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxD;

    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxA;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxB;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxC;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxD;

    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxA;
    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxB;
    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxC;
    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxD;

    //Pre-grasp to discard:
    moveit::planning_interface::MoveGroupInterface::Plan smallToDiscard;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToDiscard;
    moveit::planning_interface::MoveGroupInterface::Plan largeToDiscard;

    //Boxes to pre-pre-push:
    moveit::planning_interface::MoveGroupInterface::Plan boxAToPPP;
    moveit::planning_interface::MoveGroupInterface::Plan boxBToPPP;
    moveit::planning_interface::MoveGroupInterface::Plan boxCToPPP;
    moveit::planning_interface::MoveGroupInterface::Plan boxDToPPP;

    //Pre-pre-push to pre-push:
    moveit::planning_interface::MoveGroupInterface::Plan pppTopp;

    //Pre-push to push:
    moveit::planning_interface::MoveGroupInterface::Plan ppToPush;
}

namespace reversePlans
{
    //Plans:
    //Idle to pre-grasp:
    moveit::planning_interface::MoveGroupInterface::Plan idleToPreSmall;
    moveit::planning_interface::MoveGroupInterface::Plan idleToPreMedium;
    moveit::planning_interface::MoveGroupInterface::Plan idleToPreLarge;

    //Pre-grasp to grasp:
    moveit::planning_interface::MoveGroupInterface::Plan graspPreSmall;
    moveit::planning_interface::MoveGroupInterface::Plan graspPreMedium;
    moveit::planning_interface::MoveGroupInterface::Plan graspPreLarge;

    //Pre-grasp to boxes:
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxA;
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxB;
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxC;
    moveit::planning_interface::MoveGroupInterface::Plan smallToBoxD;

    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxA;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxB;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxC;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToBoxD;

    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxA;
    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxB;
    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxC;
    moveit::planning_interface::MoveGroupInterface::Plan largeToBoxD;

    //Pre-grasp to discard:
    moveit::planning_interface::MoveGroupInterface::Plan smallToDiscard;
    moveit::planning_interface::MoveGroupInterface::Plan mediumToDiscard;
    moveit::planning_interface::MoveGroupInterface::Plan largeToDiscard;

    //Boxes to pre-pre-push:
    moveit::planning_interface::MoveGroupInterface::Plan boxAToPPP;
    moveit::planning_interface::MoveGroupInterface::Plan boxBToPPP;
    moveit::planning_interface::MoveGroupInterface::Plan boxCToPPP;
    moveit::planning_interface::MoveGroupInterface::Plan boxDToPPP;

    //Pre-pre-push to pre-push:
    moveit::planning_interface::MoveGroupInterface::Plan pppTopp;

    //Pre-push to push:
    moveit::planning_interface::MoveGroupInterface::Plan ppToPush;
}