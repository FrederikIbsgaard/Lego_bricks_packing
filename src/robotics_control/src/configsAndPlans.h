//Configs:
// std::vector<double> idleConfig = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};

std::vector<double> preGraspSmall = {0.846311, -1.34897, 2.30981, -3.21674, -1.41184, 1.35423};
std::vector<double> preGraspMedium = {1.32114, -1.2628, 2.17958, -3.20639, -1.72189, 1.73393};
std::vector<double> preGraspLarge = {1.03776, -1.38167, 2.33591, -3.26135, -1.45834, 1.48253};

std::vector<double> graspSmall = {0.866801, -1.10362, 2.11568, -3.21984, -1.37686, 1.35432};
std::vector<double> graspMedium = {1.31588, -1.07263, 2.0361, -3.20941, -1.72159, 1.7339};
std::vector<double> graspLarge = {1.04001, -1.1492, 2.16316, -3.26365, -1.45432, 1.4825};

std::vector<double> aboveBoxA = {0.999368, -1.63055, 2.35564, -2.2854, -1.49426, 1.35431};
// std::vector<double> aboveBoxB = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
// std::vector<double> aboveBoxD = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
// std::vector<double> aboveBoxC = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};

std::vector<double> aboveDiscard = {1.46152, -1.43358, 2.16974, -2.28073, -1.49425, 1.35426};

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