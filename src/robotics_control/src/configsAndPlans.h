//Configs:
std::vector<double> idleConfig = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};

std::vector<double> preGraspSmall = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};
std::vector<double> preGraspMedium = {1.03682, -0.663927, 1.41798, -2.64278, -1.51245, -0.164062};
std::vector<double> preGraspLarge = {0.894718, -0.697766, 1.41983, -2.63806, -1.48795, -0.164074};

std::vector<double> graspSmall = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};
std::vector<double> graspMedium = {1.03682, -0.663927, 1.41798, -2.64278, -1.51245, -0.164062};
std::vector<double> graspLarge = {0.894718, -0.697766, 1.41983, -2.63806, -1.48795, -0.164074};

std::vector<double> aboveBoxA = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
std::vector<double> aboveBoxB = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
std::vector<double> aboveBoxD = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
std::vector<double> aboveBoxC = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};

std::vector<double> aboveDiscard = {1.41853, -0.659879, 1.01533, -1.89876, -1.54768, -0.164098};

std::vector<double> prePrePush = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
std::vector<double> prePush = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};
std::vector<double> push = {1.00412, -1.14127, 1.99322, -2.39901, -1.6049, -0.164074};


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