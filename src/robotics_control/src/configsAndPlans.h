//Configs:
// std::vector<double> idleConfig = {1.17439, -0.663472, 1.41558, -2.69422, -1.61225, 0.0158134};

std::vector<double> preGraspSmall = {0.843428, -1.23899, 1.76526, 0.30557, 1.31795, -1.79182};
std::vector<double> preGraspMedium = {1.09693, -1.2536, 1.74324, 0.327678, 1.46964, -1.57507};
std::vector<double> preGraspLarge = {1.4015, -1.18991, 1.66153, 0.338223, 1.71877, -1.26703};

std::vector<double> graspSmall = {0.862274, -1.06465, 1.65372, 0.261691, 1.31075, -1.75682};
std::vector<double> graspMedium = {1.10091, -1.08257, 1.6599, 0.214763, 1.45441, -1.60761};
std::vector<double> graspLarge = {1.36148, -1.00791, 1.51369, 0.319509, 1.70587, -1.42803};

std::vector<double> aboveBoxA = {0.898476, -1.24867, 1.76276, 1.1328, 1.53992, -0.233797};
std::vector<double> aboveBoxB = {1.0585, -1.24929, 1.73833, 1.08831, 1.4586, -0.0862463};
std::vector<double> aboveBoxC = {0.947087, -0.984642, 1.46641, 1.12633, 1.51005, -0.187966};
std::vector<double> aboveBoxD = {1.08616, -0.98475, 1.46544, 1.11344, 1.46546, -0.0236838};

std::vector<double> aboveDiscard = {1.04679, -1.18712, 1.66846, 0.445689, 1.47154, -1.58375};

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