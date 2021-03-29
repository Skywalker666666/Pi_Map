//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __pose_H__
#define  __pose_H__
namespace PIRoBot {
class Pose2d {
  public:
    double x; // meter
    double y; // meter
    double th; // radium
    Pose2d():x(0), y(0), th(0) {};
    Pose2d(double _x, double _y, double _th);
    friend Pose2d operator +(const Pose2d & lhs, const Pose2d & rhs);
    friend Pose2d operator -(const Pose2d & lhs, const Pose2d & rhs);
    friend Pose2d absoluteDifference(const Pose2d& p1, const Pose2d& p2);
    friend Pose2d absoluteSum(const Pose2d& p1, const Pose2d& p2);
    friend double euclideanDist(const Pose2d& p1, const Pose2d& p2);
    friend bool operator ==(const Pose2d & lhs, const Pose2d & rhs); 

};

typedef Pose2d RobotPose;

}
#endif   /* ----- #ifndef __pose_H__  ----- */

