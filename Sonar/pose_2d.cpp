//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "pose_2d.h"
#include <cmath>

namespace PIRoBot {
Pose2d::Pose2d(double _x, double _y, double _th): x(_x), y(_y), th(_th) {

}
Pose2d operator +(const Pose2d & lhs, const Pose2d & rhs) {
  Pose2d ret;
  ret.x = lhs.x + rhs.x;
  ret.y = lhs.y + rhs.y;
  ret.th = lhs.th + rhs.th;
  return ret;
}
Pose2d operator -(const Pose2d & lhs, const Pose2d & rhs) {
  Pose2d ret;
  ret.x = lhs.x - rhs.x;
  ret.y = lhs.y - rhs.y;
  ret.th = lhs.th - rhs.th;
  return ret;
}
Pose2d absoluteDifference(const Pose2d& p1, const Pose2d& p2) {
  Pose2d delta = p1 - p2;
  delta.th = atan2(sin(delta.th), cos(delta.th));
  double s=sin(p2.th), c=cos(p2.th);
  return Pose2d(c*delta.x + s*delta.y, -s*delta.x + c*delta.y, delta.th);
}
Pose2d absoluteSum(const Pose2d& p1, const Pose2d& p2) {
  double s = sin(p1.th), c = cos(p1.th);
  return Pose2d(c*p2.x - s*p2.y, s*p2.x + c*p2.y, p2.th) + p1;
}
double euclideanDist(const Pose2d& p1, const Pose2d& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return sqrt( dx*dx + dy*dy );
}

}
