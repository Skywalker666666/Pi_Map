//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include "aux_func.h"

void mapCoord2ImgCoord(const cv::Point3d & ptMap, cv::Point2i & ptImg, const double img2mapScale, const cv::Point2d & imgOriginOffsetInMap) {
  ptImg.x = ( ptMap.x - imgOriginOffsetInMap.x ) / img2mapScale;
  ptImg.y = -( ptMap.y - imgOriginOffsetInMap.y ) / img2mapScale;
}

void restrict_pi_rad(double & angle) {
  if(angle > M_PI)
    angle -= 2*M_PI;
  if(angle < -M_PI)
    angle += 2*M_PI;
}
