//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __aux_func_H__
#define  __aux_func_H__
#include <opencv2/core.hpp>

void mapCoord2ImgCoord(const cv::Point3d & ptMap, cv::Point2i & ptImg, const double img2mapScale, const cv::Point2d & imgOriginOffsetInMap);

void restrict_pi_rad(double & angle);

#endif   /* ----- #ifndef __aux_func_H__  ----- */

