//============================================================================
// Copyright   : PerceptIn
//============================================================================
#include <algorithm>
#include <cmath>
#include <glog/logging.h>
#include "occupancy_grid_map.h"

namespace PIRoBot {

PntIdx operator+(const PntIdx &a, const PntIdx &b) {
  return PntIdx(a.x_idx + b.x_idx, a.y_idx + b.y_idx);
}

bool operator==(const PntIdx& a, const PntIdx& b) {
  return a.x_idx == b.x_idx && a.y_idx == b.y_idx;
}

bool operator!=(const PntIdx& a, const PntIdx& b) {
  return !(a == b);
}

bool operator<(const PntIdx &a, const PntIdx &b) {
  if (a.y_idx == b.y_idx) {
    return a.x_idx < b.x_idx;
  }
  return a.y_idx < b.y_idx;
}

OccupancyGridMap::OccupancyGridMap(const double& res, const double& sq_size) :
    res_(res),
    sq_size_(sq_size),
    idx_max_(static_cast<size_t>(sq_size / res + 0.5)),
    ogm_(idx_max_ * idx_max_, 0.5) {
  CHECK_GT(res_, 0.0);
  CHECK_GT(sq_size_, 0.0);
  CHECK_GT(idx_max_, 1);
}

OccupancyGridMap::OccupancyGridMap(const OccupancyGridMap& ogm) :
    res_(ogm.res_),
    sq_size_(ogm.sq_size_),
    idx_max_(ogm.idx_max_),
    ogm_(ogm.ogm_) {}

OccupancyGridMap::~OccupancyGridMap() {}
void OccupancyGridMap::InitOgm(double val) {
  for(auto it = ogm_.begin(); it != ogm_.end(); it++) {
    *it = val;
  }
}
const std::vector<double>& OccupancyGridMap::ogm() const {
  return ogm_;
}

const double& OccupancyGridMap::res() const {
  return res_;
}

const double& OccupancyGridMap::sq_size() const {
  return sq_size_;
}

const size_t& OccupancyGridMap::idx_max() const {
  return idx_max_;
}

double OccupancyGridMap::GetVal(
    const size_t& x_idx, const size_t& y_idx) const {
  if (x_idx >= idx_max_ || y_idx >= idx_max_) {
    return -1.0;
  }

  return ogm_[OgmIndex(x_idx, y_idx)];
}

double OccupancyGridMap::GetVal(const PntIdx &idx) const {
  return GetVal(idx.x_idx, idx.y_idx);
}

double OccupancyGridMap::GetArea() const {
  return res_ * res_ * static_cast<double>(ogm_.size());
}

cv::Rect2d OccupancyGridMap::GetRoi() const {
  return cv::Rect2d(-sq_size_ / 2.0, -sq_size_ / 2.0, sq_size_, sq_size_);
}

bool OccupancyGridMap::GetCenter(
    const size_t x_idx, const size_t y_idx, double *x, double *y) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(y);

  if (x_idx >= idx_max_ || y_idx >= idx_max_) {
    return false;
  }

  const double half_sq_size = sq_size_ / 2.0;

  *x = (static_cast<double>(x_idx) + 0.5) * res_ - half_sq_size;
  *y = (static_cast<double>(y_idx) + 0.5) * res_ - half_sq_size;
  return true;
}

bool OccupancyGridMap::GetCenter(const PntIdx &idx, cv::Point2d *xy) const {
  CHECK_NOTNULL(xy);
  return GetCenter(idx.x_idx, idx.y_idx, &(xy->x), &(xy->y));
}

std::vector<double>& OccupancyGridMap::ogm_mutable() {
  return ogm_;
}

/*
The x and y are in meters in world coordinate from pose side.
This function is used in update_ogm and outsiders shouldn't
need to explicitly calculate x_idx and y_idx.
*/
bool OccupancyGridMap::CalcIdx(
    const double x, const double y, size_t* x_idx, size_t* y_idx) const {
  CHECK_NOTNULL(x_idx);
  CHECK_NOTNULL(y_idx);

  const double half_sq_size = sq_size_ / 2.0;

  // always be careful about the boundary
  if (x <= - half_sq_size || x >= half_sq_size
      || y <= - half_sq_size || y >= half_sq_size) {
    return false;
  }

  *x_idx = (x + half_sq_size) / res_;
  *y_idx = (y + half_sq_size) / res_;

  return true;
}

bool OccupancyGridMap::CalcIdx(
    const double x, const double y, PntIdx *idx) const {
  CHECK_NOTNULL(idx);
  return CalcIdx(x, y, &(idx->x_idx), &(idx->y_idx));
}

bool OccupancyGridMap::CalcIdx(const cv::Point2d &xy, PntIdx *idx) const {
  CHECK_NOTNULL(idx);
  return CalcIdx(xy.x, xy.y, &(idx->x_idx), &(idx->y_idx));
}
bool OccupancyGridMap::CalcCoord(const size_t ix, const size_t iy, double * x, double * y) {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(y);

  if( ix >= idx_max_ || iy >= idx_max_ ) {
    return false;
  }
  
  const double half_sq_size = sq_size_ / 2.0;
  *x = static_cast<double>(ix) * res_ - half_sq_size;
  *y = static_cast<double>(iy) * res_ - half_sq_size;

  return true;
}

bool OccupancyGridMap::CalcCoord(const PntIdx & idx, double * x, double * y) {
  return CalcCoord(idx.x_idx, idx.y_idx, x, y);
}

bool OccupancyGridMap::SetVal(
    const size_t& x_idx, const size_t& y_idx, const double val) {
  if (x_idx >= idx_max_ || y_idx >= idx_max_) {
    return false;
  }
  #ifndef __ANDROID__
  if(val < 0) {
    LOG(WARNING) << "Marking a cell (" << x_idx << ", " << y_idx << ") invalid "
                 << "by setting its value to negative (" << val << ").";
  }
  #endif

  ogm_[OgmIndex(x_idx, y_idx)] = val;

  return true;
}

bool OccupancyGridMap::SetVal(const PntIdx &idx, const double val) {
  return SetVal(idx.x_idx, idx.y_idx, val);
}

/*
This function will generate the list of cells for the ray (or rays),
and calculate the update value for each cell.
*/
bool OccupancyGridMap::GenerateUpdateCells(
    const size_t x_obs_idx, const size_t y_obs_idx,
    const size_t x_cur_idx, const size_t y_cur_idx,
    const size_t radius_,
    std::vector<size_t>* x_idx_ptr, std::vector<size_t>* y_idx_ptr,
    std::vector<double>* val) {
  CHECK_GT(radius_, 0);
  CHECK_NOTNULL(x_idx_ptr);
  CHECK_NOTNULL(y_idx_ptr);
  CHECK_NOTNULL(val);

  // nothing to update so immediately return true
  if (x_obs_idx == x_cur_idx && y_obs_idx == y_cur_idx) {
    return true;
  }
  /*
  LOG(ERROR) << "x_obs/y_obs/x_cur/y_cur - "
             << x_obs_idx << " / " << y_obs_idx << " / "
             << x_cur_idx << " / " << y_cur_idx;
  */

  // STEP 1: calculate the circle area pixels
  // TODO(zzhang): for now give very small probability to the circle area
  int y_min = std::max((int)(y_cur_idx - radius_), 0);
  int y_max = std::min((int)(y_cur_idx + radius_), (int)(idx_max_ - 1));
  for (int i = y_min; i <= y_max; i++) {
    int x_length =
        (int)std::sqrt((double)(radius_ * radius_
        - (i - y_cur_idx) * (i - y_cur_idx))) + 1;
    int x_min = std::max((int)(x_cur_idx - x_length), 0);
    int x_max = std::min((int)(x_cur_idx + x_length), (int)(idx_max_ - 1));
    for (int j = x_min; j <= x_max; j++) {
      x_idx_ptr->push_back(j);
      y_idx_ptr->push_back(i);
      val->push_back(0.001);
    }
  }

  // STEP 2: calculate the ray pixels
  // TODO(zzhang): for now give very small probability to the ray pixels
  size_t y_delta = (y_obs_idx > y_cur_idx)
      ? (y_obs_idx - y_cur_idx) : (y_cur_idx - y_obs_idx);
  size_t x_delta = (x_obs_idx > x_cur_idx)
      ? (x_obs_idx - x_cur_idx) : (x_cur_idx - x_obs_idx);
  if (y_delta < x_delta) {
    double k
        = (static_cast<double>(y_obs_idx) - static_cast<double>(y_cur_idx))
        / (static_cast<double>(x_obs_idx) - static_cast<double>(x_cur_idx));
    LOG(ERROR) << "case 1 with k - " << k;
    if (x_cur_idx < x_obs_idx) {
      // We can exclude the robot itself's pixels but it's OK.
      size_t x_start = x_cur_idx; // + radius_;
      while (x_start < x_obs_idx) {
        // TODO(zzhang): There may be 1 pixel off when reaching the end
        // but given the obstacle pixels will be updated any way after
        // STEP 2, so it's OK.
        size_t y_step = (size_t)((static_cast<double>(x_start)
            - static_cast<double>(x_cur_idx)) * k) + y_cur_idx;
        x_idx_ptr->push_back(x_start);
        y_idx_ptr->push_back(y_step);
        val->push_back(0.001);
        x_start++;
      }
    } else {
      size_t x_start = x_cur_idx; // - radius_;
      while (x_start > x_obs_idx) {
        size_t y_step = (size_t)((static_cast<double>(x_start)
            - static_cast<double>(x_cur_idx)) * k) + y_cur_idx;
        x_idx_ptr->push_back(x_start);
        y_idx_ptr->push_back(y_step);
        val->push_back(0.001);
        x_start--;
      }
    }
  } else {
    double k = (static_cast<double>(x_obs_idx) - static_cast<double>(x_cur_idx))
        / (static_cast<double>(y_obs_idx) - static_cast<double>(y_cur_idx));
    LOG(ERROR) << "case 2 with k - " << k;
    if (y_cur_idx < y_obs_idx) {
      size_t y_start = y_cur_idx; // + radius_;
      while (y_start < y_obs_idx) {
        size_t x_step = (size_t)((static_cast<double>(y_start)
            - static_cast<double>(y_cur_idx)) * k) + x_cur_idx;
        x_idx_ptr->push_back(x_step);
        y_idx_ptr->push_back(y_start);
        val->push_back(0.001);
        y_start++;
      }
    } else {
      size_t y_start = y_cur_idx; // - radius_;
      while (y_start > y_obs_idx) {
        size_t x_step = (size_t)((static_cast<double>(y_start)
            - static_cast<double>(y_cur_idx)) * k) + x_cur_idx;
        x_idx_ptr->push_back(x_step);
        y_idx_ptr->push_back(y_start);
        val->push_back(0.001);
        y_start--;
      }
    }
  }

  // STEP 3: calculate the obstacle pixels
  // TODO(zzhang): for now give a 3x3 filter with the same high probability
  // of being occupied
  int y_tmp_min = std::max((int)(y_obs_idx - 1), 0);
  int y_tmp_max = std::min((int)(y_obs_idx + 1), (int)(idx_max_ - 1));
  int x_tmp_min = std::max((int)(x_obs_idx - 1), 0);
  int x_tmp_max = std::min((int)(x_obs_idx + 1), (int)(idx_max_ - 1));
  for (int i = x_tmp_min; i <= x_tmp_max; i++) {
    for (int j = y_tmp_min; j <= y_tmp_max; j++) {
      x_idx_ptr->push_back(i);
      y_idx_ptr->push_back(j);
      val->push_back(0.999);
    }
  }

  return true;
}

/*
The x and y are in meters in world coordinate from pose side.
Outsiders shouldn't need to know anything about x_idx and y_idx.
*/
bool OccupancyGridMap::UpdateOgm(
    const double x_obs, const double y_obs,
    const double x_cur, const double y_cur,
    const double diameter) {
  CHECK_GT(diameter, -1e-6);

  // Get the observer's radius in terms of number of cells.
  const size_t radius =
      static_cast<size_t>(std::max((int)(diameter / 2.0 / res_), 1));

  size_t x_obs_idx, y_obs_idx;
  if (!CalcIdx(x_obs, y_obs, &x_obs_idx, &y_obs_idx)) {
    return false;
  }
  size_t x_cur_idx, y_cur_idx;
  if (!CalcIdx(x_cur, y_cur, &x_cur_idx, &y_cur_idx)) {
    return false;
  }

  /*
  y
  ^
  |
  |  (0,0) in distance
  |
  0---------> x
  in pixel
  The saving order in ogm_ is increasing both x and y.
  */
  std::vector<size_t> x_idx;
  std::vector<size_t> y_idx;
  std::vector<double> val;
  if (!GenerateUpdateCells(
          x_obs_idx, y_obs_idx, x_cur_idx, y_cur_idx, radius,
          &x_idx, &y_idx, &val)) {
    return false;
  }
  CHECK_EQ(x_idx.size(), y_idx.size());
  CHECK_EQ(x_idx.size(), val.size());
  for (int i = 0; i < val.size(); i++) {
    ogm_[OgmIndex(x_idx[i], y_idx[i])] = val[i];
  }

  return true;
}

cv::Mat OccupancyGridMap::Visualize() const {
  cv::Mat img(idx_max_, idx_max_, CV_8UC1);
  for (int r = 0; r < img.rows; r++) {
    for (int c = 0; c < img.cols; c++) {
      const size_t i = (img.rows - r - 1) * img.cols + c;
      CHECK_LT(i, ogm_.size());
      const double val = std::max(std::min(ogm_[i], 1.0), 0.0);
      img.at<uchar>(r, c) = 255 - static_cast<uchar>(val * 255.0);
    }
  }
  return img;
}

bool OccupancyGridMap::DilateOgm(
    const size_t iter, std::vector<double>* ogm_dilate_ptr,
    const double value_occupied) {
  CHECK_NOTNULL(ogm_dilate_ptr);
  // first copy ogm_ to ogm_dilate
  *ogm_dilate_ptr = ogm_;

  for (size_t k = 0; k < iter; k++) {
    std::vector<double> tmp_dilating_map(*ogm_dilate_ptr);
    for (size_t i = 1 ; i < idx_max_ - 1; i++) {
      for (size_t j = 1 ; j < idx_max_ - 1; j++) {
        // dilate the "4-neighbors"
        if ((*ogm_dilate_ptr)[i * idx_max_ + j] >= value_occupied) {
          // only update if there is "more obstacle"
          if (tmp_dilating_map[i * idx_max_ + j]
              > tmp_dilating_map[i * idx_max_ + j + 1]) {
            tmp_dilating_map[i * idx_max_ + j + 1]
                = tmp_dilating_map[i * idx_max_ + j];
          }
          if (tmp_dilating_map[i * idx_max_ + j]
              > tmp_dilating_map[i * idx_max_ + j - 1]) {
            tmp_dilating_map[i * idx_max_ + j - 1]
                = tmp_dilating_map[i * idx_max_ + j];
          }
          if (tmp_dilating_map[i * idx_max_ + j]
              > tmp_dilating_map[(i + 1) * idx_max_ + j]) {
            tmp_dilating_map[(i + 1) * idx_max_ + j]
                = tmp_dilating_map[i * idx_max_ + j];
          }
          if (tmp_dilating_map[i * idx_max_ + j]
              > tmp_dilating_map[(i - 1) * idx_max_ + j]) {
            tmp_dilating_map[(i - 1) * idx_max_ + j]
                = tmp_dilating_map[i * idx_max_ + j];
          }
        }
      }
    }
    *ogm_dilate_ptr = tmp_dilating_map;
  }

  ogm_ = *ogm_dilate_ptr;

  return true;
}

}  // namespace PIRoBot
