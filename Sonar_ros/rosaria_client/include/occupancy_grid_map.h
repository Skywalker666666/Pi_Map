//============================================================================
// Copyright   : PerceptIn
//============================================================================

#ifndef OCCUPANCY_GRID_MAP_H
#define OCCUPANCY_GRID_MAP_H

#include <vector>

#include <opencv2/core.hpp>

namespace PIRoBot {

/**
 * Index of a cell in the occupancy grid.
 */
struct PntIdx {
  /// Horizontal index.
  size_t x_idx;
  /// Vertical index.
  size_t y_idx;

  PntIdx(): x_idx(0), y_idx(0) {}
  PntIdx(const size_t x_idx, const size_t y_idx) : x_idx(x_idx), y_idx(y_idx) {}
};

/**
 * @brief Add an offset to a cell index.
 * @note idx_out = idx_1 + indx_2.
 */
PntIdx operator+(const PntIdx &a, const PntIdx &b);

/**
 * @brief Define operator for comparing two cell index.
 */
bool operator==(const PntIdx& a, const PntIdx& b);

/**
 * @brief Define operator for comparing two cell index.
 */
bool operator!=(const PntIdx& a, const PntIdx& b);

/**
 * @brief Define operator to support std::map or std::set.
 */
bool operator<(const PntIdx &a, const PntIdx &b);

/**
 * Data structure for an occupancy grid.
 *
 * The occupancy grid covers a square of a 2d metric space, where the center is
 * at (0, 0) of the metric space. Each cell in the grid is also a square. The
 * grid has the same number of cells in both horizontal and vertical directions.
 * The x-axis of the metric space is pointing to the right,a
 */
class OccupancyGridMap {
 public:
  /**
   * @brief Constructor.
   * @param res Size of the occupancy grid.
   * @param sq_size Size of a single cell in the occupancy grid.
   * @warning #res must be larger than #sq_size.
   */
  OccupancyGridMap(const double& res, const double& sq_size);
  /**
   * @brief Copy constructor.
   */
  OccupancyGridMap(const OccupancyGridMap& ogm);
  virtual ~OccupancyGridMap();
  void InitOgm(double val);

  // Getters.
  const std::vector<double>& ogm() const;
  const double& res() const;
  const double& sq_size() const;
  const size_t& idx_max() const;  /// Maximum index in both dimensions.
  // Setters.
  std::vector<double>& ogm_mutable();
  /**
   * @brief Retrieve the value of a cell in the grid from its 2-d index.
   * @param x_idx Index of the cell along the horizontal direction.
   * @param y_idx Index of the cell along the vertical direction.
   * @return The value at cell (x_idx, y_idx). Negative value if the index is
   *         outside of the grid.
   */
  double GetVal(const size_t& x_idx, const size_t& y_idx) const;
  /**
   * @brief Retrieve the grid's value from its 2-d index.
   * @param idx Index of the cell.
   * @return The value at cell (x_idx, y_idx). Negative value if the index is
   *         outside of the grid.
   */
  double GetVal(const PntIdx &idx) const;
  /**
   * @return The area that the occupancy grid covers.
   */
  double GetArea() const;
  /**
   * @brief Get the metric value (x, y) of the center of the cell.
   */
  bool GetCenter(
      const size_t x_idx, const size_t y_idx, double *x, double *y) const;
  /**
   * @brief Get the metric value (x, y) of the center of the cell.
   */
  bool GetCenter(const PntIdx &idx, cv::Point2d *xy) const;
  /*
   * @return The ROI that the occupancy grid covered.
   */
  cv::Rect2d GetRoi() const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the occupancy grid.
   * @param x X location of the point in the metric coordinate.
   * @param y Y location of the point in the metric coordinate.
   * @param[out] x_idx Horizontal index of the cell.
   * @param[out] y_idx Vertical index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool CalcIdx(
      const double x, const double y, size_t* x_idx, size_t* y_idx) const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the occupancy grid.
   * @param x X location of the point in the metric coordinate.
   * @param y Y location of the point in the metric coordinate.
   * @param[out] idx Index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool CalcIdx(const double x, const double y, PntIdx *idx) const;
  /**
   * @brief Calculate which cell a point (x, y) in the metric coordinate belongs
   *        to in the occupancy grid.
   * @param xy Location of the point in the metric coordinate.
   * @param[out] idx Index of the cell.
   * @return True if the point (x, y) is within the grid's mapped area.
   */
  bool CalcIdx(const cv::Point2d &xy, PntIdx *idx) const;
  /**
   * @brief Set the value of a cell.
   * @param x_idx Horizontal index of the cell to edit.
   * @param y_idx Vertical index of the cell to edit.
   * @param val Value to set to.
   * @return True if (#x_idx, #y_idx) is a cell in the grid.
   */
  bool CalcCoord(const PntIdx & idx, double * x, double * y);
  bool CalcCoord(const size_t ix, const size_t iy, double * x, double * y);
  bool SetVal(
      const size_t& x_idx, const size_t& y_idx, const double val);
  /**
   * @brief Set the value of a cell.
   * @param idx Index of the cell to edit.
   * @param val Value to set to.
   * @return True if #idx is a cell in the grid.
   */
  bool SetVal(const PntIdx &idx, const double val);
  /**
   * @brief Generate a sequence of cells to update from the location where the
   *        observer is at and the observed point.
   * @note This is an example of updating the cells according to a point
   *       observation.
   * @param x_obs_idx Horizontal index of the observed point.
   * @param y_obs_idx Vertical index of the observed point.
   * @param x_cur_idx Horizontal index of the observer.
   * @param y_cur_idx Vertical index of the observer.
   * @param radius Observer's size in terms of number of cells.
   * @param[out] x_idx_ptr Horizontal indices of the sequence cells to update.
   * @param[out] y_idx_ptr Vertical indices of the sequence cells to update.
   * @param[out] val Values to update to.
   * @return True if the sequence of cells are computed.
   */
  bool GenerateUpdateCells(
      const size_t x_obs_idx, const size_t y_obs_idx,
      const size_t x_cur_idx, const size_t y_cur_idx,
      const size_t radius,
      std::vector<size_t>* x_idx_ptr, std::vector<size_t>* y_idx_ptr,
      std::vector<double>* val);
  /**
   * @brief Update a sequence of cells from the location where the observer is
   *        at and the observed point.
   * @param x_obs X location of the observed point.
   * @param y_obs Y location of the observed point.
   * @param x_cur X location of the observer.
   * @param y_cur Y location of the observer.
   * @param diameter Size of the observer. (Must be non-negative.)
   * @return True if the occupancy grid was updated.
   * @note This is an example of updating the cells according to a point
   *       observation.
   */
  bool UpdateOgm(
      const double x_obs, const double y_obs,
      const double x_cur, const double y_cur,
      const double diameter);
  /**
   * @brief Visualize the occupancy grid as a cv::Mat image. Each pixel in the
   *        image represents a cell. A cell with value 1.0 is marked as white,
   *        and 0.0 is marked as black.
   * @return The image showing the values in the grid.
   */
  cv::Mat Visualize() const;
  /**
   * @brief Dilate the occupancy grid according to the size of the observer.
   * @note This is useful for path planning or other algorithms that treated the
   *       observer as a dot in the metric coordinate or a cell in the grid.
   * @param iter Number of pixels to dilate.
   * @param[out] ogm_dilate_ptr Dilated occupancy grid.
   * @param value_occupied (optional) Value of a cell to be considered as
   *                       occupied.
   * @return True if the occupancy grid was dilated.
   */
  bool DilateOgm(const size_t iter,
                 std::vector<double>* ogm_dilate_ptr,
                 const double value_occupied = 0.7);

 protected:
  /**
   * @brief Convert 2d cell index to 1d index in #ogm_.
   * @param x_idx Horizontal index of the cell.
   * @param y_idx Vertical index of the cell.
   * @return Index in #ogm_.
   */
  inline size_t OgmIndex(const size_t x_idx, const size_t y_idx) const {
    return y_idx * idx_max_ + x_idx;
  }

  /// Note, setting #res_, #sq_size_, and #idx_max_ constant until we figure out
  /// how to properly resize an occupancy grid.
  /// Resolution of each cell in the occupancy grid map.
  const double res_;
  /// Length of a side of the occupancy grid map.
  const double sq_size_;
  /// Maximum cell index.
  const size_t idx_max_;
  /// The occupancy grid map. The value is in the range of [0.0, 1.0]. Negative
  /// value means the cell is invalid.
  std::vector<double> ogm_;
};

} // namespace PIRoBot

#endif  // OCCUPANCY_GRID_MAP_H
