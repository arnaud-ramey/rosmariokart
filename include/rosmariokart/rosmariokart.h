#ifndef ROSMARIOKART_H
#define ROSMARIOKART_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
// c includes
#include <stdio.h>
#include <sys/time.h>

//#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
#define DEBUG_PRINT(...)   ROS_WARN(__VA_ARGS__)

enum GameStatus {
  GAME_STATUS_WAITING   = 0, // waiting for robots or joypads
  GAME_STATUS_COUNTDOWN = 1, // countdown + lakitu
  GAME_STATUS_RACE      = 2, // race time
  GAME_STATUS_RACE_OVER = 3, // players cant move anymore
  NGAME_STATUES         = 4
};
enum Item {
  ITEM_NONE            = 0,
  ITEM_BOO             = 1,
  ITEM_GOLDENMUSHROOM  = 2,
  ITEM_LIGHTNING       = 3,
  ITEM_MIRROR          = 4,
  ITEM_MUSHROOM        = 5,
  ITEM_REDSHELL        = 6,
  ITEM_REDSHELL2       = 7,
  ITEM_REDSHELL3       = 8,
  ITEM_STAR            = 9,
  ITEM_ROULETTE        = 10,
  NITEMS               = 11,
};
enum Curse {
  CURSE_NONE               = 0,
  CURSE_BOO                = 1,
  CURSE_DUD_START          = 2, // missed rocket start - http://www.mariowiki.com/Rocket_Start
  CURSE_GOLDENMUSHROOM     = 3,
  CURSE_LIGHTNING          = 4,
  CURSE_MIRROR             = 5,
  CURSE_MUSHROOM           = 6,
  CURSE_REDSHELL_COMING    = 7,
  CURSE_REDSHELL_HIT       = 8,
  CURSE_ROCKET_START       = 9, // missed rocket start - http://www.mariowiki.com/Rocket_Start
  CURSE_STAR               = 10,
  CURSE_TIMEBOMB_COUNTDOWN = 11,
  CURSE_TIMEBOMB_HIT       = 12,
  NCURSES                  = 13
};
enum JoypadStatus {
  JOYPAD_NEVER_RECEIVED = 0,
  JOYPAD_OK             = 1,
  JOYPAD_BAD_AXES_NB    = 2,
  JOYPAD_BAD_BUTTONS_NB = 3,
  JOYPAD_TIMEOUT        = 4,
  NJOYPAD_STATUSES      = 5
};
enum RobotStatus {
  ROBOT_NEVER_RECEIVED = 0,
  ROBOT_OK             = 1,
  ROBOT_TIMEOUT        = 2,
  NROBOT_STATUSES      = 3
};
enum LakituStatus {
  LAKITU_INVISIBLE      = 0,
  LAKITU_LIGHT0         = 1,
  LAKITU_LIGHT1         = 2,
  LAKITU_LIGHT2         = 3,
  LAKITU_LIGHT3         = 4,
  LAKITU_RACE_OVER      = 5,
  NLAKITU_STATUSES      = 6
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline bool is_real_item(Item i) {
  return (i != ITEM_NONE && i != ITEM_ROULETTE);
}

////////////////////////////////////////////////////////////////////////////////

inline Item random_item() {
  //return ITEM_LIGHTNING; // debug test
  Item i = (Item) (rand() % NITEMS);
  return (is_real_item(i) ? i : random_item());
}

////////////////////////////////////////////////////////////////////////////////

inline bool imread_vector(std::vector<cv::Mat3b> & v,
                          unsigned int idx,
                          const std::string & filename,
                          unsigned int width = -1) {
  if (idx >= v.size()) {
    ROS_WARN("Index '%i' out of range [0, %i]", idx, v.size());
    return false;
  }
  std::string fullfilename = filename;
  cv:: Mat m = cv::imread(fullfilename , cv::IMREAD_COLOR);
  if (m.empty()) {
    ROS_WARN("Could not load image '%s'", fullfilename.c_str());
    return false;
  }
  m.copyTo(v[idx]);
  if (width > 0)
    cv::resize(v[idx], v[idx], cv::Size(width , width ));
  return true;
} // end imread_vector()

////////////////////////////////////////////////////////////////////////////////

template<class Img, class Rect>
inline Rect img_bbox(const Img & i) {
  return Rect(0, 0, i.cols, i.rows);
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

//! \retun true if \a small is included into \a big
template<class Rect>
inline bool bboxes_included(const Rect & big, const Rect & small) {
  if (big.x > small.x)
    return 0;
  if (big.y > small.y)
    return 0;
  if (big.x + big.width < small.x + small.width)
    return 0;
  if (big.y + big.height < small.y + small.height)
    return 0;
  return 1;
}

////////////////////////////////////////////////////////////////////////////////

template<class Rect, class Img>
inline bool bbox_included_image(const Rect & r, const Img & img) {
  return bboxes_included(img_bbox<Img, Rect>(img), r);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 Determine the ROIs for pasting an image onto another.
 \param topaste
 \param dst
 \param topaste_x, topaste_y
    The coordinates where to paste "topaste" into "dst"
 \param topaste_roi (out)
   If width = 0, means "topaste" would be out of "dst" (= nothing to do).
   Otherwise, the ROI of "topaste" that needs to be copied.
 \param dst_roi (out)
   If width = 0, means "topaste" would be out of "dst" (= nothing to do).
   Otherwise, the ROI of "dst" where the ROI of "topaste" needs to be pasted.
*/
template<class Image>
inline void paste_img_compute_rois(const Image & topaste, const Image & dst,
                                   int topaste_x, int topaste_y,
                                   cv::Rect & topaste_roi, cv::Rect & dst_roi) {
  dst_roi = cv::Rect(0, 0, dst.cols, dst.rows);
  topaste_roi = cv::Rect(topaste_x, topaste_y, topaste.cols, topaste.rows);

  // sizes check
  if (topaste.cols == 0 || topaste.rows == 0 || dst.rows == 0 || dst.cols == 0) {
    dst_roi.width = dst_roi.height = topaste_roi.width = topaste_roi.height = 0;
    return;
  }

  cv::Rect inter_roi = (dst_roi & topaste_roi);
  // disjoint rectangles => do nothing
  if (inter_roi.width <= 0 || inter_roi.height <= 0) {
    dst_roi.width = dst_roi.height = topaste_roi.width = topaste_roi.height = 0;
    return;
  }
  // the corresponding width and height in input images will be the one of the inter
  dst_roi.width = inter_roi.width;
  dst_roi.height = inter_roi.height;
  topaste_roi.width = inter_roi.width;
  topaste_roi.height = inter_roi.height;
  // adjust x, depending on which side of the left edge of "dst" we paste
  if (topaste_x >= 0) { // we paste on the right side of the left edge of "dst"
    dst_roi.x = inter_roi.x;
    topaste_roi.x = 0;
  }
  else { // we paste on the left side of "dst"
    dst_roi.x = 0;
    topaste_roi.x = -topaste_x;
  }
  // same thing with y
  if (topaste_y >= 0) {
    dst_roi.y = inter_roi.y;
    topaste_roi.y = 0;
  }
  else {
    dst_roi.y = 0;
    topaste_roi.y = -topaste_y;
  }
} // end paste_img_compute_rois()

////////////////////////////////////////////////////////////////////////////////

/*! copies to \arg dst
    matrix elements of \arg topaste that are
    marked with non-zero \arg mask elements. */
template<class Image>
inline void paste_img(const Image & topaste, Image & dst,
                      int topaste_x, int topaste_y,
                      const cv::Mat* mask = NULL,
                      const std::string title = "",
                      const cv::Scalar title_color = cv::Scalar(0, 0, 255)) {
  //  printf("paste_img()\n");

  cv::Rect topaste_roi, dst_roi;
  paste_img_compute_rois(topaste, dst, topaste_x, topaste_y, topaste_roi, dst_roi);
  //  printf("topaste_roi:(%ix%i)+(%ix%i), dst_roi:(%ix%i)+(%ix%i)\n",
  //         topaste_roi.x, topaste_roi.y, topaste_roi.width, topaste_roi.height,
  //         dst_roi.x, dst_roi.y, dst_roi.width, dst_roi.height);
  if (topaste_roi.width == 0 || topaste_roi.height == 0
      || dst_roi.width == 0 || dst_roi.height == 0)
    return;

  // make the proper pasting
  Image topaste_sub = topaste(topaste_roi),
      dst_sub = dst(dst_roi);
  bool use_mask = (mask != NULL);
  if (use_mask && !bbox_included_image(topaste_roi, *mask)) {
    printf("paste_img(): incorrect dims of mask (%i,%i)\n",
           mask->cols, mask->rows);
    use_mask = false;
  }
  if (use_mask) {
    const cv::Mat mask_sub = (*mask)(topaste_roi);
    topaste_sub.copyTo(dst_sub, mask_sub);
  }
  else
    topaste_sub.copyTo(dst_sub);

  // put the title if needed
  if (title != "")
    cv::putText(dst, title,
                cv::Point(dst_roi.x + 5, dst_roi.y + dst_roi.height - 10),
                cv::FONT_HERSHEY_PLAIN, 1, title_color);
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline std::string cast2string(const _T in) {
  std::ostringstream ans;
  ans << in;
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

class Timer {
public:
  typedef float Time;
  static const Time NOTIME = -1;
  Timer() { reset(); }
  virtual inline void reset() {
    gettimeofday(&start, NULL);
  }
  //! get the time since ctor or last reset (milliseconds)
  virtual inline Time getTimeSeconds() const {
    struct timeval end;
    gettimeofday(&end, NULL);
    return (Time) (// seconds
                   (end.tv_sec - start.tv_sec)
                   +
                   // useconds
                   (end.tv_usec - start.tv_usec)
                   / 1E6);
  }
private:
  struct timeval start;
}; // end class Timer

#endif // ROSMARIOKART_H
