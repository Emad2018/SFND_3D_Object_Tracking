
#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// Create groups of Lidar points whose projection into the camera falls into the
// same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes,
                         std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor, cv::Mat &P_rect_xx,
                         cv::Mat &R_rect_xx, cv::Mat &RT) {
  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    // pixel coordinates
    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

    vector<vector<BoundingBox>::iterator>
        enclosingBoxes; // pointers to all bounding boxes which enclose the
                        // current Lidar point
    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin();
         it2 != boundingBoxes.end(); ++it2) {
      // shrink current bounding box slightly to avoid having too many outlier
      // points around the edges
      cv::Rect smallerBox;
      smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
      smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
      smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
      smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

      // check wether point is within current bounding box
      if (smallerBox.contains(pt)) {
        enclosingBoxes.push_back(it2);
      }

    } // eof loop over all bounding boxes

    // check wether point has been enclosed by one or by multiple boxes
    if (enclosingBoxes.size() == 1) {
      // add Lidar point to bounding box
      enclosingBoxes[0]->lidarPoints.push_back(*it1);
    }

  } // eof loop over all Lidar points
}

/*
 * The show3DObjects() function below can handle different output image sizes,
 * but the text output has been manually tuned to fit the 2000x2000 size.
 * However, you can make this function work for other sizes too.
 * For instance, to use a 1000x1000 size, adjusting the text positions by
 * dividing them by 2.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize,
                   cv::Size imageSize, bool bWait) {
  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

  for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
    // create randomized color for current 3D object
    cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150),
                                      rng.uniform(0, 150));

    // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end();
         ++it2) {
      // world coordinates
      float xw =
          (*it2).x; // world position in m with x facing forward from sensor
      float yw = (*it2).y; // world position in m with y facing left from sensor
      xwmin = xwmin < xw ? xwmin : xw;
      ywmin = ywmin < yw ? ywmin : yw;
      ywmax = ywmax > yw ? ywmax : yw;

      // top-view coordinates
      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // find enclosing rectangle
      top = top < y ? top : y;
      left = left < x ? left : x;
      bottom = bottom > y ? bottom : y;
      right = right > x ? right : x;

      // draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),
                  cv::Scalar(0, 0, 0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50),
            cv::FONT_ITALIC, 2, currColor);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125),
            cv::FONT_ITALIC, 2, currColor);
  }

  // plot distance markers
  float lineSpacing = 2.0; // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) +
            imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y),
             cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "3D Objects";
  cv::namedWindow(windowName, 1);
  cv::imshow(windowName, topviewImg);

  if (bWait) {
    cv::waitKey(0); // wait for key to be pressed
  }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr,
                              std::vector<cv::DMatch> &kptMatches) {
  double max_dist = 0;
  double min_dist = 100;
  for (int i = 0; i < kptMatches.size(); i++) {
    double dist = kptMatches[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
  }

  for (int match_index = 0; match_index < kptMatches.size(); match_index++) {
    auto point = kptsCurr[kptMatches[match_index].trainIdx];
    auto point_xy = cv::Point(point.pt.x, point.pt.y);
    if (boundingBox.roi.contains(point_xy)) {
      if (kptMatches[match_index].distance <= ((min_dist + max_dist) / 2)) {
        boundingBox.kptMatches.push_back(kptMatches[match_index]);
      }
    }
  }
  std::cout << "kptMatches.size()= " << kptMatches.size() << "\n";
  std::cout << "boundingBox.kptMatches.size()= "
            << boundingBox.kptMatches.size() << "\n";
}

// Compute time-to-collision (TTC) based on keypoint correspondences in
// successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate,
                      double &TTC, cv::Mat *visImg) {
  // compute distance ratios between all matched keypoints
  vector<double> distRatios; // stores the distance ratios for all keypoints
                             // between curr. and prev. frame
  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1;
       ++it1) { // outer kpt. loop

    // get current keypoint and its matched partner in the prev. frame
    cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
    cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end();
         ++it2) { // inner kpt.-loop

      double minDist = 100.0; // min. required distance

      // get next keypoint and its matched partner in the prev. frame
      cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
      cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

      // compute distances and distance ratios
      double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
      double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

      if (distPrev > std::numeric_limits<double>::epsilon() &&
          distCurr >= minDist) { // avoid division by zero

        double distRatio = distCurr / distPrev;
        distRatios.push_back(distRatio);
      }
    } // eof inner loop over all matched kpts
  }   // eof outer loop over all matched kpts

  // only continue if list of distance ratios is not empty
  if (distRatios.size() == 0) {
    TTC = NAN;
    return;
  }

  // STUDENT TASK (replacement for meanDistRatio)
  std::sort(distRatios.begin(), distRatios.end());
  long medIndex = floor(distRatios.size() / 2.0);
  double medDistRatio =
      distRatios.size() % 2 == 0
          ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0
          : distRatios[medIndex]; // compute median dist. ratio to remove
                                  // outlier influence

  double dT = 1 / frameRate;
  TTC = -dT / (1 - medDistRatio);
  // EOF STUDENT TASK
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate,
                     double &TTC) {
  std::vector<double> x;
  double d0, d1 = 0;
  float ratio = .1;
  for (auto point : lidarPointsCurr) {
    x.push_back(point.x);
  }
  sort(x.begin(), x.end());
  d1 = x[(int)x.size() * ratio];
  x.clear();
  for (auto point : lidarPointsPrev) {
    x.push_back(point.x);
  }
  sort(x.begin(), x.end());
  d0 = x[(int)x.size() * ratio];
  cout << "(" << d1 << ", " << frameRate << ", " << d0 << ", " << (d0 - d1)
       << ")\n";
  if (abs(d0 - d1)) {
    TTC = d1 / frameRate / (d0 - d1);
  } else {
    TTC = d1 / frameRate / .001;
  }
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bbBestMatches, DataFrame &prevFrame,
                        DataFrame &currFrame) {
  // loop over all BB match pairs
  for (auto it1 = currFrame.boundingBoxes.begin();
       it1 != currFrame.boundingBoxes.end(); ++it1) {
    std::vector<int> current_box_points;
    int score, match_id = 0;
    int max_score = 20;
    for (int match_index = 0; match_index < matches.size(); match_index++) {
      auto point = currFrame.keypoints[matches[match_index].trainIdx];
      auto point_xy = cv::Point(point.pt.x, point.pt.y);
      if (it1->roi.contains(point_xy)) {
        current_box_points.push_back(match_index);
      }
    }
    cout << "current_box_points= " << current_box_points.size() << "\n";
    for (auto it2 = prevFrame.boundingBoxes.begin();
         it2 != prevFrame.boundingBoxes.end(); ++it2) {
      score = 0;
      for (int index : current_box_points) {
        auto point = prevFrame.keypoints[matches[index].queryIdx];
        auto point_xy = cv::Point(point.pt.x, point.pt.y);
        if (it2->roi.contains(point_xy)) {
          score++;
        }
      }
      if (score > max_score) {
        max_score = score;
        match_id = it2->boxID;
      }
    }
    if (max_score > 0) {
      cout << "max_score= " << max_score << "\n";
      bbBestMatches.insert({match_id, it1->boxID});
    }
  }
}
