#include <ros/ros.h>
#include <human_follow_rknn/TargetInfo.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include "human_follow_rknn/rknn_yolo.h"
#include <deque>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_tracker_node");
  ros::NodeHandle nh("~");

  ros::Publisher pub = nh.advertise<human_follow_rknn::TargetInfo>("person_target", 1);

  int cam_index, width, height, fps;
  nh.param("camera_index", cam_index, 0);
  nh.param("width", width, 640);
  nh.param("height", height, 480);
  nh.param("fps", fps, 30);

  std::string model_path;
  nh.param<std::string>("model_path", model_path,
                        "/home/youruser/FAST_WS/model/yolov5n_rk3588.rknn");
  int person_class_id;
  nh.param("person_class_id", person_class_id, 0); // COCO 中 0 是 person

  int miss_patience;
  nh.param("miss_patience", miss_patience, 20);

  RknnYoloV5 yolo;
  if(!yolo.init(model_path, 80, 640))
  {
    ROS_ERROR("Failed to init RKNN YOLO, path=%s", model_path.c_str());
    return -1;
  }

  cv::VideoCapture cap(cam_index);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, fps);

  if(!cap.isOpened())
  {
    ROS_ERROR_STREAM("Cannot open camera index " << cam_index);
    return -1;
  }

  cv::namedWindow("tracker_view", cv::WINDOW_NORMAL);

  cv::Ptr<cv::Tracker> tracker;
  bool has_target = false;
  cv::Rect2d track_box;
  std::deque<cv::Point> history;
  const int HISTORY_LEN = 60;

  int miss_count = 0;

  ros::Rate rate(fps);

  while(ros::ok())
  {
    cv::Mat frame;
    if(!cap.read(frame))
    {
      ROS_WARN("Failed to read frame");
      break;
    }

    int H = frame.rows;
    int W = frame.cols;
    cv::Mat vis = frame.clone();

    // 键盘交互
    char key = (char)cv::waitKey(1);
    if(key == 'q' || key == 'Q') break;
    if(key == 's' || key == 'S')
    {
      cv::Rect2d roi = cv::selectROI("tracker_view", frame,
                                     false, false);
      if(roi.width > 0 && roi.height > 0)
      {
        tracker = cv::TrackerCSRT::create();
        tracker->init(frame, roi);
        has_target = true;
        track_box = roi;
        history.clear();
        miss_count = 0;
      }
      else
      {
        ROS_WARN("Invalid ROI");
      }
    }

    human_follow_rknn::TargetInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.target_acquired = false;
    msg.u_norm = 0.0;
    msg.v_norm = 0.0;
    msg.box_height_norm = 0.0;
    msg.confidence = 0.0;

    if(has_target && tracker)
    {
      bool ok = tracker->update(frame, track_box);
      if(ok)
      {
        miss_count = 0;
        cv::rectangle(vis, track_box, cv::Scalar(0,255,255), 2);

        cv::Point c(track_box.x + track_box.width * 0.5,
                    track_box.y + track_box.height * 0.5);
        history.push_back(c);
        if((int)history.size() > HISTORY_LEN)
          history.pop_front();

        for(size_t i = 1; i < history.size(); ++i)
        {
          cv::line(vis, history[i-1], history[i],
                   cv::Scalar(0,255,255), 2);
        }
        cv::circle(vis, c, 4, cv::Scalar(0,255,255), -1);

        float u_norm = (c.x - W * 0.5f) / (W * 0.5f);
        float v_norm = (c.y - H * 0.5f) / (H * 0.5f);
        float h_norm = track_box.height / (float)H;

        msg.target_acquired = true;
        msg.u_norm = u_norm;
        msg.v_norm = v_norm;
        msg.box_height_norm = h_norm;
        msg.confidence = 1.0;

      }
      else
      {
        miss_count++;
        if(miss_count > miss_patience)
        {
          // 用 RKNN YOLO 再捕获
          std::vector<Detection> dets;
          if(yolo.infer(frame, dets, 0.3f, 0.45f))
          {
            cv::Point last_center(track_box.x + track_box.width*0.5,
                                  track_box.y + track_box.height*0.5);
            double best_dist = 1e9;
            cv::Rect2d best_box;
            bool found = false;

            for(const auto& d : dets)
            {
              if(d.class_id != person_class_id) continue;
              cv::Point c(d.box.x + d.box.width/2,
                          d.box.y + d.box.height/2);
              double dist = cv::norm(c - last_center);
              if(dist < best_dist)
              {
                best_dist = dist;
                best_box = d.box;
                found = true;
              }
            }

            if(found)
            {
              tracker = cv::TrackerCSRT::create();
              tracker->init(frame, best_box);
              track_box = best_box;
              history.clear();
              miss_count = 0;
              has_target = true;
            }
            else
            {
              has_target = false;
            }
          }
          else
          {
            has_target = false;
          }
        }
      }
    }

    if(!has_target)
    {
      // 没目标时 msg 保持 target_acquired=false
    }

    pub.publish(msg);

    cv::putText(vis, "S: select ROI, Q: quit", cv::Point(10,30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0,255,0), 2);
    cv::imshow("tracker_view", vis);

    ros::spinOnce();
    rate.sleep();
  }

  cap.release();
  cv::destroyAllWindows();
  return 0;
}
