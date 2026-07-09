#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <memory>
#include <algorithm>
#include <numeric>

extern "C" {
#include "rknn_api.h"
}

using namespace std;
using namespace cv;

struct SystemConfig {
  string model_path = "/home/rpdzkj/fast_ws/midas_v21_small_256.rknn";
  int img_width = 640, img_height = 480;
  double fx = 391.50625, fy = 395.18355, cx = 335.18186, cy = 217.7217;
  int cloud_stride = 4;
  double max_depth_m = 40.0;
  int scale_window = 15;
};

class RknnDepthEstimator {
public:
  explicit RknnDepthEstimator(const SystemConfig& cfg): cfg_(cfg) {
    FILE* fp = fopen(cfg_.model_path.c_str(), "rb");
    if(!fp) throw runtime_error("无法打开RKNN模型: " + cfg_.model_path);
    fseek(fp, 0, SEEK_END); model_len_ = ftell(fp); rewind(fp);
    model_data_ = malloc(model_len_);
    if(fread(model_data_, 1, model_len_, fp) != model_len_) {
      fclose(fp); throw runtime_error("RKNN模型读取失败");
    }
    fclose(fp);
    if(rknn_init(&ctx_, model_data_, model_len_, 0, nullptr) < 0) {
      throw runtime_error("RKNN模型初始化失败");
    }
  }
  ~RknnDepthEstimator() {
    if(ctx_) rknn_destroy(ctx_);
    if(model_data_) free(model_data_);
  }
  Mat infer(const Mat& bgr){
    Mat resized, rgb, f32; resize(bgr, resized, Size(256,256));
    cvtColor(resized, rgb, COLOR_BGR2RGB);
    rgb.convertTo(f32, CV_32FC3, 1.0/255.0);
    rknn_input in{}; in.index=0; in.buf=f32.data; in.size=sizeof(float)*256*256*3;
    in.pass_through=0; in.type=RKNN_TENSOR_FLOAT32; in.fmt=RKNN_TENSOR_NHWC;
    if(rknn_inputs_set(ctx_, 1, &in) < 0) throw runtime_error("rknn_inputs_set 失败");
    if(rknn_run(ctx_, nullptr) < 0) throw runtime_error("rknn_run 失败");
    rknn_output out{}; out.want_float=1;
    if(rknn_outputs_get(ctx_, 1, &out, nullptr) < 0) throw runtime_error("rknn_outputs_get 失败");
    Mat d256(256,256,CV_32FC1, out.buf);
    Mat d01, dsz; normalize(d256,d01,0,1,NORM_MINMAX);
    resize(d01, dsz, Size(cfg_.img_width, cfg_.img_height));
    rknn_outputs_release(ctx_, 1, &out);
    return dsz;
  }
private:
  SystemConfig cfg_;
  rknn_context ctx_{0};
  void* model_data_{nullptr};
  size_t model_len_{0};
};

class EgoDepthFusedNode {
public:
  EgoDepthFusedNode(ros::NodeHandle& nh, ros::NodeHandle& pnh): nh_(nh), pnh_(pnh) {
    // 读取参数
    pnh_.param<string>("model_path",  cfg_.model_path,  cfg_.model_path);
    pnh_.param<int>("img_width",      cfg_.img_width,   cfg_.img_width);
    pnh_.param<int>("img_height",     cfg_.img_height,  cfg_.img_height);
    pnh_.param<double>("fx",          cfg_.fx,          cfg_.fx);
    pnh_.param<double>("fy",          cfg_.fy,          cfg_.fy);
    pnh_.param<double>("cx",          cfg_.cx,          cfg_.cx);
    pnh_.param<double>("cy",          cfg_.cy,          cfg_.cy);
    pnh_.param<int>("cloud_stride",   cfg_.cloud_stride,cfg_.cloud_stride);
    pnh_.param<double>("max_depth_m", cfg_.max_depth_m, cfg_.max_depth_m);
    pnh_.param<int>("scale_window",   cfg_.scale_window,cfg_.scale_window);
    pnh_.param<string>("image_topic", image_topic_, string("/camera640/image_raw"));
    pnh_.param<string>("odom_topic",  odom_topic_,  string("/vins/odometry"));
    pnh_.param<string>("frame_id",    frame_id_,    string("default_cam_optical"));

    // 估计器
    try {
      estimator_.reset(new RknnDepthEstimator(cfg_));
    } catch (const std::exception& e) {
      ROS_FATAL("%s", e.what());
      throw;
    }

    // pub/sub
    depth_pub_     = nh_.advertise<sensor_msgs::Image>("/ego/depth_image", 1);
    rel_depth_pub_ = nh_.advertise<sensor_msgs::Image>("/ego/depth_relative", 1);
    cloud_pub_     = nh_.advertise<sensor_msgs::PointCloud2>("/ego/points", 1);

    image_sub_ = nh_.subscribe(image_topic_, 1, &EgoDepthFusedNode::onImage, this);
    odom_sub_  = nh_.subscribe(odom_topic_,  50, &EgoDepthFusedNode::onOdom, this);

    ROS_INFO("订阅 [%s], [%s]；发布 [/ego/depth_image], [/ego/depth_relative], [/ego/points]",
             image_topic_.c_str(), odom_topic_.c_str());
  }

  void spin() { ros::spin(); }

private:
  struct Pose { ros::Time stamp; cv::Matx33d R; cv::Vec3d t; };

  static cv::Matx33d quatToR(double w,double x,double y,double z){
    return cv::Matx33d(
      1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w),
      2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w),
      2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y));
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg){
    Pose p; p.stamp = msg->header.stamp;
    const auto &q = msg->pose.pose.orientation;
    const auto &tt= msg->pose.pose.position;
    p.R = quatToR(q.w,q.x,q.y,q.z);
    p.t = cv::Vec3d(tt.x, tt.y, tt.z);
    last_odom2_ = last_odom1_;
    last_odom1_ = p;
  }

  bool hasTwoOdom() const {
    return last_odom1_.stamp != ros::Time(0) && last_odom2_.stamp != ros::Time(0);
  }

  void onImage(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (bgr.empty()) return;

    const auto tic = ros::Time::now();
    cv::Mat depth_rel = estimator_->infer(bgr);  // 0~1

    // 相对深度（调试用）
    rel_depth_pub_.publish(cv_bridge::CvImage(msg->header, "32FC1", depth_rel).toImageMsg());

    if (!prev_gray_.empty() && !prev_depth_rel_.empty() && hasTwoOdom()){
      double t_vins = cv::norm(last_odom1_.t - last_odom2_.t);
      double t_vo   = estimateDeltaByPnP(prev_gray_, prev_depth_rel_, bgr);
      if (t_vo > 1e-6 && t_vins > 1e-6) {
        double s_inst = t_vins / t_vo;
        s_hist_.push_back(s_inst);
        while ((int)s_hist_.size() > cfg_.scale_window) s_hist_.pop_front();
        s_ = median(s_hist_);
      }
    }

    cv::Mat depth_m = depth_rel * s_;
    depth_pub_.publish(cv_bridge::CvImage(msg->header, "32FC1", depth_m).toImageMsg());

    publishCloud(bgr, depth_m, msg->header);

    cvtColor(bgr, prev_gray_, COLOR_BGR2GRAY);
    prev_depth_rel_ = depth_rel;

    double ms = (ros::Time::now() - tic).toSec() * 1000.0;
    ROS_DEBUG("Frame total %.1f ms (s=%.3f)", ms, s_);
  }

  double estimateDeltaByPnP(const cv::Mat& prev_gray, const cv::Mat& prev_depth_rel, const cv::Mat& curr_bgr){
    static thread_local Ptr<ORB> orb = ORB::create(1000);
    vector<KeyPoint> k1,k2; Mat d1,d2;
    orb->detectAndCompute(prev_gray, noArray(), k1, d1);
    Mat gray2; cvtColor(curr_bgr, gray2, COLOR_BGR2GRAY);
    orb->detectAndCompute(gray2, noArray(), k2, d2);
    if (d1.empty() || d2.empty()) return 0.0;

    BFMatcher matcher(NORM_HAMMING, true);
    vector<DMatch> matches; matcher.match(d1, d2, matches);
    if (matches.size() < 15) return 0.0;

    vector<Point3f> pts3; vector<Point2f> pts2;
    pts3.reserve(matches.size()); pts2.reserve(matches.size());
    for (auto &m : matches) {
      const Point2f &u1 = k1[m.queryIdx].pt;
      const Point2f &u2 = k2[m.trainIdx].pt;
      int x = lround(u1.x), y = lround(u1.y);
      if (x<=1 || y<=1 || x>=prev_depth_rel.cols-1 || y>=prev_depth_rel.rows-1) continue;
      float z = prev_depth_rel.at<float>(y,x);
      if (!(z>1e-3f)) continue;
      float X = ((u1.x - cfg_.cx)/cfg_.fx) * z;
      float Y = ((u1.y - cfg_.cy)/cfg_.fy) * z;
      pts3.emplace_back(X,Y,z);
      pts2.emplace_back(u2);
    }
    if (pts3.size() < 15) return 0.0;

    Mat K = (Mat_<double>(3,3) << cfg_.fx,0,cfg_.cx, 0,cfg_.fy,cfg_.cy, 0,0,1);
    Mat rvec,tvec; vector<int> inliers;
    bool ok = solvePnPRansac(pts3, pts2, K, noArray(), rvec, tvec, false,
                             100, 3.0, 0.99, inliers, SOLVEPNP_ITERATIVE);
    if (!ok || inliers.size() < 10) return 0.0;
    return norm(tvec);
  }

  static double median(const std::deque<double>& q){
    if (q.empty()) return 1.0;
    vector<double> v(q.begin(), q.end());
    nth_element(v.begin(), v.begin()+v.size()/2, v.end());
    return v[v.size()/2];
  }

  void publishCloud(const cv::Mat& bgr, const cv::Mat& depth_m, const std_msgs::Header& h){
    const int W = depth_m.cols, H = depth_m.rows, step = std::max(1, cfg_.cloud_stride);
    size_t N = (W/step)*(H/step);

    sensor_msgs::PointCloud2 cloud;
    cloud.header = h;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1; cloud.width = N; cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    mod.resize(N);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud,"x"), it_y(cloud,"y"), it_z(cloud,"z");
    sensor_msgs::PointCloud2Iterator<uint8_t> it_r(cloud,"r"), it_g(cloud,"g"), it_b(cloud,"b");

    for(int v=0; v<H; v+=step){
      for(int u=0; u<W; u+=step){
        float Z = depth_m.at<float>(v,u);
        if (!(Z>0 && Z<cfg_.max_depth_m)) {
          *it_x = *it_y = *it_z = std::numeric_limits<float>::quiet_NaN();
        } else {
          *it_x = ((u - cfg_.cx)/cfg_.fx) * Z;
          *it_y = ((v - cfg_.cy)/cfg_.fy) * Z;
          *it_z = Z;
        }
        const cv::Vec3b& c = bgr.at<cv::Vec3b>(v,u);
        *it_r = c[2]; *it_g = c[1]; *it_b = c[0];
        ++it_x; ++it_y; ++it_z; ++it_r; ++it_g; ++it_b;
      }
    }
    cloud_pub_.publish(cloud);
  }

private:
  ros::NodeHandle nh_, pnh_;
  SystemConfig cfg_;
  std::unique_ptr<RknnDepthEstimator> estimator_;

  ros::Subscriber image_sub_, odom_sub_;
  ros::Publisher  depth_pub_, rel_depth_pub_, cloud_pub_;

  cv::Mat prev_gray_, prev_depth_rel_;
  Pose last_odom1_, last_odom2_;
  std::deque<double> s_hist_;
  double s_{1.0};
  std::string image_topic_, odom_topic_, frame_id_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ego_depth_fused_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  EgoDepthFusedNode node(nh, pnh);
  node.spin();
  return 0;
}
