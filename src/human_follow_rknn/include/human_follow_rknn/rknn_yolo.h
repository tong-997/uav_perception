#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// 单个检测结果
struct Detection
{
  cv::Rect box;   // 在原图上的像素框
  float score;    // 置信度
  int   class_id; // 类别 id（COCO 的话 0 = person）
};

class RknnYoloV5
{
public:
  RknnYoloV5();
  ~RknnYoloV5();

  // model_path: yolov5n_rk3588.rknn 的完整路径
  bool init(const std::string& model_path,
            int num_classes = 80,
            int input_size = 640);

  // 输入 BGR 图像，输出 dets（已经映射回原图分辨率）
  bool infer(const cv::Mat& img_bgr, std::vector<Detection>& dets,
             float conf_thres = 0.25f, float nms_thres = 0.45f);

private:
  int rknn_ctx_;
  bool initialized_;

  int num_classes_;
  int input_size_;   // 正方形输入 640/416 等

  int input_width_;
  int input_height_;
  int input_channels_;

  // 典型 yolov5: 三个输出 head
  int output_count_;
  std::vector<int> out_zps_;
  std::vector<float> out_scales_;
  std::vector<std::vector<int>> out_shapes_;

  bool preprocess(const cv::Mat& img_bgr, cv::Mat& resized,
                  float& scale, int& pad_x, int& pad_y);

  void decode_outputs(const std::vector<cv::Mat>& output_mats,
                      float scale, int pad_x, int pad_y,
                      int img_w, int img_h,
                      float conf_thres,
                      std::vector<Detection>& dets);

  void nms(std::vector<Detection>& dets, float nms_thres);

  // 禁用拷贝
  RknnYoloV5(const RknnYoloV5&) = delete;
  RknnYoloV5& operator=(const RknnYoloV5&) = delete;
};
