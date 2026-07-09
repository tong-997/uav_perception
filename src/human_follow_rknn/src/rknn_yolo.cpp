#include "human_follow_rknn/rknn_yolo.h"
#include <rknn_api.h>
#include <iostream>
#include <cmath>

RknnYoloV5::RknnYoloV5()
  : rknn_ctx_(-1),
    initialized_(false),
    num_classes_(80),
    input_size_(640),
    input_width_(0),
    input_height_(0),
    input_channels_(0),
    output_count_(0)
{
}

RknnYoloV5::~RknnYoloV5()
{
  if(initialized_)
  {
    rknn_destroy(rknn_ctx_);
  }
}

bool RknnYoloV5::init(const std::string& model_path,
                      int num_classes,
                      int input_size)
{
  num_classes_ = num_classes;
  input_size_  = input_size;

  int ret = rknn_init(&rknn_ctx_, (void*)model_path.c_str(), 0, 0, NULL);
  if(ret < 0)
  {
    std::cerr << "rknn_init failed, ret=" << ret << std::endl;
    return false;
  }

  // 查询输入信息
  rknn_input_output_num io_num;
  ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  if(ret < 0)
  {
    std::cerr << "rknn_query IN_OUT_NUM failed, ret=" << ret << std::endl;
    return false;
  }

  // 输入 tensor 信息
  std::vector<rknn_tensor_attr> input_attrs(io_num.input_num);
  for(uint32_t i = 0; i < io_num.input_num; ++i)
  {
    input_attrs[i].index = i;
    input_attrs[i].type  = RKNN_TENSOR_FLOAT32;
    input_attrs[i].size  = 0;
    input_attrs[i].fmt   = RKNN_TENSOR_NHWC;
    char name[256];
    memset(name, 0, sizeof(name));
    input_attrs[i].name  = name;
    ret = rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR,
                     &input_attrs[i], sizeof(input_attrs[i]));
    if(ret < 0)
    {
      std::cerr << "rknn_query INPUT_ATTR failed, ret=" << ret << std::endl;
      return false;
    }
  }

  // 假定只有一个输入
  input_width_  = input_attrs[0].dims[2];
  input_height_ = input_attrs[0].dims[1];
  input_channels_ = input_attrs[0].dims[3];

  // 输出 tensor 信息
  output_count_ = io_num.output_num;
  out_zps_.resize(output_count_);
  out_scales_.resize(output_count_);
  out_shapes_.resize(output_count_);

  for(uint32_t i = 0; i < io_num.output_num; ++i)
  {
    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.index = i;
    attr.type = RKNN_TENSOR_INT8;
    attr.fmt  = RKNN_TENSOR_NCHW;
    char name[256];
    memset(name, 0, sizeof(name));
    attr.name = name;

    ret = rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR,
                     &attr, sizeof(attr));
    if(ret < 0)
    {
      std::cerr << "rknn_query OUTPUT_ATTR failed, ret=" << ret << std::endl;
      return false;
    }

    out_zps_[i] = attr.zp;
    out_scales_[i] = attr.scale;
    std::vector<int> shape(attr.n_dims);
    for(int d = 0; d < attr.n_dims; ++d) shape[d] = attr.dims[d];
    out_shapes_[i] = shape;
  }

  initialized_ = true;
  std::cout << "RknnYoloV5 init done. input: "
            << input_width_ << "x" << input_height_
            << " ch=" << input_channels_
            << " outputs=" << output_count_ << std::endl;
  return true;
}

bool RknnYoloV5::preprocess(const cv::Mat& img_bgr, cv::Mat& out,
                            float& scale, int& pad_x, int& pad_y)
{
  if(img_bgr.empty()) return false;

  int img_w = img_bgr.cols;
  int img_h = img_bgr.rows;
  int dst_w = input_width_;
  int dst_h = input_height_;

  float r = std::min(dst_w * 1.0f / img_w, dst_h * 1.0f / img_h);
  int new_w = static_cast<int>(round(img_w * r));
  int new_h = static_cast<int>(round(img_h * r));
  scale = r;

  pad_x = (dst_w - new_w) / 2;
  pad_y = (dst_h - new_h) / 2;

  cv::Mat resized;
  cv::resize(img_bgr, resized, cv::Size(new_w, new_h));

  out = cv::Mat::zeros(dst_h, dst_w, CV_8UC3);
  resized.copyTo(out(cv::Rect(pad_x, pad_y, new_w, new_h)));

  return true;
}

static inline float sigmoid(float x)
{
  return 1.0f / (1.0f + std::exp(-x));
}

void RknnYoloV5::decode_outputs(const std::vector<cv::Mat>& outs,
                                float scale, int pad_x, int pad_y,
                                int img_w, int img_h,
                                float conf_thres,
                                std::vector<Detection>& dets)
{
  dets.clear();

  // 这里假定每个输出 tensor 已经被转成 float32 NHWC 或 NCHW 的 feature，
  // 并且是 [1, num_anchor, (5+num_classes)] 的展开。
  // 实际维度可能因你的导出方式不同而变化，你需要根据 rknn demo 调整。
  for(size_t oi = 0; oi < outs.size(); ++oi)
  {
    const cv::Mat& out = outs[oi]; // shape: [num, 5+num_classes]
    int num = out.rows;
    int dim = out.cols;
    if(dim < 5 + num_classes_) continue;

    const float* ptr = (float*)out.data;

    for(int i = 0; i < num; ++i)
    {
      const float* p = ptr + i * dim;
      float bx = p[0];
      float by = p[1];
      float bw = p[2];
      float bh = p[3];
      float obj_conf = sigmoid(p[4]);

      // 找到类别得分最高的一个
      int best_cls = -1;
      float best_cls_score = 0.0f;
      for(int c = 0; c < num_classes_; ++c)
      {
        float cls_score = sigmoid(p[5 + c]);
        if(cls_score > best_cls_score)
        {
          best_cls_score = cls_score;
          best_cls = c;
        }
      }

      float conf = obj_conf * best_cls_score;
      if(conf < conf_thres) continue;

      // 坐标还原：这里假定 bx,by,bw,bh 已经是输入尺度上的 box(x,y,w,h)，
      // 若不是，你需要参考自己的后处理代码进行修改。
      float x1 = bx - bw / 2.0f;
      float y1 = by - bh / 2.0f;
      float x2 = bx + bw / 2.0f;
      float y2 = by + bh / 2.0f;

      // 去掉 padding + 缩放回原图
      x1 = (x1 - pad_x) / scale;
      y1 = (y1 - pad_y) / scale;
      x2 = (x2 - pad_x) / scale;
      y2 = (y2 - pad_y) / scale;

      x1 = std::max(0.0f, std::min(x1, (float)img_w - 1));
      y1 = std::max(0.0f, std::min(y1, (float)img_h - 1));
      x2 = std::max(0.0f, std::min(x2, (float)img_w - 1));
      y2 = std::max(0.0f, std::min(y2, (float)img_h - 1));

      Detection det;
      det.box = cv::Rect(cv::Point((int)x1, (int)y1),
                         cv::Point((int)x2, (int)y2));
      det.score = conf;
      det.class_id = best_cls;

      if(det.box.width > 0 && det.box.height > 0)
        dets.push_back(det);
    }
  }
}

void RknnYoloV5::nms(std::vector<Detection>& dets, float nms_thres)
{
  std::sort(dets.begin(), dets.end(),
            [](const Detection& a, const Detection& b){
              return a.score > b.score;
            });

  std::vector<Detection> result;
  std::vector<bool> removed(dets.size(), false);

  for(size_t i = 0; i < dets.size(); ++i)
  {
    if(removed[i]) continue;
    result.push_back(dets[i]);

    for(size_t j = i + 1; j < dets.size(); ++j)
    {
      if(removed[j]) continue;
      float xx1 = std::max((float)dets[i].box.x, (float)dets[j].box.x);
      float yy1 = std::max((float)dets[i].box.y, (float)dets[j].box.y);
      float xx2 = std::min((float)(dets[i].box.x + dets[i].box.width),
                           (float)(dets[j].box.x + dets[j].box.width));
      float yy2 = std::min((float)(dets[i].box.y + dets[i].box.height),
                           (float)(dets[j].box.y + dets[j].box.height));
      float w = std::max(0.0f, xx2 - xx1);
      float h = std::max(0.0f, yy2 - yy1);
      float inter = w * h;
      float area1 = dets[i].box.area();
      float area2 = dets[j].box.area();
      float ovr = inter / (area1 + area2 - inter);
      if(ovr > nms_thres) removed[j] = true;
    }
  }

  dets.swap(result);
}

bool RknnYoloV5::infer(const cv::Mat& img_bgr, std::vector<Detection>& dets,
                       float conf_thres, float nms_thres)
{
  if(!initialized_) return false;
  if(img_bgr.empty()) return false;

  int img_w = img_bgr.cols;
  int img_h = img_bgr.rows;

  cv::Mat input_img;
  float scale;
  int pad_x, pad_y;
  if(!preprocess(img_bgr, input_img, scale, pad_x, pad_y))
    return false;

  // 准备输入
  rknn_input input;
  memset(&input, 0, sizeof(input));
  input.index = 0;
  input.type  = RKNN_TENSOR_UINT8;
  input.size  = input_width_ * input_height_ * input_channels_;
  input.fmt   = RKNN_TENSOR_NHWC;
  input.buf   = input_img.data;

  int ret = rknn_inputs_set(rknn_ctx_, 1, &input);
  if(ret < 0)
  {
    std::cerr << "rknn_inputs_set failed, ret=" << ret << std::endl;
    return false;
  }

  ret = rknn_run(rknn_ctx_, NULL);
  if(ret < 0)
  {
    std::cerr << "rknn_run failed, ret=" << ret << std::endl;
    return false;
  }

  // 获取输出
  std::vector<rknn_output> outputs(output_count_);
  for(int i = 0; i < output_count_; ++i)
  {
    memset(&outputs[i], 0, sizeof(outputs[i]));
    outputs[i].want_float = 1;  // 直接让 runtime 帮你转成 float
  }

  ret = rknn_outputs_get(rknn_ctx_, output_count_, outputs.data(), NULL);
  if(ret < 0)
  {
    std::cerr << "rknn_outputs_get failed, ret=" << ret << std::endl;
    return false;
  }

  std::vector<cv::Mat> out_mats;
  out_mats.reserve(output_count_);
  for(int i = 0; i < output_count_; ++i)
  {
    rknn_output& o = outputs[i];
    // 这里假定输出是一维展开的 [num * dim]
    int elem_num = o.size / sizeof(float);
    // 具体 num, dim 要根据你的模型决定，这里先假定 dim = 5+num_classes
    int dim = 5 + num_classes_;
    int num = elem_num / dim;
    cv::Mat out(num, dim, CV_32F, o.buf);
    out_mats.emplace_back(out.clone()); // clone 一份，后面可以 rknn_outputs_release
  }

  rknn_outputs_release(rknn_ctx_, output_count_, outputs.data());

  // 后处理
  std::vector<Detection> raw_dets;
  decode_outputs(out_mats, scale, pad_x, pad_y,
                 img_w, img_h, conf_thres, raw_dets);
  nms(raw_dets, nms_thres);
  dets.swap(raw_dets);
  return true;
}
