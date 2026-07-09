# human_follow_rknn/msg/TargetInfo.msg
std_msgs/Header header

bool   target_acquired        # 是否锁定到目标
float32 u_norm                # (u - cx)/cx，左负右正
float32 v_norm                # (v - cy)/cy， 上负下正
float32 box_height_norm       # bbox 高 / 图像高，表示远近
float32 confidence            # 0~1
