# 瓶子检测结果消息定义

# 标准消息头
std_msgs/Header header

# 检测到的瓶子数量
int32 num_detections

# 包围盒坐标 (x1,y1,x2,y2) - 左上角和右下角
float32[] bbox_x1
float32[] bbox_y1
float32[] bbox_x2
float32[] bbox_y2

# 置信度得分
float32[] scores

# 瓶子中心点坐标
float32[] center_x
float32[] center_y

# 瓶子距离信息（如果有的话）
float32[] distances

# 是否有距离信息
bool[] has_distance