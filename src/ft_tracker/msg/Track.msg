std_msgs/Header header

# 轨迹 ID，跨帧保持不变（同一段锁定期间）
uint32 id

# true: 当前帧有观测命中；false: 当前帧是纯预测
bool tracking

# 相机坐标系下的滤波后装甲板中心，单位米
geometry_msgs/Point position

# 相机坐标系下的滤波后速度，单位 m/s
geometry_msgs/Vector3 velocity
