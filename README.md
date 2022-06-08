# ndt_localization

1. 将原本的手动初始化位姿替换为GNSS初始化，可实现任意位置下的初始化。
     需要在gps_pose.cpp中设置gps第一帧经纬高：
    
      geoConverter.Reset(29.4943633258, 106.567516844, 234.04);
      
2. 添加了雷达检测模块
     
