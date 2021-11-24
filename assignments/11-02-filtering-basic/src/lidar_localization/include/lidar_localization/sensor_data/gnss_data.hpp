/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include <GeographicLib/LocalCartesian.hpp>

namespace lidar_localization {
/**
 * GNSS数据类型
 */ 
class GNSSData {
  public:
    double time = 0.0;        // 时间
    double longitude = 0.0;   // 经度
    double latitude = 0.0;    // 纬度
    double altitude = 0.0;    // 高度
    double local_E = 0.0;     // 东
    double local_N = 0.0;     // 北
    double local_U = 0.0;     // 天
    int status = 0;           // 卫星定位状态信息 -1未解算位置；0未增强；1使用卫星增强；2使用地面增强
    int service = 0;          // 使用了哪些gps信号，1GPS；2GLONASS；4COMPASS；8GALILEO

    static double origin_longitude;     // 原点的经度
    static double origin_latitude;      // 原点的纬度
    static double origin_altitude;      // 原点的高度

  private:
    static GeographicLib::LocalCartesian geo_converter;   // 局部坐标
    static bool origin_position_inited;                   // 是否进行位置初始化

  public: 
    /**
     * @brief 重设原点
     */ 
    void InitOriginPosition();

    /**
     * @brief 将初始化后的经纬高坐标转化为东北天坐标
     */ 
    void UpdateXYZ();

    /**
     * @brief 将东北天坐标转化为经纬高坐标
     * @param local_E 东
     * @param local_N 北
     * @param local_U 天
     * @param lat 纬度
     * @param lon 经度
     * @param alt 高度
     */    
    static void Reverse(
      const double &local_E, const double &local_N, const double &local_U,
      double &lat, double &lon, double &alt
    );

    /**
     * @brief gnss数据的时间同步
     *        使用线性插值的方法来对经纬高坐标和东北天坐标进行求解
     * @param UnsyncedData  未同步的gnss数据
     * @param SyncedData  已经同步的gnss数据
     * @param sync_time   同步的时间
     * @return 是否成功完成时间同步
     */      
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}
#endif