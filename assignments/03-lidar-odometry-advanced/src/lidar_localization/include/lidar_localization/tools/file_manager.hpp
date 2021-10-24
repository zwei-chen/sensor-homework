/*
 * @Description: 读写文件管理
 * @Author: Ren Qian
 * @Date: 2020-02-24 19:22:53
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace lidar_localization {
/**
 * 读写文件管理模块
 */ 
class FileManager{
  public:
    /**
     * @brief 创建文件
     * @param ofs 文件
     * @param file_path 文件路径
     * @return 创建文件成功则返回true，否则false
     */ 
    static bool CreateFile(std::ofstream& ofs, std::string file_path);

    /**
     * @brief 文件夹如果没有创建便创建文件夹，如果还是 无法创建则返回false
     * @param directory_path 文件夹路径
     * @return 文件夹创建成功则返回true，否则false
     */    
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
