/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()),
     global_map_ptr_(new CloudData::CLOUD()),
     result_cloud_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
    // 读取配置文件
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitDataPath(config_node);                                      // 初始化数据文件夹
    InitRegistration(registration_ptr_, config_node);               // 初始化位姿求解方法
    InitFilter("local_map", local_map_filter_ptr_, config_node);    // 选择滑窗地图点云滤波方法
    InitFilter("frame", frame_filter_ptr_, config_node);            // 选择当前帧点云滤波方法
    InitFilter("display", display_filter_ptr_, config_node);        // rviz 实时显示点云时滤波方法

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    data_path_ = config_node["data_path"].as<std::string>();
    if (data_path_ == "./") {
        data_path_ = WORK_SPACE_PATH;
    }
    data_path_ += "/slam_data";

    // 如果文件夹存在，移出所有文件
    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }

    // 创建文件夹，并输出路径
    boost::filesystem::create_directory(data_path_);
    if (!boost::filesystem::is_directory(data_path_)) {
        LOG(WARNING) << "Cannot create directory " << data_path_ << "!";
        return false;
    } else {
        LOG(INFO) << "Point Cloud Map Output Path: " << data_path_;
    }

    // 创建关键帧文件夹
    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(data_path_ + "/key_frames");
    if (!boost::filesystem::is_directory(key_frame_path)) {
        LOG(WARNING) << "Cannot create directory " << key_frame_path << "!";
        return false;
    } else {
        LOG(INFO) << "Key Frames Output Path: " << key_frame_path << std::endl << std::endl;
    }

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "Point Cloud Registration Method: " << registration_method;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } 
    else if (registration_method == "ICP") {
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    } 
    else if (registration_method == "ICP_SVD") {
        registration_ptr = std::make_shared<ICPSVDRegistration>(config_node[registration_method]);
    }
    else if (registration_method == "SICP") {
        registration_ptr = std::make_shared<SICPRegistration>(config_node[registration_method]);
    } 
    else if (registration_method == "YOUR_OWN_METHOD") {
        /*
            TODO: register your custom implementation here
        */
    }
    else {
        LOG(ERROR) << "Point cloud registration method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "Point Cloud Filter Method: " << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        // CloudFilterInterface通过make_shared到VoxelFilter获得虚函数的实例化
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "Point cloud filter method " << filter_mothod << " NOT FOUND!";
        return false;
    }

    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;   // 更新当前帧的时间戳
    std::vector<int> indices;   // 有效点云的索引
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices); // 剔除无效点云

    // 进行滤波
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr); // 使用体素网络滤波器对其进行滤波

    // 姿态相关参数
    // 由于其是static，所以在最开始进行数值初始化，初始化后不会改变
    // 只在第一次运行的时候进行
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();     // 两帧之间的位姿变换
    static Eigen::Matrix4f last_pose = init_pose_;                      // 上一帧的位姿,由Gnss信息提供
    static Eigen::Matrix4f predict_pose = init_pose_;                   // 位姿的先验值，由GNSS信息提供，用于预测
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;            // 上一次关键帧的位姿

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        // 更新当前雷达里程计的位姿
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    // 输入滤波后的点云、GNSS的预测值、返回点云匹配结果和当前帧的位姿
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose;   // 雷达位姿，会更新front_end_flow的laser_odometry_

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    // 把关键帧点云存储到硬盘里，节省内存
    std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    Frame key_frame = new_key_frame;    // 新建关键帧
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));    // 实例化
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    // 局部地图的维护，保持局部地图为20帧
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        // 使用局部地图帧的位姿对局部地图的点云进行位姿变换，在记录到局部地图中
        // 实际上就是将传感器数据往全局底数数据转换
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;  // 有局部地图了便生成新的局部地图

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    global_map_frames_.push_back(key_frame);

    return true;
}

bool FrontEnd::SaveMap() {
    // 清空全局地图的点云指针
    global_map_ptr_.reset(new CloudData::CLOUD());

    // 初始化关键帧和相应变换的点云容器
    std::string key_frame_path = "";
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    // 读取关键帧数据并构建地图点云
    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
        // 读取关键帧数据
        key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);

        pcl::transformPointCloud(*key_frame_cloud_ptr, 
                                *transformed_cloud_ptr, 
                                global_map_frames_.at(i).pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
    }
    
    // 存储地图点云
    std::string map_file_path = data_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
    has_new_global_map_ = true;

    return true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {
        has_new_global_map_ = false;
        display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
    return true;
}
}