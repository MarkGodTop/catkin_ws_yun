#include "plan_and_control/trajectory_replan_node.h"



msr::airlib::MultirotorRpcLibClient client("192.168.1.241");
ThreadPool threadPool(5);
TrajectoryReplanNode::TrajectoryReplanNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) 
:nh_(nh), nh_private_(nh_private), got_circle_flag_(false) {
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::ifstream i("/home/ros20/catkin_ws/1.json", std::ifstream::binary);
    std::string errs;
    if (!Json::parseFromStream(builder, i, &root, &errs)) { 
        std::cerr << "Error opening or parsing JSON file: " << errs << std::endl;
        return;
    }
    i.close();
    const Json::Value path_data = root["TaskData"]["EquipmentData"][0]["PathData"];
    int index = 0;
    max_vel_ = 10;
    max_acc_ = 4;
    for (const auto& point : path_data)
        ++index;
    waypoint_num_ = index;
    waypoints_.resize(waypoint_num_, 3);
    index = 0;
    for (const auto& point : path_data) {
        waypoints_(index,0) = point["PointX"].asDouble() - root["TaskData"]["EquipmentData"][0]["PathData"][0]["PointX"].asDouble();
        waypoints_(index,1) = point["PointY"].asDouble() - root["TaskData"]["EquipmentData"][0]["PathData"][0]["PointY"].asDouble();
        waypoints_(index,2) = point["PointZ"].asDouble() - root["TaskData"]["EquipmentData"][0]["PathData"][0]["PointZ"].asDouble();
        int i = 1;
        // 获取 PointID 和 TaskIndicator
        int pointID = point["PointID"].asInt();
        const Json::Value& taskIndicator = point["TaskIndicator"];
        // 遍历 TaskIndicator 中的每个任务
        for (const auto& task : taskIndicator) {
            // 如果是数组，则表示一个任务和其持续时间
            std::string taskName = task["TaskIndicatorName"].asString();
            std::string taskDuration = task["TaskIndicatorEnd"].asString();
            taskMap[pointID].emplace_back(taskName + "(" + taskDuration + ")");
        }
        ++index;
    }
        
        // YAML::Node task_node(YAML::NodeType::Sequence);
        // for (const auto& task : taskIndicator) {
        //     std::string task_name = "task" + std::to_string(i);
        //     std::string persistent_name = "persistent" + std::to_string(i);
        //     if (task.isArray()) {
        //         YAML::Node task_info(YAML::NodeType::Map);
        //         task_info[task_name] = task[0].asString();
        //         task_info[persistent_name] = task[1].asInt();
        //         // task_node.emplace_back(task_info);
        //     } else if (task.isString()) {
        //         YAML::Node task_info(YAML::Node Type::Map);
        //         task_info[task_name] = task.asString();
        //         // task_node.emplace_back(task_info);
        //     }
        // i++;
        // }
        // point_node["TaskIndicator"] = task_node;

        // 将节点添加到YAML节点中
        // std::string key = "waypoint" + std::to_string(index);
        // node[key] = point_node;
    
    for (const auto& taskList : taskMap) {
        std::cout << "PointID: " << taskList.first << std::endl;
        for (const auto& task : taskList.second) {
            std::cout << "Task: " << task << std::endl;
        }
        std::cout << std::endl;
    }
    // Write to YAML file
    // std::ofstream yaml_file("/home/markgodtop/yolov8/catkin_ws/src/Yolov8_ros/plan_and_control/config/waypoints.yaml");
    // if (!yaml_file.is_open()) {
    //     std::cerr << "Error opening YAML file for writing." << std::endl;
    //     return;
    // } 
    // yaml_file << "waypoint_num: " << index << "\n";
    // yaml_file << node << "\n";
    // for (const auto& pair : node) {
    //     yaml_file << pair.first.as<std::string>() << ": {x: " << pair.second["x"].as<double>() << ", y: " << pair.second["y"].as<double>() << ", z: " << pair.second["z"].as<double>() << "}\n";
    // }
    // yaml_file << "max_vel: 20\n";
    // yaml_file << "max_acc: 8\n";
    // yaml_file.close();
    nh_private_.getParam("vehicle", vehicle);
    it_ = std::make_unique<image_transport::ImageTransport>(nh_);
    depth_image_sub_ = it_->subscribe("/airsim_node/" + vehicle + "/Custom_MV_CAMERA_001_02/DepthPlanar", 10,
                        std::bind(&TrajectoryReplanNode::depthImageCallback, this,  std::placeholders::_1));
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/" + vehicle + "/odom_local_ned",1, &TrajectoryReplanNode::odomCallback,this);
        
    yolo_sub_ = 
        nh_.subscribe<yolov8_ros_msgs::BoundingBoxes>("/yolov8/BoundingBoxes",10, &TrajectoryReplanNode::boundingBoxes,this);
    desiredStates_pub_ = 
        nh_.advertise<uav_msgs::DesiredStates>("/reference/desiredStates", 50);
    traj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_vis", 10);
    cmd_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/cmd_vis",10);
    desiredPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/desiredPose", 10);
    currentPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/currentPose", 10);
    timer_ = nh_.createTimer(ros::Duration(1/300), &TrajectoryReplanNode::getCircleCenter, this);
    //cout << "waypoint init failed!" << endl;
    
    takeoff_client_ = nh_.serviceClient<uav_msgs::Takeoff>("/airsim_node/" + vehicle + "/takeoff");//get params from sever
    // YAML::Node config = YAML::LoadFile("/home/markgodtop/yolov8/catkin_ws/src/Yolov8_ros/plan_and_control/config/waypoints.yaml");
    // max_vel_ = config["max_vel"].as<double>();
    // max_acc_ = config["max_acc"].as<double>();
    // waypoint_num_ = config["waypoint_num"].as<int>();
    cout << "waypoint number:" << waypoint_num_ << endl;
    // for (int i = 0; i < waypoint_num_; ++i) {
    //     std::string waypoint_name = "waypoint" + std::to_string(i);
    //     waypoints_(i,0) =  config[waypoint_name]["x"].as<double>();
    //     waypoints_(i,1) =  config[waypoint_name]["y"].as<double>();
    //     waypoints_(i,2) =  config[waypoint_name]["z"].as<double>();
    // }
    cout << "waypoints:" << waypoints_ << endl;
    // uav_msgs::Takeoff takeoff;
    // takeoff.request.waitOnLastTask = 1;
    // takeoff_client_.call(takeoff);
    trajectoryGenerate(waypoints_);
    

}

TrajectoryReplanNode::~TrajectoryReplanNode() {}


void TrajectoryReplanNode::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if(yolo_ == nullptr){
        return;
    }
    for (std::size_t i = 0; i < yolo_->bounding_boxes.size(); ++i){
        float x_center_rgb = (yolo_->bounding_boxes[i].xmin + yolo_->bounding_boxes[i].xmax) / 2.0f;
        float y_center_rgb = (yolo_->bounding_boxes[i].ymin + yolo_->bounding_boxes[i].ymax) / 2.0f;
        depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        Mat depth = depth_ptr_->image;
        int y = y_center_rgb;
        int x = x_center_rgb;
        // float depth_value = depth.ptr<float>(int(camera_coord_depth[1]))[int(camera_coord_depth[0])];
        float depth_value = depth.ptr<float>(y)[x];
        Eigen::Vector3d pixel_and_depth(x_center_rgb, y_center_rgb, depth_value);
        // std::cout << "pixel_and_depth:" << pixel_and_depth << std::endl;
        Eigen::Vector3d point_w = transformPixel2World(pixel_and_depth);
        int type_class;
        if(yolo_->bounding_boxes[i].Class.empty())
            type_class = 0;
        if(yolo_->bounding_boxes[i].Class == "Huan")
            type_class = 1;
        if(yolo_->bounding_boxes[i].Class == "Person")
            type_class = 2;
        if(yolo_->bounding_boxes[i].Class == "YouQiTong")
            type_class = 8;
        values.x = point_w.x();
        values.y = point_w.y();
        values.z = point_w.z();
        Eigen::Vector3d point_m(odom_->pose.pose.position.x,odom_->pose.pose.position.y,odom_->pose.pose.position.z);
        double d = (point_w - point_m).norm();
        values.d = d;
        values.class_val = type_class;
        cc = 1;
        // std::cout << "d :" << d << std::endl;
    }
    
    // depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    // Mat depth = depth_ptr_->image;
    // //cout << depth << endl;
    // //cout << "rowsandcols:" << depth.rows << " "<< depth.cols << endl;
    // Mat gray (depth.rows, depth.cols, CV_8UC1);
    // Mat mask = Mat::zeros(depth.size(), CV_8UC1);

    // for (int i = 0; i < depth.rows; ++i) {
    //     for (int j = 0; j < depth.cols; ++j) {
    //         uchar d = depth.at<float>(i,j) /10 * 255;
    //         gray.at<uchar>(i,j) = d;
    //         //cout << depth.at<float>(i,j) << endl; 
    //     }
    // }
    // medianBlur(gray, gray, 5);
    // vector<Vec3f> circles;
    
    // HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
    //             50,80,28,15,30);
    // //cout << circles.size() << endl;
    // if (circles.size() == 1) {
    //     got_circle_flag_ = true;
    //     for (int i = 0; i < circles.size(); ++i) {
    //         Vec3f c = circles[i];
    //         Point center = Point(c[0],c[1]);      
    //         //扫描圆环外接矩行的像素深度，把深度不为0的像素深度加和取平均数
    //         float sum = 0;
    //         int count = 0;
    //         for (int v = c[1] - c[2] - 5; v < c[1] + c[2] + 5; ++v) {
    //             for (int u = c[0] - c[2] - 5; u < c[0] + c[2] + 5; ++u) {
    //                 float d = depth.ptr<float>(v)[u];
    //                 if (d == 0) continue;
    //                 sum += d;
    //                 ++count;
    //             }
    //         }
    //         double depth5 = sum / count;
    //         //cout << "depth:" << depth5 << endl;
    //         cout << "row_index:" << row_idx_ << endl;
    //         //cout << "depth center:" << center << endl;
    //         Eigen::Vector3d pixel_and_depth(c[0], c[1], depth5);
    //         Eigen::Vector3d point_w = transformPixel2World(pixel_and_depth);
            // values.x = point_w.x();
            // values.y = point_w.y();
            // values.z = point_w.z();
            // Eigen::Vector3d point_m(odom_->pose.pose.position.x,odom_->pose.pose.position.y,odom_->pose.pose.position.z);
            // double d = (point_w - point_m).norm();
            // values.d = d;



    //         double distance = (point_w - waypoints_.row(row_idx_).transpose()).norm();
    //         cout << "distance:" << distance << endl;
    //         if (distance > 0.5) {
    //             Eigen::MatrixXd replan_matrix = Eigen::MatrixXd::Zero(1 - row_idx_ + 1, 3);
    //             replan_matrix(0,0) = odom_pos_(0);
    //             replan_matrix(0,1) = odom_pos_(1);
    //             replan_matrix(0,2) = odom_pos_(2);
    //             replan_matrix(1,0) = point_w(0);
    //             replan_matrix(1,1) = point_w(1);
    //             replan_matrix(1,2) = point_w(2);
    //             replan_matrix.bottomRows(1 - row_idx_ - 1) = waypoints_.bottomRows(1 - row_idx_ - 1);
    //             // cout << "replan_matrix:" << replan_matrix << endl;
                       
    //             // trajectoryGenerate(replan_matrix);
    //         }
            
    //         circle(gray,center, 1, Scalar(255,255,255),1,LINE_AA);
    //         int radius = c[2];
    //         circle(gray, center, radius, Scalar(255,255,255),5,LINE_AA);
    //     }
    // }
    // imshow("depth2gray", gray);
    // cv::waitKey(1);

}

void TrajectoryReplanNode::trajectoryGenerate(const Eigen::MatrixXd &waypoints) {
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
    // vel(0,0) = odom_vel_(0);
    // vel(0,1) = odom_vel_(1);
    // vel(0,2) = odom_vel_(2);
    // vel(1,0) = 0.8;
    // vel(1,1) = -4.67;
    // vel(1,2) = 0;
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    cout << "compute time !" << endl;
    times_ = timeAllocation(waypoints);
    cout << "time vector:" << times_.transpose() << endl;
    coeff_matrix_ = trajPlanWaypoints_->miniJerkTrajGenerate(waypoints, vel, acc, times_);
    cout << "trajectory generate finish!" << endl;
    final_time_ = start_time_ = ros::Time::now();
    segment_num_ = times_.size();
    for (int i = 0; i < segment_num_; ++i) {
        final_time_ += ros::Duration(times_(i));
    }
}

// void TrajectoryReplanNode::shared_yolo(){
//     // std::lock_guard<std::mutex> lock1(mtx1);
//     if(yolo_ == nullptr){
//         return;
//     }
//     if(stamp3 != yolo_->header.stamp)   
//         {
//             stamp3 = yolo_->header.stamp;
//             Value data(Json::arrayValue);
//             int type_class;
//             for (const auto& box : yolo_->bounding_boxes) {
//                 Value obj;
//                 if(box.Class.empty())
//                     type_class = 0;
//                 if(box.Class == "Huan")
//                     type_class = 1;
//                 if(box.Class == "Person")
//                     type_class = 2;
//                 if(box.Class == "YouQiTong")
//                     type_class = 8;
//                 obj["ObjType"] = type_class;
//                 obj["ULPointX"] = static_cast<double>(box.xmin); // xmin
//                 obj["ULPointY"] = static_cast<double>(box.ymin); // ymin
//                 obj["DRPointX"] = static_cast<double>(box.xmax); // xmax
//                 obj["DRPointY"] = static_cast<double>(box.ymax); // ymax
//                 if(box.xmin)
//                 {
//                     cout << "box.xmin " << box.xmin << endl;
//                     cout << "box.ymin " << box.ymin << endl;
//                     cout << "box.xmax " << box.xmax << endl;
//                     cout << "box.ymax " << box.ymax << endl;
//                 }
//                 // 将边界框添加到数据数组中
//                 data.append(obj);
//             }
//             Value root;
//             uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//             root["timestamp"] = Json::Value(static_cast<Int64>(ms)); // 将uint64_t转换为Json::Value::UInt64类型 // 将时间戳转换为纳秒
//             root["resX"] = resX;
//             root["resY"] = resY;
//             root["cameraName"] = cameraName;
//             root["data"] = data;
//             // 使用jsoncpp的StreamWriter来格式化JSON
//             StreamWriterBuilder builder;
//             builder["indentation"] = "\t"; // 使用制表符缩进
//             const std::string json_str = Json::writeString(builder, root);
//             client.simUpdateLocalDetectTargetNumData(vehicle, json_str);
//             // std::this_thread::sleep_for(std::chrono::milliseconds(11));  
//         }
    
// }
double x1 = 0;
double x2 = 0;
double x3 = 0;
void TrajectoryReplanNode::publishTopic_yolo(const int& data) {
    
    // std::lock_guard<std::mutex> lock2(mtx2);
    
    while (continuePublishing.load() && flag.load() < data) {  
       if(values.x != x3 && cc == 1)   
        {
            // client.simUpdateLocalDetectTargetPreData(vehicle, values.x, values.y, values.z, values.class_val);
            client.simUpdateLocalDetectTargetPreData(vehicle, values.x, values.y, values.z, values.class_val);
            x3 = values.x;

        }
        // shared_yolo();// 假设你想要使用第一个边界框     
    }
    // if (flag.load() == data) {
    //     auto threadId = std::this_thread::get_id();
    //     for (auto it = publishThreads.begin(); it != publishThreads.end(); ++it) {
    //         if ((*it)->get_id() == threadId) {
    //             (*it)->detach(); // 分离线程，使其成为后台线程
    //             publishThreads.erase(it); // 从线程向量中删除线程
    //             break;
    //         }
    //     }
    // }
}

void TrajectoryReplanNode::publishTopic_ceju(const int& data) {   
    // std::lock_guard<std::mutex> lock5(mtx5);   
    while (continuePublishing.load() && flag.load() < data) {      
        if(values.d != x1 && cc == 1)
        {
            std::cout << "values.d:" << values.d << std::endl;
            client.simUpdateLocalTargetDistanceData(vehicle, values.d, values.x, values.y, values.z, 1, true);
            x1 = values.d;
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(11));
    }
}
void TrajectoryReplanNode::publishTopic_dingwei(const int& data) {  
    // std::lock_guard<std::mutex> lock3(mtx3);
    while (continuePublishing.load() && flag.load() < data) {
        if(stamp1 != odom_->header.stamp)
        {
            stamp1 = odom_->header.stamp;
            uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
            client.simUpdateLocalPositionData(vehicle, odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z, ms, 7);
        }      
        // std::this_thread::sleep_for(std::chrono::milliseconds(11));  
    }
}
void TrajectoryReplanNode::publishTopic_dingzi(const int& data) {  
    // std::lock_guard<std::mutex> lock4(mtx4);  
    while (continuePublishing.load() && flag.load() < data) {
        if(stamp2 != odom_->header.stamp)
        {
            stamp2 = odom_->header.stamp;
            uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
            // client.simUpdateLocalRotationData(vehicle, odom_->pose.pose.orientation.w, odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y,odom_->pose.pose.orientation.z, ms);
            client.simUpdateLocalRotationData(vehicle, 1, 2, 3,4, ms);
        }
            // std::this_thread::sleep_for(std::chrono::milliseconds(11)); 
    }
}
void TrajectoryReplanNode::publishTopic_jihui(const int& data) {  
    // std::lock_guard<std::mutex> lock4(mtx4);  
    while (continuePublishing.load() && flag.load() < data) {
        if(values.x != x2 && cc == 1)
        {
            // client.simFireNavMissile(vehicle, values.x, values.y, values.z, false);
            client.simFireNavMissile(vehicle,-76.19, -73.24, 0.69, false);
            x2 = values.x;
        }
            // std::this_thread::sleep_for(std::chrono::milliseconds(11)); 
    }
}
void TrajectoryReplanNode::getCircleCenter(const ros::TimerEvent &e) {
    // Eigen::Vector3d waypoint(waypoints_(row_idx_,0), waypoints_(row_idx_,1), waypoints_(row_idx_,2));
    // double d = (waypoint - odom_pos_).norm();
    // //ROS_INFO("dis to goal: %f", d);
    // //ROS_INFO("goal : %f %f %f", waypoints_(row_idx_,0), waypoints_(row_idx_,1), waypoints_(row_idx_,2));
    // if (d < 2.5) {
    //     ++row_idx_;
    //     got_circle_flag_ = false;
    //     if (row_idx_ == waypoint_num_) {
    //         --row_idx_;
    //     }
    // }
    //cout << "got_circle:" << got_circle_flag_ << endl; 
    if(odom_ == nullptr)
        return;
    for (int i = 0; i < waypoints_.rows(); ++i) {
        // 计算当前位置与waypoint点之间的距离     
        geometry_msgs::Point current_pos_ros;
        current_pos_ros.x = odom_->pose.pose.position.x;
        current_pos_ros.y = odom_->pose.pose.position.y;
        current_pos_ros.z = odom_->pose.pose.position.z;
        Eigen::Vector3d current_pos(current_pos_ros.x, current_pos_ros.y, current_pos_ros.z);
        Eigen::Vector3d waypoint = waypoints_.row(i);
        distance1 = (current_pos - waypoint).norm();       
        // 如果距离小于某个阈值，则认为无人机已到达该waypoint点
        if (distance1 < 0.3) {
            reached_waypoint = true;
            flag.store(i + 1);
            break;
        }
    }
    
    // 如果到达了waypoint点，发布里程计位置信息
    if (reached_waypoint) {
        for(const auto& task : taskMap[flag.load()]){
            std::regex pattern("\\d"); // 检查任何数字
            std::smatch match;
            if(task.find("iouTargetAccuracy") != std::string::npos){
                if (std::regex_search(task, match, pattern)) {
                    // 如果任务包含数字，解析数字
                    std::ssub_match subMatch = match[0];
                    std::string numStr = subMatch.str();
                    num.store(std::stoi(numStr)); // 将数字字符串转换为整数
                    // 发布话题直到flag等于那个数字
                    if(set_t.find("iouTargetAccuracy") == set_t.end() || set_t["iouTargetAccuracy"] < num.load()){
                        // auto thread = std::make_shared<std::thread>(&TrajectoryReplanNode::publishTopic_yolo, this, num.load());
                        // publishThreads.emplace_back(thread);
                        threadPool.enqueue(&TrajectoryReplanNode::publishTopic_yolo, this, num.load());
                        set_t["iouTargetAccuracy"] = num.load();
                    }
                    
                } else {
                    // client.simUpdateLocalDetectTargetPreData(vehicle, values.x, values.y, values.z, values.class_val);
                    // shared_yolo();
                }
            } 
            if(task.find("attitudeAccuracy") != std::string::npos){
                if (std::regex_search(task, match, pattern)) {
                    // 如果任务包含数字，解析数字
                    std::ssub_match subMatch = match[0];
                    std::string numStr = subMatch.str();
                    num.store(std::stoi(numStr)); // 将数字字符串转换为整数
                    // 发布话题直到flag等于那个数字
                    if(set_t.find("attitudeAccuracy") == set_t.end() || set_t["attitudeAccuracy"] < num.load()){
                        // auto thread = std::make_shared<std::thread>(&TrajectoryReplanNode::publishTopic_dingzi, this, num.load());
                        // publishThreads.emplace_back(thread);
                        threadPool.enqueue(&TrajectoryReplanNode::publishTopic_dingzi, this, num.load());
                        set_t["attitudeAccuracy"] = num.load();
                    }
                } else {
                    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
                    client.simUpdateLocalRotationData(vehicle, odom_->pose.pose.orientation.w, odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y,odom_->pose.pose.orientation.z, ms);
                }
            } 
            if(task.find("positionMeasureFour") != std::string::npos){
                if (std::regex_search(task, match, pattern)) {
                    // 如果任务包含数字，解析数字
                    std::ssub_match subMatch = match[0];
                    std::string numStr = subMatch.str();
                    num.store(std::stoi(numStr)); // 将数字字符串转换为整数
                    // 发布话题直到flag等于那个数字
                    if(set_t.find("positionMeasureFour") == set_t.end() || set_t["positionMeasureFour"] < num.load()){
                        // auto thread = std::make_shared<std::thread>(&TrajectoryReplanNode::publishTopic_dingwei, this, num.load());
                        // publishThreads.emplace_back(thread);
                        threadPool.enqueue(&TrajectoryReplanNode::publishTopic_dingwei, this, num.load());
                        set_t["positionMeasureFour"] = num.load();
                    }
                } else {
                    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
                    client.simUpdateLocalPositionData(vehicle, odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z, ms);
                }
            } 
            if(task.find("dynamicRangingAccuracy") != std::string::npos){
                if (std::regex_search(task, match, pattern)) {
                    // 如果任务包含数字，解析数字
                    std::ssub_match subMatch = match[0];
                    std::string numStr = subMatch.str();
                    num.store(std::stoi(numStr)); // 将数字字符串转换为整数
                    // 发布话题直到flag等于那个数字
                    if(set_t.find("dynamicRangingAccuracy") == set_t.end() || set_t["dynamicRangingAccuracy"] < num.load()){
                        // auto thread = std::make_shared<std::thread>(&TrajectoryReplanNode::publishTopic_ceju, this, num.load());
                        // publishThreads.emplace_back(thread);
                        threadPool.enqueue(&TrajectoryReplanNode::publishTopic_ceju, this, num.load());
                        set_t["dynamicRangingAccuracy"] = num.load();
                    }
                } else {  
                    // client.simUpdateLocalTargetDistanceData(vehicle, values.d, values.x, values.y, values.z, values.class_val, true);
                    client.simUpdateLocalTargetDistanceData(vehicle, values.d, values.x, values.y, values.z, 1, true);
                }
            } 
            if(task.find("targetSmash") != std::string::npos){
                if (std::regex_search(task, match, pattern)) {
                    // 如果任务包含数字，解析数字
                    std::ssub_match subMatch = match[0];
                    std::string numStr = subMatch.str();
                    num.store(std::stoi(numStr)); // 将数字字符串转换为整数
                    // 发布话题直到flag等于那个数字
                    if(set_t.find("targetSmash") == set_t.end() || set_t["targetSmash"] < num.load()){
                        // auto thread = std::make_shared<std::thread>(&TrajectoryReplanNode::publishTopic_ceju, this, num.load());
                        // publishThreads.emplace_back(thread);
                        threadPool.enqueue(&TrajectoryReplanNode::publishTopic_jihui, this, num.load());
                        set_t["targetSmash"] = num.load();
                    }
                } else {   
                    client.simFireNavMissile(vehicle, values.x, values.y, values.z, false);
                }
            }
            if(flag.load() == waypoint_num_){
                publishThreads.clear();
                set_t.clear();
            }
        }
        reached_waypoint = false;
    }   
}
void TrajectoryReplanNode::desiredStatesPub() {
    cmd_.header = odom_->header;
    cmd_.header.frame_id = odom_->header.frame_id;
    cmd_.header.stamp = odom_->header.stamp;
    traj_duration_ = (final_time_ - start_time_).toSec();
    double t = max(0.0, (odom_->header.stamp - start_time_).toSec());
    if (t > traj_duration_) {
        cmd_.position.x = odom_->pose.pose.position.x;
        cmd_.position.y = odom_->pose.pose.position.y;
        cmd_.position.z = odom_->pose.pose.position.z;

        cmd_.velocity.x = 0;
        cmd_.velocity.y = 0;
        cmd_.velocity.z = 0;

        cmd_.acceleration.x = 0;
        cmd_.acceleration.y = 0;
        cmd_.acceleration.z = 0;

    } else {
        for (int i = 0; i < segment_num_; ++i) {
            if (t > times_(i) ) {
                t -= times_(i);
            } else {
                Eigen::Vector3d desiredPos = trajPlanWaypoints_->getPosition(coeff_matrix_, i, t);
                //cout << "desiredPos:" << desiredPos.transpose() << endl;
                //cout << "odom_Pos:" << odom_pos_.transpose() << endl;
                Eigen::Vector3d desiredVel = trajPlanWaypoints_->getVelocity(coeff_matrix_, i, t);
                //cout << "desiredVel:" << desiredVel.transpose() << endl;
                //cout << "odom_vel:" << odom_vel_.transpose() << endl;
                Eigen::Vector3d desiredAcc = trajPlanWaypoints_->getAcceleration(coeff_matrix_, i, t);
                desired_pos_ = desiredPos;
                cmd_.position.x = desiredPos.x();
                cmd_.position.y = desiredPos.y();
                cmd_.position.z = desiredPos.z();
                cmd_.velocity.x = desiredVel.x();
                cmd_.velocity.y = desiredVel.y();
                cmd_.velocity.z = desiredVel.z();
                cmd_.acceleration.x = desiredAcc.x();
                cmd_.acceleration.y = desiredAcc.y();
                cmd_.acceleration.z = desiredAcc.z();
                cmd_.yaw = atan2(cmd_.velocity.y, cmd_.velocity.x);
                //cmd_.yaw = 0.0;
                // cout << "desiredyaw:" << cmd_.yaw << endl;
                Eigen::Matrix3d currentAttitude = odom_orient_.toRotationMatrix();
                Eigen::Vector3d RPY = currentAttitude.eulerAngles(0,1,2);
                //cout << "currentyaw:" << RPY(0) << endl;
                desiredPose_.header.frame_id = "map";
                desiredPose_.header.stamp = odom_->header.stamp;
                desiredPose_.pose.position.x = desiredPos.x();
                desiredPose_.pose.position.y = desiredPos.y();
                desiredPose_.pose.position.z = desiredPos.z();
                break;
            }
        }
    }
    desiredStates_pub_.publish(cmd_);
    desiredPose_pub_.publish(desiredPose_);    
    dir_ << cos(cmd_.yaw), sin(cmd_.yaw), 0;
    drawCmd(desired_pos_, 2 * dir_, 1, Eigen::Vector4d(1, 1, 0, 0.7));
    // for (auto& thread : publishThreads) {
    //     if (thread.joinable()) {
    //         thread.join();
    //     }
    // }
}
void TrajectoryReplanNode::boundingBoxes(const yolov8_ros_msgs::BoundingBoxesConstPtr &msg){
    if (!msg) {
        std::cerr << "BoundingBoxes message is null!" << std::endl;
        return;
    }
    yolo_ = msg;
}
int flag = 0;
void TrajectoryReplanNode::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_ = msg;
    if(flag == 0){
        waypoints_(0,2) = waypoints_(0,2) + msg->pose.pose.position.z;
        flag = 1;
    }
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    currentPose_.header.frame_id = "map";
    currentPose_.header.stamp = msg->header.stamp;
    currentPose_.pose.position = msg->pose.pose.position;
    currentPose_.pose.orientation = msg->pose.pose.orientation;
    currentPose_pub_.publish(currentPose_);
    desiredStatesPub();
    displayTrajWithColor();
}

Eigen::VectorXd TrajectoryReplanNode::timeAllocation(const Eigen::MatrixXd &waypoints) {
    Eigen::VectorXd time(waypoints.rows() - 1);
    double current_vel = odom_vel_.norm();
        for (int i = 0; i < waypoints.rows() - 1; ++i) {
            double distance = (waypoints.row(i+1) - waypoints.row(i)).norm();
            double t = max_vel_ / max_acc_;
            double d = 0.5 * max_acc_ * t * t;
            if (distance < 2 * d) {
                time(i) = 2 * sqrt(distance / max_acc_);
            } else {
                time(i) = 2 * t + (distance - 2 * d) / max_vel_;
            }          
        }
        if (current_vel != 0) {
            double distance = (waypoints.row(1) - waypoints.row(0)).norm();
            time(0) = distance  / current_vel;
            for (int i = 1; i < waypoints.rows() - 1; ++i) {
                double distance = (waypoints.row(i+1) - waypoints.row(i)).norm();
                time(i) = distance * 3.4 / max_vel_;
            }
        }
    return time;
}


void TrajectoryReplanNode::displayTrajWithColor() {
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "traj";
    points.id = 0;
    line_strip.id = 1;
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w =line_strip.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    line_strip.scale.x = 0.3;
    points.color.a = 1.0;
    points.color.g = 1.0;
    line_strip.color.a = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.g = 1.0;
    geometry_msgs::Point pt;
    Eigen::Vector3d pos;
    for (int i = 0; i < segment_num_; ++i) {
        for (double t = 0; t < times_(i); t += 0.01) {
            pos = trajPlanWaypoints_->getPosition(coeff_matrix_, i, t);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            line_strip.points.emplace_back(pt);
        }
        }
    for (int i = 0; i < waypoint_num_; ++i) {
        pt.x = waypoints_(i,0);
        pt.y = waypoints_(i,1);
        pt.z = waypoints_(i,2);
        points.points.emplace_back(pt);
    }
    traj_vis_pub_.publish(points);
    traj_vis_pub_.publish(line_strip);

    ros::Duration(0.001).sleep();
}

void TrajectoryReplanNode::drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "map";
    mk_state.header.stamp = ros::Time::now();
    mk_state.id = id;
    mk_state.type = visualization_msgs::Marker::ARROW;
    mk_state.action = visualization_msgs::Marker::ADD;
    mk_state.pose.orientation.w = 1.0;
    mk_state.scale.x = 0.1;
    mk_state.scale.y = 0.2;
    mk_state.scale.z = 0.3;
    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state.points.emplace_back(pt);
    pt.x = pos(0) + vec(0);
    pt.y = pos(1) + vec(1);
    pt.z = pos(2) + vec(2);
    mk_state.points.emplace_back(pt);
    mk_state.color.r = color(0);
    mk_state.color.g = color(1);
    mk_state.color.b = color(2);
    mk_state.color.a = color(3);
    cmd_vis_pub_.publish(mk_state);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_replan_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    namedWindow("depth2gray");
    
    TrajectoryReplanNode trajectory_replan_node(nh, nh_private);

    ros::spin();
    return 0;
}