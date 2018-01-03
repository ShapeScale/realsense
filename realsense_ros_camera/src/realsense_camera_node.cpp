// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <constants.h>
#include <realsense_ros_camera/Extrinsics.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <realsense_ros_camera/Start.h>
#include <realsense_ros_camera/Stop.h>
#include <realsense_ros_camera/IMUInfo.h>
#include <csignal>
#include <eigen3/Eigen/Geometry>

#include <RTIMULib.h>
#include <sensor_msgs/Imu.h>

#include <boost/asio.hpp>

#include <fstream>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>
#include <memory>
#include <atomic>

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;
static const double G_TO_MPSS = 9.80665;

namespace realsense_ros_camera
{
    using namespace std::chrono_literals;
    using namespace std::string_literals;
    using stream_index_pair = std::pair<rs2_stream, int>;

    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};

    const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
                                                                        {COLOR},
                                                                       }};


    inline void signalHandler(int signum)
    {
        ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminate RealSense Node...");
        ros::shutdown();
        exit(signum);
    }

    class RealSenseCameraNodelet: public nodelet::Nodelet
    {
    public:
        RealSenseCameraNodelet() :
            _serial_no(""),
            _base_frame_id(""),
            _intialize_time_base(false),
            running_(false),
            record_to_file_(true)
        {
            ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
            ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

            signal(SIGINT, signalHandler);
            auto severity = rs2_log_severity::RS2_LOG_SEVERITY_ERROR;
            tryGetLogSeverity(severity);
            if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
                ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

            rs2::log_to_console(severity);

            // Types for depth stream
            _format[DEPTH] = RS2_FORMAT_Z16;   // libRS type
            _image_format[DEPTH] = CV_16UC1;    // CVBridge type
            _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
            _unit_step_size[DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
            _stream_name[DEPTH] = "depth";

            // Infrared stream - Left
            _format[INFRA1] = RS2_FORMAT_Y8;   // libRS type
            _image_format[INFRA1] = CV_8UC1;    // CVBridge type
            _encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[INFRA1] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[INFRA1] = "infra1";

            // Infrared stream - Right
            _format[INFRA2] = RS2_FORMAT_Y8;   // libRS type
            _image_format[INFRA2] = CV_8UC1;    // CVBridge type
            _encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[INFRA2] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[INFRA2] = "infra2";

            // Types for color stream
            _format[COLOR] = RS2_FORMAT_RGB8;   // libRS type
            _image_format[COLOR] = CV_8UC3;    // CVBridge type
            _encoding[COLOR] = sensor_msgs::image_encodings::RGB8; // ROS message type
            _unit_step_size[COLOR] = 3; // sensor_msgs::ImagePtr row step size
            _stream_name[COLOR] = "color";

        }

        virtual ~RealSenseCameraNodelet()
        {}

    private:
        virtual void onInit()
        {
            setupSubscribers();
            //publishStaticTransforms();
            getParameters();
            setupDevice();
            setupPublishers();
            setupStreams();
            ROS_INFO_STREAM("RealSense Node Is Up!");
            ros::spin();
            ROS_INFO_STREAM("RealSense onInit about to exit!");
           
        }

        void IMUInit()
        {
            std::string calibration_file_path;
            if (!_pnh.getParam("calibration_file_path", calibration_file_path))
            {
                ROS_ERROR("The calibration_file_path parameter must be set to use a "
                                      "calibration file.");
                ROS_BREAK();
            }

            std::string calibration_file_name = "RTIMULib";
            if (!_pnh.getParam("calibration_file_name", calibration_file_name))
            {
                ROS_WARN_STREAM("No calibration_file_name provided - default: "
                                << calibration_file_name);
            }

            

            ros::Publisher imu_pub = _pnh.advertise<sensor_msgs::Imu>("camera/imu", 1);

            // Load the RTIMULib.ini config file
            RTIMUSettings *settings = new RTIMUSettings(calibration_file_path.c_str(),
                                                        calibration_file_name.c_str());

            RTIMU *imu = RTIMU::createIMU(settings);

            if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
            {
                ROS_ERROR("No Imu found");
                ROS_BREAK();
            }

            // Initialise the imu object
            imu->IMUInit();

            // Set the Fusion coefficient
            imu->setSlerpPower(0.02);
            // Enable the sensors
            imu->setGyroEnable(true);
            imu->setAccelEnable(true);
            imu->setCompassEnable(true);
            	static int i = 0;
                sensor_msgs::Imu imu_msg;
                while (ros::ok())
                {
                    if (imu->IMURead())
                    {

                        RTIMU_DATA imu_data = imu->getIMUData();

                        imu_msg.header.stamp = ros::Time::now();
                        imu_msg.header.frame_id = 1;

                        imu_msg.orientation.x = imu_data.fusionQPose.x();
                        imu_msg.orientation.y = imu_data.fusionQPose.y();
                        imu_msg.orientation.z = imu_data.fusionQPose.z();
                        imu_msg.orientation.w = imu_data.fusionQPose.scalar();

                        imu_msg.angular_velocity.x = imu_data.gyro.x();
                        imu_msg.angular_velocity.y = imu_data.gyro.y();
                        imu_msg.angular_velocity.z = imu_data.gyro.z();

                        imu_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
                        imu_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
                        imu_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;
                        if(running_)
                        {
                             imu_pub.publish(imu_msg);
                        }


                    }
                    ros::Duration(imu->IMUGetPollInterval() / 1000.0).sleep();
                }
        }

        void Start(const Start::ConstPtr &)
        {
             ROS_INFO_STREAM("Start message recived!");
            if(running_) return;
            running_= true;
            //std::thread([this]() {
                try
                {
                 

                    auto profile = pipe_->start(*configuration_);
                    while (running_)
                    {
                        rs2::frameset frame_set = pipe_->wait_for_frames();
                        //scan_traits<rs2::frameset>::convert_to(points);
                        if (frame_set.is<rs2::frameset>())
                        {
                            //~ ROS_DEBUG("Frameset arrived");
                            auto frameset = frame_set.as<rs2::frameset>();
                            for (auto it = frameset.begin(); it != frameset.end(); ++it)
                            {
                                auto frame = (*it);
                                auto stream_type = frame.get_profile().stream_type();
                                ros::Time t;
                                t = ros::Time(_ros_time_base.toSec() + (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000);
                                //ROS_DEBUG("Frameset contain %s frame. frame_number: %llu ; frame_TS: %f ; ros_TS(NSec): %lu",
                                          //rs2_stream_to_string(stream_type), frame.get_frame_number(), frame.get_timestamp(), t.toNSec());
                                publishFrame(frame, t);
                            }
                        }
                        ros::spinOnce();
                    }
                    ROS_INFO("Stoping pipeline");
                    pipe_->stop();
      
                }
                catch (const std::exception &ex)
                {
                    running_=false;
                    ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
                    throw;
                }
           // })
           //     .detach();
        }
        
        
        
        
        

        void Stop(const Start::ConstPtr &)
        {
            ROS_INFO_STREAM("Stop message recived!");

            running_=false;

           // std::this_thread::sleep_for(1s);
        }
        void getParameters()
        {
            ROS_INFO("getParameters...");

            _pnh = getPrivateNodeHandle();

            _pnh.param("enable_pointcloud", _pointcloud, POINTCLOUD);
            _pnh.param("enable_sync", _sync_frames, SYNC_FRAMES);
            if (_pointcloud)
                _sync_frames = true;

            _pnh.param("serial_no", _serial_no);

            _pnh.param("depth_width", _width[DEPTH], DEPTH_WIDTH);
            _pnh.param("depth_height", _height[DEPTH], DEPTH_HEIGHT);
            _pnh.param("depth_fps", _fps[DEPTH], DEPTH_FPS);
            _pnh.param("enable_depth", _enable[DEPTH], ENABLE_DEPTH);

            _pnh.param("infra1_width", _width[INFRA1], INFRA1_WIDTH);
            _pnh.param("infra1_height", _height[INFRA1], INFRA1_HEIGHT);
            _pnh.param("infra1_fps", _fps[INFRA1], INFRA1_FPS);
            _pnh.param("enable_infra1", _enable[INFRA1], ENABLE_INFRA1);

            _pnh.param("infra2_width", _width[INFRA2], INFRA2_WIDTH);
            _pnh.param("infra2_height", _height[INFRA2], INFRA2_HEIGHT);
            _pnh.param("infra2_fps", _fps[INFRA2], INFRA2_FPS);
            _pnh.param("enable_infra2", _enable[INFRA2], ENABLE_INFRA2);

            _pnh.param("color_width", _width[COLOR], COLOR_WIDTH);
            _pnh.param("color_height", _height[COLOR], COLOR_HEIGHT);
            _pnh.param("color_fps", _fps[COLOR], COLOR_FPS);
            _pnh.param("enable_color", _enable[COLOR], ENABLE_COLOR);
            _pnh.param("depth_scale", depth_scale_, DEFAULT_DEPTHSCALE);

            _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
            _pnh.param("depth_frame_id", _frame_id[DEPTH], DEFAULT_DEPTH_FRAME_ID);
            _pnh.param("infra1_frame_id", _frame_id[INFRA1], DEFAULT_INFRA1_FRAME_ID);
            _pnh.param("infra2_frame_id", _frame_id[INFRA2], DEFAULT_INFRA2_FRAME_ID);
            _pnh.param("color_frame_id", _frame_id[COLOR], DEFAULT_COLOR_FRAME_ID);


            _pnh.param("depth_optical_frame_id", _optical_frame_id[DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
            _pnh.param("infra1_optical_frame_id", _optical_frame_id[INFRA1], DEFAULT_INFRA1_OPTICAL_FRAME_ID);
            _pnh.param("infra2_optical_frame_id", _optical_frame_id[INFRA2], DEFAULT_INFRA2_OPTICAL_FRAME_ID);
            _pnh.param("color_optical_frame_id", _optical_frame_id[COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);

        }

        void setupDevice()
        {
            ROS_INFO("setupDevice...");
            try{
                configuration_ = std::make_unique<rs2::config>();
                pipe_ = std::make_unique<rs2::pipeline>();
                configuration_->enable_stream(RS2_STREAM_DEPTH,  _width[DEPTH], _height[DEPTH], RS2_FORMAT_ANY, _fps[DEPTH]);
                configuration_->enable_stream(RS2_STREAM_COLOR,  _width[COLOR], _height[COLOR], RS2_FORMAT_RGB8, _fps[COLOR]);
                auto profile = configuration_->resolve(*pipe_);

                auto sensor = profile.get_device().first<rs2::depth_sensor>();

                sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
                
				//ROS_INFO(("Setting depthscale to "+std::to_string(depth_scale_)).c_str());
                //sensor.set_option(RS2_OPTION_DEPTH_UNITS, depth_scale_);
                //rs2::region_of_interest roi;
                
                
                
                _ctx.reset(new rs2::context());

                auto list = _ctx->query_devices();
                if (0 == list.size())
                {
                    _ctx.reset();
                    ROS_ERROR("No RealSense devices were found! Terminate RealSense Node...");
                    ros::shutdown();
                    exit(1);
                }

                // Take the first device in the list.
                // TODO: Add an ability to get the specific device to work with from outside
                _dev = list.front();
                _ctx->set_devices_changed_callback([this](rs2::event_information& info)
                {
                    if (info.was_removed(_dev))
                    {
                        ROS_FATAL("The device has been disconnected! Terminate RealSense Node...");
                        ros::shutdown();
                        exit(1);
                    }
                });

                auto camera_name = _dev.get_info(RS2_CAMERA_INFO_NAME);
                ROS_INFO_STREAM("Device Name: " << camera_name);

                _serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                ROS_INFO_STREAM("Device Serial No: " << _serial_no);

                auto fw_ver = _dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
                ROS_INFO_STREAM("Device FW version: " << fw_ver);

                auto pid = _dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
                ROS_INFO_STREAM("Device Product ID: " << pid);

                ROS_INFO_STREAM("Sync Mode: " << ((_sync_frames)?"On":"Off"));

                auto dev_sensors = _dev.query_sensors();

                ROS_INFO_STREAM("Device Sensors: ");
                for(auto&& elem : dev_sensors)
                {
                    std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
                    if ("Stereo Module" == module_name || "Coded-Light Depth Sensor" == module_name)
                    {
                        auto sen = new rs2::sensor(elem);
                        _sensors[DEPTH] = std::unique_ptr<rs2::sensor>(sen);
                        _sensors[INFRA1] = std::unique_ptr<rs2::sensor>(sen);
                        _sensors[INFRA2] = std::unique_ptr<rs2::sensor>(sen);
                    }
                    else if ("RGB Camera" == module_name)
                    {
                        _sensors[COLOR] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense! Terminate RealSense Node...");
                        ros::shutdown();
                        exit(1);
                    }
                    ROS_INFO_STREAM(std::string(elem.get_info(RS2_CAMERA_INFO_NAME)) << " was found.");
                }

                // Update "enable" map
                std::vector<std::vector<stream_index_pair>> streams(IMAGE_STREAMS);
                for (auto& elem : streams)
                {
                    for (auto& stream_index : elem)
                    {
                        if (true == _enable[stream_index] && _sensors.find(stream_index) == _sensors.end()) // check if device supports the enabled stream
                        {
                            ROS_INFO_STREAM(rs2_stream_to_string(stream_index.first) << " sensor isn't supported by current device! -- Skipping...");
                            _enable[stream_index] = false;
                        }
                    }
                }
            }

                
            
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
                throw;
            }
            catch(...)
            {
                ROS_ERROR_STREAM("Unknown exception has occured!");
                throw;
            }
        }
       
        void setupSubscribers()
        {
            ROS_INFO("setupSubscriber...");
            start_subscriber_ = _node_handle.subscribe("camera/Start", 1, &RealSenseCameraNodelet::Start, this);
            stop_subscriber_ = _node_handle.subscribe("camera/Stop", 1, &RealSenseCameraNodelet::Stop, this);

        }
        void setupPublishers()
        {
            ROS_INFO("setupPublishers...");
            image_transport::ImageTransport image_transport(getMTNodeHandle());

            if (true == _enable[DEPTH])
            {
                _image_publishers[DEPTH] = image_transport.advertise("camera/depth/image_raw", 1);
                _info_publisher[DEPTH] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info", 1);

                if (_pointcloud)
                    _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
            }

       
            if (true == _enable[COLOR])
            {
                _image_publishers[COLOR] = image_transport.advertise("camera/color/image_raw", 1);
                _info_publisher[COLOR] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/color/camera_info", 1);
            }


        }
        
        void setupStreams()
        {
            ROS_INFO("setupStreams...");
            try{
                for (auto& streams : IMAGE_STREAMS)
                {
                    for (auto& elem : streams)
                    {
                        if (true == _enable[elem])
                        {
                            auto& sens = _sensors[elem];
                            auto profiles = sens->get_stream_profiles();
                            for (auto& profile : profiles)
                            {
                                auto video_profile = profile.as<rs2::video_stream_profile>();
                                if (video_profile.format() == _format[elem] &&
                                    video_profile.width()  == _width[elem] &&
                                    video_profile.height() == _height[elem] &&
                                    video_profile.fps()    == _fps[elem] &&
                                    video_profile.stream_index() == elem.second)
                                {
                                    _enabled_profiles[elem].push_back(profile);

                                    _image[elem] = cv::Mat(_width[elem], _height[elem], _image_format[elem], cv::Scalar(0, 0, 0));
                                    ROS_INFO_STREAM(_stream_name[elem] << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
                                    break;
                                }
                            }
                            if (_enabled_profiles.find(elem) == _enabled_profiles.end())
                            {
                                ROS_WARN_STREAM("Given stream configuration is not supported by the device! " <<
                                                " Stream: " << rs2_stream_to_string(elem.first) <<
                                                ", Format: " << _format[elem] <<
                                                ", Width: " << _width[elem] <<
                                                ", Height: " << _height[elem] <<
                                                ", FPS: " << _fps[elem]);
                                _enable[elem] = false;
                            }
                        }
                    }
                }

                // Publish image stream info
                for (auto& profiles : _enabled_profiles)
                {
                    for (auto& profile : profiles.second)
                    {
                        auto video_profile = profile.as<rs2::video_stream_profile>();
                        updateStreamCalibData(video_profile);
                    }
                }

                

                // Streaming IMAGES
                // for (auto& streams : IMAGE_STREAMS)
                // {
                //     std::vector<rs2::stream_profile> profiles;
                //     for (auto& elem : streams)
                //     {
                //         if (!_enabled_profiles[elem].empty())
                //         {
                //             profiles.insert(profiles.begin(),
                //                             _enabled_profiles[elem].begin(),
                //                             _enabled_profiles[elem].end());
                //         }
                //     }

                //     if (!profiles.empty())
                //     {
                //         auto stream = streams.front();
                //         auto& sens = _sensors[stream];
                //         sens->open(profiles);

                //         if (DEPTH == stream)
                //         {
                //             auto depth_sensor = sens->as<rs2::depth_sensor>();
                //             _depth_scale_meters = depth_sensor.get_depth_scale();
                //         }

                //         if (_sync_frames)
                //         {
                //             sens->start(_syncer);
                //         }
                //         else
                //         {
                //             sens->start(frame_callback);
                //         }
                //     }
                // }//end for

                // if (_sync_frames)
                // {
                //     _syncer.start(frame_callback);
                // }
                
            }
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
                throw;
            }
            catch(...)
            {
                ROS_ERROR_STREAM("Unknown exception has occured!");
                throw;
            }
        }

        void updateStreamCalibData(const rs2::video_stream_profile& video_profile)
        {
            stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
            auto intrinsic = video_profile.get_intrinsics();
            _stream_intrinsics[stream_index] = intrinsic;

            _camera_info[stream_index].width = intrinsic.width;
            _camera_info[stream_index].height = intrinsic.height;
            _camera_info[stream_index].header.frame_id = _optical_frame_id[stream_index];

            _camera_info[stream_index].K.at(0) = intrinsic.fx;
            _camera_info[stream_index].K.at(2) = intrinsic.ppx;
            _camera_info[stream_index].K.at(4) = intrinsic.fy;
            _camera_info[stream_index].K.at(5) = intrinsic.ppy;
            _camera_info[stream_index].K.at(8) = 1;

            _camera_info[stream_index].P.at(0) = _camera_info[stream_index].K.at(0);
            _camera_info[stream_index].P.at(1) = 0;
            _camera_info[stream_index].P.at(2) = _camera_info[stream_index].K.at(2);
            _camera_info[stream_index].P.at(3) = 0;
            _camera_info[stream_index].P.at(4) = 0;
            _camera_info[stream_index].P.at(5) = _camera_info[stream_index].K.at(4);
            _camera_info[stream_index].P.at(6) = _camera_info[stream_index].K.at(5);
            _camera_info[stream_index].P.at(7) = 0;
            _camera_info[stream_index].P.at(8) = 0;
            _camera_info[stream_index].P.at(9) = 0;
            _camera_info[stream_index].P.at(10) = 1;
            _camera_info[stream_index].P.at(11) = 0;

            rs2::stream_profile depth_profile;
            if (!getEnabledProfile(DEPTH, depth_profile))
            {
                ROS_ERROR_STREAM("Depth profile not found!");
                ros::shutdown();
                exit(1);
            }

            // TODO: Why Depth to Color?
            if (stream_index == DEPTH && _enable[DEPTH] && _enable[COLOR])
            {
                _depth2color_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[COLOR].front());
                // set depth to color translation values in Projection matrix (P)
                _camera_info[stream_index].P.at(3) = _depth2color_extrinsics.translation[0];     // Tx
                _camera_info[stream_index].P.at(7) = _depth2color_extrinsics.translation[1];     // Ty
                _camera_info[stream_index].P.at(11) = _depth2color_extrinsics.translation[2];    // Tz
            }

            _camera_info[stream_index].distortion_model = "plumb_bob";

            // set R (rotation matrix) values to identity matrix
            _camera_info[stream_index].R.at(0) = 1.0;
            _camera_info[stream_index].R.at(1) = 0.0;
            _camera_info[stream_index].R.at(2) = 0.0;
            _camera_info[stream_index].R.at(3) = 0.0;
            _camera_info[stream_index].R.at(4) = 1.0;
            _camera_info[stream_index].R.at(5) = 0.0;
            _camera_info[stream_index].R.at(6) = 0.0;
            _camera_info[stream_index].R.at(7) = 0.0;
            _camera_info[stream_index].R.at(8) = 1.0;

            for (int i = 0; i < 5; i++)
            {
                _camera_info[stream_index].D.push_back(intrinsic.coeffs[i]);
            }
        }

        Eigen::Quaternionf rotationMatrixToQuaternion(float rotation[3]) const
        {
            Eigen::Matrix3f m;
            m << rotation[0], rotation[1], rotation[2],
                 rotation[3], rotation[4], rotation[5],
                 rotation[6], rotation[7], rotation[8];
            Eigen::Quaternionf q(m);
            return q;
        }

        void publishStaticTransforms()
        {
            ROS_INFO("publishStaticTransforms...");
            // Publish transforms for the cameras
            tf::Quaternion q_c2co;
            geometry_msgs::TransformStamped b2c_msg; // Base to Color
            geometry_msgs::TransformStamped c2co_msg; // Color to Color_Optical

            tf::Quaternion q_d2do;
            geometry_msgs::TransformStamped b2d_msg; // Base to Depth
            geometry_msgs::TransformStamped d2do_msg; // Depth to Depth_Optical

            tf::Quaternion ir1_2_ir1o;
            geometry_msgs::TransformStamped b2ir1_msg; // Base to IR1
            geometry_msgs::TransformStamped ir1_2_ir1o_msg; // IR1 to IR1_Optical

            tf::Quaternion ir2_2_ir2o;
            geometry_msgs::TransformStamped b2ir2_msg; // Base to IR2
            geometry_msgs::TransformStamped ir2_2_ir2o_msg; // IR2 to IR2_Optical

            // Get the current timestamp for all static transforms
            ros::Time transform_ts_ = ros::Time::now();

            // The depth frame is used as the base frame.
            // Hence no additional transformation is done from base frame to depth frame.
            // Transform base frame to depth frame
            b2d_msg.header.stamp = transform_ts_;
            b2d_msg.header.frame_id = _base_frame_id;
            b2d_msg.child_frame_id = _frame_id[DEPTH];
            b2d_msg.transform.translation.x = 0;
            b2d_msg.transform.translation.y = 0;
            b2d_msg.transform.translation.z = 0;
            b2d_msg.transform.rotation.x = 0;
            b2d_msg.transform.rotation.y = 0;
            b2d_msg.transform.rotation.z = 0;
            b2d_msg.transform.rotation.w = 1;
            _static_tf_broadcaster.sendTransform(b2d_msg);

            // Transform depth frame to depth optical frame
            q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
            d2do_msg.header.stamp = transform_ts_;
            d2do_msg.header.frame_id = _frame_id[DEPTH];
            d2do_msg.child_frame_id = _optical_frame_id[DEPTH];
            d2do_msg.transform.translation.x = 0;
            d2do_msg.transform.translation.y = 0;
            d2do_msg.transform.translation.z = 0;
            d2do_msg.transform.rotation.x = q_d2do.getX();
            d2do_msg.transform.rotation.y = q_d2do.getY();
            d2do_msg.transform.rotation.z = q_d2do.getZ();
            d2do_msg.transform.rotation.w = q_d2do.getW();
            _static_tf_broadcaster.sendTransform(d2do_msg);

            rs2::stream_profile depth_profile;
            if (!getEnabledProfile(DEPTH, depth_profile))
            {
                ROS_ERROR_STREAM("Depth profile not found!");
                ros::shutdown();
                exit(1);
            }

            if (true == _enable[COLOR])
            {
                // Transform base frame to color frame
                auto q = rotationMatrixToQuaternion(_depth2color_extrinsics.rotation);

                b2c_msg.header.stamp = transform_ts_;
                b2c_msg.header.frame_id = _base_frame_id;
                b2c_msg.child_frame_id = _frame_id[COLOR];
                b2c_msg.transform.translation.x = _depth2color_extrinsics.translation[2];
                b2c_msg.transform.translation.y = -_depth2color_extrinsics.translation[0];
                b2c_msg.transform.translation.z = -_depth2color_extrinsics.translation[1];
                b2c_msg.transform.rotation.x = q.x();
                b2c_msg.transform.rotation.y = q.y();
                b2c_msg.transform.rotation.z = q.z();
                b2c_msg.transform.rotation.w = q.w();
                _static_tf_broadcaster.sendTransform(b2c_msg);

                // Transform color frame to color optical frame
                q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
                c2co_msg.header.stamp = transform_ts_;
                c2co_msg.header.frame_id = _frame_id[COLOR];
                c2co_msg.child_frame_id = _optical_frame_id[COLOR];
                c2co_msg.transform.translation.x = 0;
                c2co_msg.transform.translation.y = 0;
                c2co_msg.transform.translation.z = 0;
                c2co_msg.transform.rotation.x = q_c2co.getX();
                c2co_msg.transform.rotation.y = q_c2co.getY();
                c2co_msg.transform.rotation.z = q_c2co.getZ();
                c2co_msg.transform.rotation.w = q_c2co.getW();
                _static_tf_broadcaster.sendTransform(c2co_msg);
            }

            if (true == _enable[INFRA1])
            {
                auto d2ir1_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[INFRA1].front());
                auto q = rotationMatrixToQuaternion(d2ir1_extrinsics.rotation);

                // Transform base frame to infra1
                b2ir1_msg.header.stamp = transform_ts_;
                b2ir1_msg.header.frame_id = _base_frame_id;
                b2ir1_msg.child_frame_id = _frame_id[INFRA1];
                b2ir1_msg.transform.translation.x = d2ir1_extrinsics.translation[2];
                b2ir1_msg.transform.translation.y = -d2ir1_extrinsics.translation[0];
                b2ir1_msg.transform.translation.z = -d2ir1_extrinsics.translation[1];

                b2ir1_msg.transform.rotation.x = q.x();
                b2ir1_msg.transform.rotation.y = q.y();
                b2ir1_msg.transform.rotation.z = q.z();
                b2ir1_msg.transform.rotation.w = q.w();
                _static_tf_broadcaster.sendTransform(b2ir1_msg);

                // Transform infra1 frame to infra1 optical frame
                ir1_2_ir1o.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
                ir1_2_ir1o_msg.header.stamp = transform_ts_;
                ir1_2_ir1o_msg.header.frame_id = _frame_id[INFRA1];
                ir1_2_ir1o_msg.child_frame_id = _optical_frame_id[INFRA1];
                ir1_2_ir1o_msg.transform.translation.x = 0;
                ir1_2_ir1o_msg.transform.translation.y = 0;
                ir1_2_ir1o_msg.transform.translation.z = 0;
                ir1_2_ir1o_msg.transform.rotation.x = ir1_2_ir1o.getX();
                ir1_2_ir1o_msg.transform.rotation.y = ir1_2_ir1o.getY();
                ir1_2_ir1o_msg.transform.rotation.z = ir1_2_ir1o.getZ();
                ir1_2_ir1o_msg.transform.rotation.w = ir1_2_ir1o.getW();
                _static_tf_broadcaster.sendTransform(ir1_2_ir1o_msg);
            }

            if (true == _enable[INFRA2])
            {
                auto d2ir2_extrinsics = depth_profile.get_extrinsics_to(_enabled_profiles[INFRA2].front());
                auto q = rotationMatrixToQuaternion(d2ir2_extrinsics.rotation);

                // Transform base frame to infra2
                b2ir2_msg.header.stamp = transform_ts_;
                b2ir2_msg.header.frame_id = _base_frame_id;
                b2ir2_msg.child_frame_id = _frame_id[INFRA2];
                b2ir2_msg.transform.translation.x = d2ir2_extrinsics.translation[2];
                b2ir2_msg.transform.translation.y = -d2ir2_extrinsics.translation[0];
                b2ir2_msg.transform.translation.z = -d2ir2_extrinsics.translation[1];
                b2ir2_msg.transform.rotation.x = q.x();
                b2ir2_msg.transform.rotation.y = q.y();
                b2ir2_msg.transform.rotation.z = q.z();
                b2ir2_msg.transform.rotation.w = q.w();
                _static_tf_broadcaster.sendTransform(b2ir2_msg);

                // Transform infra2 frame to infra1 optical frame
                ir2_2_ir2o.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
                ir2_2_ir2o_msg.header.stamp = transform_ts_;
                ir2_2_ir2o_msg.header.frame_id = _frame_id[INFRA2];
                ir2_2_ir2o_msg.child_frame_id = _optical_frame_id[INFRA2];
                ir2_2_ir2o_msg.transform.translation.x = 0;
                ir2_2_ir2o_msg.transform.translation.y = 0;
                ir2_2_ir2o_msg.transform.translation.z = 0;
                ir2_2_ir2o_msg.transform.rotation.x = ir2_2_ir2o.getX();
                ir2_2_ir2o_msg.transform.rotation.y = ir2_2_ir2o.getY();
                ir2_2_ir2o_msg.transform.rotation.z = ir2_2_ir2o.getZ();
                ir2_2_ir2o_msg.transform.rotation.w = ir2_2_ir2o.getW();
                _static_tf_broadcaster.sendTransform(ir2_2_ir2o_msg);
            }
            // TODO: Publish Fisheye TF
        }

        void publishPCTopic(const ros::Time& t)
        {
            auto color_intrinsics = _stream_intrinsics[COLOR];
            auto image_depth16 = reinterpret_cast<const uint16_t*>(_image[DEPTH].data);
            auto depth_intrinsics = _stream_intrinsics[DEPTH];
            sensor_msgs::PointCloud2 msg_pointcloud;
            msg_pointcloud.header.stamp = t;
            msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
            msg_pointcloud.width = depth_intrinsics.width;
            msg_pointcloud.height = depth_intrinsics.height;
            msg_pointcloud.is_dense = true;

            sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

            modifier.setPointCloud2Fields(4,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "rgb", 1, sensor_msgs::PointField::FLOAT32);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

            sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
            sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
            sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");

            sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");

            float depth_point[3], color_point[3], color_pixel[2], scaled_depth;
            unsigned char* color_data = _image[COLOR].data;

            // Fill the PointCloud2 fields
            for (int y = 0; y < depth_intrinsics.height; ++y)
            {
                for (int x = 0; x < depth_intrinsics.width; ++x)
                {
                    scaled_depth = static_cast<float>(*image_depth16) * _depth_scale_meters;
                    float depth_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    rs2_deproject_pixel_to_point(depth_point, &depth_intrinsics, depth_pixel, scaled_depth);

                    if (depth_point[2] <= 0.f || depth_point[2] > 5.f)
                    {
                        depth_point[0] = 0.f;
                        depth_point[1] = 0.f;
                        depth_point[2] = 0.f;
                    }

                    *iter_x = depth_point[0];
                    *iter_y = depth_point[1];
                    *iter_z = depth_point[2];

                    rs2_transform_point_to_point(color_point, &_depth2color_extrinsics, depth_point);
                    rs2_project_point_to_pixel(color_pixel, &color_intrinsics, color_point);

                    if (color_pixel[1] < 0.f || color_pixel[1] > color_intrinsics.height
                        || color_pixel[0] < 0.f || color_pixel[0] > color_intrinsics.width)
                    {
                        // For out of bounds color data, default to a shade of blue in order to visually distinguish holes.
                        // This color value is same as the librealsense out of bounds color value.
                        *iter_r = static_cast<uint8_t>(96);
                        *iter_g = static_cast<uint8_t>(157);
                        *iter_b = static_cast<uint8_t>(198);
                    }
                    else
                    {
                        auto i = static_cast<int>(color_pixel[0]);
                        auto j = static_cast<int>(color_pixel[1]);

                        auto offset = i * 3 + j * color_intrinsics.width * 3;
                        *iter_r = static_cast<uint8_t>(color_data[offset]);
                        *iter_g = static_cast<uint8_t>(color_data[offset + 1]);
                        *iter_b = static_cast<uint8_t>(color_data[offset + 2]);
                    }

                    ++image_depth16;
                    ++iter_x; ++iter_y; ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }
            }
            _pointcloud_publisher.publish(msg_pointcloud);
        }

        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics) const
        {
            Extrinsics extrinsicsMsg;
            for (int i = 0; i < 9; ++i)
            {
                extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
                if (i < 3)
                    extrinsicsMsg.translation[i] = extrinsics.translation[i];
            }

            return extrinsicsMsg;
        }


        struct float3
        {
            float x, y, z;
        };

       

        void tryGetLogSeverity(rs2_log_severity& severity) const
        {
            static const char* severity_var_name = "LRS_LOG_LEVEL";
            auto content = getenv(severity_var_name);

            if (content)
            {
                std::string content_str(content);
                std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

                for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
                {
                    auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
                    std::transform(current.begin(), current.end(), current.begin(), ::toupper);
                    if (content_str == current)
                    {
                        severity = (rs2_log_severity)i;
                        break;
                    }
                }
            }
        }

        void publishFrame(rs2::frame f, const ros::Time& t)
        {
            ROS_DEBUG("publishFrame(...)");
            stream_index_pair stream{f.get_profile().stream_type(), f.get_profile().stream_index()};
            auto& image = _image[stream];
            image.data = (uint8_t*)f.get_data();
            ++(_seq[stream]);
            auto& info_publisher = _info_publisher[stream];
            auto& image_publisher = _image_publishers[stream];
            if(0 != info_publisher.getNumSubscribers() ||
               0 != image_publisher.getNumSubscribers())
            {
                auto width = 0;
                auto height = 0;
                auto bpp = 1;
                if (f.is<rs2::video_frame>())
                {
                    auto image = f.as<rs2::video_frame>();
                    width = image.get_width();
                    height = image.get_height();
                    bpp = image.get_bytes_per_pixel();
                }

                sensor_msgs::ImagePtr img;
                img = cv_bridge::CvImage(std_msgs::Header(), _encoding[stream], image).toImageMsg();
                img->width = width;
                img->height = height;
                img->is_bigendian = false;
                img->step = width * bpp;
                img->header.frame_id = _optical_frame_id[stream];
                img->header.stamp = t;
                img->header.seq = _seq[stream];

                auto& cam_info = _camera_info[stream];
                cam_info.header.stamp = t;
                cam_info.header.seq = _seq[stream];
                info_publisher.publish(cam_info);

                image_publisher.publish(img);
                ROS_DEBUG("%s stream published", rs2_stream_to_string(f.get_profile().stream_type()));
            }
        }

        bool getEnabledProfile(const stream_index_pair& stream_index, rs2::stream_profile& profile)
        {
            // Assuming that all D400 SKUs have depth sensor
            auto profiles = _enabled_profiles[stream_index];
            auto it = std::find_if(profiles.begin(), profiles.end(),
                                   [&](const rs2::stream_profile& profile)
                                   { return (profile.stream_type() == stream_index.first); });
            if (it == profiles.end())
                return false;

            profile =  *it;
            return true;
        }

        ros::NodeHandle _node_handle, _pnh;
        std::unique_ptr<rs2::context> _ctx;
        rs2::device _dev;
        std::unique_ptr<rs2::config> configuration_;
        std::unique_ptr<rs2::pipeline> pipe_;

        std::map<stream_index_pair, std::unique_ptr<rs2::sensor>> _sensors;

        std::string _serial_no;std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;
        float _depth_scale_meters;

        std::map<stream_index_pair, rs2_intrinsics> _stream_intrinsics;
        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<stream_index_pair, bool> _enable;
        std::map<stream_index_pair, std::string> _stream_name;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

        std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
        std::map<stream_index_pair, ros::Publisher> _imu_publishers;
        std::map<stream_index_pair, int> _image_format;
        std::map<stream_index_pair, rs2_format> _format;
        std::map<stream_index_pair, ros::Publisher> _info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<stream_index_pair, std::string> _encoding;

        std::string _base_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, int> _seq;
        std::map<stream_index_pair, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
        ros::Publisher _fe_to_depth_publisher, _fe_to_imu_publisher;
        bool _intialize_time_base;
        double _camera_time_base;

        ros::Publisher _pointcloud_publisher;
        ros::Time _ros_time_base;
        bool _sync_frames;
        bool _pointcloud;
        rs2::asynchronous_syncer _syncer;
        rs2_extrinsics _depth2color_extrinsics;

        std::atomic<bool> running_;
        float depth_scale_;
        std::unique_ptr<rs2::recorder> recorder_;
        bool record_to_file_;
        ros::Subscriber start_subscriber_;
        ros::Subscriber stop_subscriber_;
        sensor_msgs::Imu current_imu_msg_;
        std::mutex imu_mutex;

    };//end class

    PLUGINLIB_EXPORT_CLASS(realsense_ros_camera::RealSenseCameraNodelet, nodelet::Nodelet)

}//end namespace
