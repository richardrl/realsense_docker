// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <signal.h>
#include <iomanip>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>       /* ceil */

// opencv stuff
#include <opencv2/core.hpp>
#include <opencv2/aruco/charuco.hpp>
//#include <opencv2/highgui.hpp>
#include <tuple>


std::tuple<cv::aruco::CharucoBoard*, cv::aruco::Dictionary> make_board(int board_num,
int num_rows=4, int num_cols=4, float marker_size=0.018, float square_size=0.024)
{
    int num_markers = ceil(num_rows * num_cols / 2);
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
//    dictionary.bytesList = dictionary.bytesList[board_num*num_markers:board_num*num_markers+num_markers,...];
    dictionary.bytesList = dictionary.bytesList.rowRange(board_num*num_markers, board_num*num_markers+num_markers);

    cv::aruco::CharucoBoard* board = cv::aruco::CharucoBoard::create(num_cols,num_rows,square_size,marker_size, &dictionary);

    return std::make_tuple(board, dictionary);
}



//------------------- TCP Server Code -------------------
//-------------------------------------------------------

typedef void * (*THREADFUNCPTR)(void *);

class Server {

    public:
        Server(int port);
        void * listener_thread();
        void init_listener_thread();
        void update_buffer(const unsigned char * data, int offset, unsigned long numbytes);

    private:
        int init_sock, conn_sock;
        char * send_buffer;
        int buffer_size = 1024;
        char receive_buffer[1024];
        struct sockaddr_in serv_addr;
        struct sockaddr_storage serv_storage;
        socklen_t addr_size;
        pthread_mutex_t buffer_access_mutex;
        pthread_t listener_thread_id;
        unsigned long frame_size;
};

Server::Server(int port) {
    init_sock = socket(PF_INET, SOCK_STREAM, 0);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons (port);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
    bind(init_sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    send_buffer = new char[buffer_size];
}

void Server::init_listener_thread() {
    pthread_create(&listener_thread_id, NULL, (THREADFUNCPTR) &Server::listener_thread, this);
    pthread_mutex_init(&buffer_access_mutex, NULL);
}

void * Server::listener_thread() {
    while(true) {
        if (listen(init_sock, 5) == 0)
            printf ("Listening...\n");
        else
            printf ("Error.\n");

        // Creates new socket for incoming connection
        addr_size = sizeof(serv_storage);
        conn_sock = accept (init_sock, (struct sockaddr *) &serv_storage, &addr_size);
        printf ("Connected to client.\n");

        while(true) {

            // Parse ping from client
            memset(receive_buffer, 0, sizeof(receive_buffer));
            int resp_msg_size = recv(conn_sock, receive_buffer, 64, 0);
            if (resp_msg_size <= 0) break;

            // Send buffer data
            pthread_mutex_lock(&buffer_access_mutex);
            int msg_size = send(conn_sock, send_buffer, buffer_size, MSG_MORE);
            if (msg_size == 0 ) printf("Warning: No data was sent to client.\n");
            int tmp = errno;
            if (msg_size < 0) printf ("Errno %d\n", tmp);
            pthread_mutex_unlock(&buffer_access_mutex);
        }
    }
}

void Server::update_buffer(const unsigned char * data, int offset, unsigned long numbytes) {
    pthread_mutex_lock(&buffer_access_mutex);

    // Update buffer size
    unsigned long new_buffer_size = numbytes + offset;
    if (new_buffer_size > buffer_size) {
        delete [] send_buffer;
        buffer_size = new_buffer_size;
        send_buffer = new char[buffer_size];
    }

    // Copy data
    memcpy(send_buffer + offset, data, numbytes);
    pthread_mutex_unlock(&buffer_access_mutex);
}

//-------------------------------------------------------
//-------------------------------------------------------

// Configure all streams to run at 1280x720 resolution at 30 frames per second
const int stream_width = 1280;
const int stream_height = 720;
const int stream_fps = 30;
const int depth_disparity_shift = 50;

// Convert from video frame to opencv mat,
// Get transform, display axises
// Then convert back
void display_transform(rs2::video_frame color_frame) {
    // convert to color frame
    cv::Mat matColor(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

    // get transform
    // apply axis visualization


    // convert back to
}

// Capture color and depth video streams, render them to the screen, send them through TCP
int main(int argc, char * argv[]) try {
    // Create a simple OpenGL window for rendering:
    window app(2560, 720, "RealSense Stream");

    // Check if RealSense device is connected
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
      std::cerr << "No device connected, please connect a RealSense device" << std::endl;
      return EXIT_FAILURE;
    }

    std::vector<Server> realsense_server_arr;
//    std::vector<window> window_arr;

    for (int i=0; i<devices.size(); ++i) {
        Server realsense_server(50000 + i);
        realsense_server.init_listener_thread();
        realsense_server_arr.emplace_back(realsense_server);
        }
//
//        window app(2560, 720, "RealSense Stream");
//        window_arr.emplace_back(app);
//    }

    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;


    // Initialize & store array of multiple pipelines
    std::vector<rs2::pipeline>  pipelines;
    for (auto&& dev : ctx.query_devices())
    {
//        rs2::pipeline pipe(ctx);
//        rs2::config cfg;
//        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        // Configure streams
        rs2::config config_pipe;
        config_pipe.enable_stream(rs2_stream::RS2_STREAM_DEPTH, stream_width, stream_height, RS2_FORMAT_Z16, stream_fps);
        config_pipe.enable_stream(rs2_stream::RS2_STREAM_COLOR, stream_width, stream_height, RS2_FORMAT_RGB8, stream_fps);
        config_pipe.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        rs2::pipeline pipe;
        pipe.start(config_pipe);
        pipelines.emplace_back(pipe);

        std::cout << "Device information: " << std::endl;
        for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++) {
              rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
              std::cout << "  " << std::left << std::setw(20) << info_type << " : ";
              if (dev.supports(info_type))
                  std::cout << dev.get_info(info_type) << std::endl;
              else
                  std::cout << "N/A" << std::endl;
        }

        // Capture 30 frames to give autoexposure, etc. a chance to settle
        for (int i = 0; i < 30; ++i) pipe.wait_for_frames();
    }

    rs2::align align(rs2_stream::RS2_STREAM_COLOR);
    while(app) {
//        for (auto&& dev : ctx.query_devices())
//        for (auto&& pipe : pipelines)

        for (int device_idx=0; device_idx<devices.size(); ++device_idx)
        {
//            rs2::pipeline_profile active_pipe_profile = pipe.get_active_profile();
//            rs2::device dev = active_pipe_profile.get_device();
            // Get active device sensors

            rs2::device dev = ctx.query_devices()[device_idx];

            rs2::pipeline pipe = pipelines[device_idx];

            // get intrinsics
            rs2::pipeline_profile active_pipe_profile = pipe.get_active_profile();
            rs2::video_stream_profile color_stream_profile = active_pipe_profile.get_stream(rs2_stream::RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            rs2_intrinsics color_intrinsics = color_stream_profile.get_intrinsics();
            float color_intrinsics_arr[9] = {color_intrinsics.fx, 0.0f, color_intrinsics.ppx,
                                             0.0f, color_intrinsics.fy, color_intrinsics.ppy,
                                             0.0f, 0.0f, 1.0f};



            std::vector<rs2::sensor> sensors = dev.query_sensors();
            rs2::sensor depth_sensor = sensors[0];
            rs2::sensor color_sensor = sensors[1];


            float depth_scale = depth_sensor.as<rs2::depth_sensor>().get_depth_scale();


            // Wait for next set of frames from the camera
            rs2::frameset data = pipe.wait_for_frames();
            rs2::frame color = data.get_color_frame();

            // Get both raw and aligned depth frames
            auto processed = align.process(data);
            rs2::depth_frame aligned_depth = processed.get_depth_frame();

            // Find and colorize the depth data
            rs2::frame depth_colorized = aligned_depth.apply_filter(color_map);

            std::cout << "Updating device " << device_idx << std::endl;


            int depth_size = aligned_depth.get_width()*aligned_depth.get_height()*aligned_depth.get_bytes_per_pixel();
            realsense_server_arr[device_idx].update_buffer((unsigned char*)aligned_depth.get_data(), 10*4, depth_size);

            int color_size = data.get_color_frame().get_width()*data.get_color_frame().get_height()*data.get_color_frame().get_bytes_per_pixel();
            realsense_server_arr[device_idx].update_buffer((unsigned char*)color.get_data(), 10*4 + depth_size, color_size);

            // Send camera intrinsics and depth scale
            realsense_server_arr[device_idx].update_buffer((unsigned char*)color_intrinsics_arr, 0, 9*4);
            realsense_server_arr[device_idx].update_buffer((unsigned char*)&depth_scale, 9*4, 4);

            // Render depth on to the first half of the screen and color on to the second
//            depth_image.render(depth_colorized, { 0, 0, app.width() / 2, app.height() });
//            color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });

//            display_transform(color);

            depth_image.render(depth_colorized, {device_idx*app.width() / devices.size(), 0, app.width() / devices.size(), app.height() / 2});
            color_image.render(color, {device_idx*app.width() / devices.size(), app.height() / 2, app.width() / devices.size(), app.height() / 2});
            }
       }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


