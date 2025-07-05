#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <gimbal_bridge_node/siyi_zr10_protocol.h>
#include <gimbal_bridge_node/GimbalState.h>

constexpr int RECV_BUF_SIZE = 64;
constexpr int SERVER_PORT = 37260;
constexpr char SERVER_IP[] = "192.168.144.25";

class gimbal_camera_state_bridge
{
public:
    // 构造函数，可指定IP和端口，默认使用题目给定的参数
    gimbal_camera_state_bridge(const std::string &server_ip = "192.168.144.25", uint16_t server_port = 37260)
        : server_ip_(server_ip), server_port_(server_port)
    {
        memset(&send_addr_, 0, sizeof(send_addr_));
        send_addr_.sin_family = AF_INET;
        send_addr_.sin_port = htons(server_port_);

        // 转换IP地址
        if (inet_pton(AF_INET, server_ip_.c_str(), &send_addr_.sin_addr) <= 0)
        {
            throw std::invalid_argument("Invalid IP address: " + server_ip_);
        }
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            ROS_ERROR("Failed to create socket: %s", strerror(errno));
        }
        ROS_INFO("Socket created successfully, ready to send data to %s:%d", server_ip_.c_str(), server_port_);
    }
    ~gimbal_camera_state_bridge()
    {
        close(sockfd);
    }
    void ping()
    {
        // 定义请求数据
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_get_firmware_version(send_buf, RECV_BUF_SIZE, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack request data");
            return;
        }
        ROS_INFO("Packed request data length: %d", request_length);

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            close(sockfd);
            return;
        }
        // 接收响应
        struct sockaddr_in recv_addr;
        socklen_t addr_len = sizeof(recv_addr);
        unsigned char recv_buf[RECV_BUF_SIZE] = {0};
        int recv_len = recvfrom(sockfd, recv_buf, RECV_BUF_SIZE, 0,
                                (struct sockaddr *)&recv_addr, &addr_len);
        if (recv_len < 0)
        {
            ROS_ERROR("recvfrom failed: %s", strerror(errno));
            close(sockfd);
            return;
        }
        FirmwareVersionResponse response;

        // 解析响应
        if (!siyi_unpack_firmware_version_response(recv_buf, recv_len, &response))
        {
            ROS_ERROR("Failed to unpack response data");
            return;
        }
        // 打印接收到的数据
        ROS_INFO("Get firmware version response:");
        ROS_INFO("Code Board Version: %u", response.code_board_ver);
        ROS_INFO("Gimbal Firmware Version: %u", response.gimbal_firmware_ver);
        ROS_INFO("Zoom Firmware Version: %u", response.zoom_firmware_ver);
    }

    void request_gimbal_state()
    {
        // 定义请求数据
        uint8_t send_buf[RECV_BUF_SIZE] = {0};
        uint16_t request_length = siyi_pack_request_data_stream(send_buf, RECV_BUF_SIZE, 0x01, 0x07, 0x0000);
        if (request_length == 0)
        {
            ROS_ERROR("Failed to pack request data");
            return;
        }
        ROS_INFO("Packed request data length: %d", request_length);

        // 发送数据
        if (sendto(sockfd, send_buf, request_length, 0,
                   (struct sockaddr *)&send_addr_, sizeof(send_addr_)) < 0)
        {
            ROS_ERROR("sendto failed: %s", strerror(errno));
            return;
        }
    }

    bool receive_and_publish()
    {
        // 接收响应
        struct sockaddr_in recv_addr;
        socklen_t addr_len = sizeof(recv_addr);
        unsigned char bag[RECV_BUF_SIZE] = {0};
        int bag_len = recvfrom(sockfd, bag, RECV_BUF_SIZE, 0,
                                (struct sockaddr *)&recv_addr, &addr_len);
        if (bag_len < 0)
        {
            ROS_ERROR("recvfrom failed: %s", strerror(errno));
            return false;
        }
        std::cout << "Received data length: " << bag_len << std::endl;
        std::cout << "Received data: ";
        for (int i = 0; i < bag_len; i++)
        {
            printf("%02x ", bag[i]);
        }
        printf("\n");
        AttitudeDataResponse response;
        // 解析响应
        if (!siyi_unpack_attitude_data_response(bag, bag_len, &response))
        {
            ROS_ERROR("Failed to unpack response data");
            return false;
        }
        // 打印接收到的数据
        ROS_INFO("Get gimbal state response:");
        ROS_INFO("Yaw: %d", response.yaw);
        ROS_INFO("Pitch: %d", response.pitch);
        ROS_INFO("Roll: %d", response.roll);
        ROS_INFO("Yaw Velocity: %d", response.yaw_velocity);
        ROS_INFO("Pitch Velocity: %d", response.pitch_velocity);
        ROS_INFO("Roll Velocity: %d", response.roll_velocity);
        // 这里可以将接收到的角度数据转换为 ROS 消息并发布
        gimbal_bridge_node::GimbalState msg;
        msg.yaw = static_cast<float>(response.yaw) / 10.0f;
        msg.pitch = static_cast<float>(response.pitch) / 10.0f;
        msg.roll = static_cast<float>(response.roll) / 10.0f;
        msg.json_string = "{\"status\":\"normal\"}";
        // 发布消息
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<gimbal_bridge_node::GimbalState>("/gimbal/state", 10);
        pub.publish(msg);
        // ROS_INFO("[Publisher] Published: yaw=%.1f, pitch=%.1f, roll=%.1f, json=%s",
        //          msg.yaw, msg.pitch, msg.roll, msg.json_string.c_str());
        return true;
    }

private:
    static constexpr int RECV_BUF_SIZE = 64;
    std::string server_ip_;
    uint16_t server_port_;
    struct sockaddr_in send_addr_;
    int sockfd;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "gimbal_state_publisher");
    ros::NodeHandle nh;

    // 创建 Publisher，Topic 名字和消息类型
    ros::Publisher pub = nh.advertise<gimbal_bridge_node::GimbalState>("/gimbal/state", 10);

    // 设置循环频率 (100Hz)
    ros::Rate loop_rate(1);

    gimbal_camera_state_bridge gimbal_bridge(SERVER_IP, SERVER_PORT);
    gimbal_bridge.ping();
    gimbal_bridge.request_gimbal_state();

    while (ros::ok())
    {
        gimbal_bridge.receive_and_publish();
        // // 创建消息实例并填充数据
        // gimbal_bridge_node::GimbalState msg;
        // msg.yaw = 10.5;
        // msg.pitch = -5.3;
        // msg.roll = 2.7;
        // msg.json_string = "{\"status\":\"normal\"}";

        // // 发布消息
        // pub.publish(msg);

        // // 输出日志
        // ROS_INFO("[Publisher] Published: yaw=%.1f, pitch=%.1f, roll=%.1f, json=%s",
        //          msg.yaw, msg.pitch, msg.roll, msg.json_string.c_str());

        // 休眠以保持频率
        loop_rate.sleep();
    }

    return 0;
}
