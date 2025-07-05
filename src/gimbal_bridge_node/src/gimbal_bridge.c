#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define RECV_BUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.144.25"

int main(int argc, char *argv[]) {
    int sockfd;
    int ret, i, recv_len;
    struct sockaddr_in send_addr, recv_addr;

    // 定义要发送的帧数据
    unsigned char send_buf[] = {0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x08,0x01,0xd1,0x12};
    unsigned char recv_buf[RECV_BUF_SIZE] = {0};

    /* 创建 UDP 套接字
       AF_INET:  ipv4 地址
       SOCK_DGRAM: UDP 协议
       0:          自动选择类型对应的默认协议
    */
    if (((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)) {
        perror("socket");
        exit(1);
    }

    /* 设置云台相机的 ip 和端口号
       sin_family:     ipv4 地址
       sin_addr.s_addr:云台相机 IP 地址
       sin_port:       云台相机端口号
    */
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    send_addr.sin_port = htons(SERVER_PORT);

    /* 发送帧数据
       sockfd:                socket 套接字文件描述符
       send_buf:              要发送的数据在内存中的首地址
       sizeof(send_buf):      要发送的数据的长度
       0:                     发送标志，一般为 0
       (struct sockaddr *)&send_addr: 数据接收端的地址（包含 IP 地址和端口号）的结构体指针
       addr_len:              数据接收端地址结构体的大小
    */
    printf("Send HEX data\n");
    socklen_t addr_len = sizeof(struct sockaddr_in);
    if (sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&send_addr, addr_len) < 0) {
        perror("sendto");
        exit(1);
    }

    /* 接收云台相机的返回数据 
       sockfd:                socket 套接字文件描述符
       recv_buf:              接收到的数据存放在内存中的位置
       RECV_BUF_SIZE:         指 buf bag的大小，即期望接收的最大数据的长度
       0:                     接收标志，一般为 0
       (struct sockaddr *)&recv_addr: 指向的结构体将被数据发送端的地址（含 IP 地址和端口号 ）所填充
       &addr_len:             所指向的存储位置，调用前应填入 src_addr 和 addr_len 的结构体大小，调用后则将被填入发送端的地址的实际大小
    */
    recv_len = recvfrom(sockfd, recv_buf, RECV_BUF_SIZE, 0, (struct sockaddr *)&recv_addr, &addr_len);
    if (recv_len < 0) {
        perror("recvfrom");
        exit(1);
    }

    // 十六进制形式打印接收到的数据
    printf("Received HEX data: ");
    for (int i = 0; i < recv_len; i++) {
        printf("%02x ", recv_buf[i]);
    }
    printf("\n");

    // 关闭套接字
    close(sockfd);
    return 0;
}