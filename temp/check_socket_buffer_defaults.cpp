// 检查socket缓冲区默认值的程序
// 编译: g++ -o check_socket_buffer_defaults check_socket_buffer_defaults.cpp
// 运行: ./check_socket_buffer_defaults

#include <iostream>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <iomanip>

int main() {
    // 创建CAN socket
    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        perror("socket");
        return 1;
    }

    // 获取默认的接收缓冲区大小
    int rcvbuf = 0;
    socklen_t len = sizeof(rcvbuf);
    if (getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, &len) == 0) {
        std::cout << "Default SO_RCVBUF: " << rcvbuf << " bytes ("
                  << std::fixed << std::setprecision(2) << rcvbuf / 1024.0 << " KB)" << std::endl;
    } else {
        perror("getsockopt(SO_RCVBUF)");
    }

    // 获取默认的发送缓冲区大小
    int sndbuf = 0;
    len = sizeof(sndbuf);
    if (getsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, &len) == 0) {
        std::cout << "Default SO_SNDBUF: " << sndbuf << " bytes ("
                  << std::fixed << std::setprecision(2) << sndbuf / 1024.0 << " KB)" << std::endl;
    } else {
        perror("getsockopt(SO_SNDBUF)");
    }

    // 获取最大值
    int rcvbuf_max = 0;
    len = sizeof(rcvbuf_max);
    if (getsockopt(sockfd, SOL_SOCKET, SO_RCVBUFFORCE, &rcvbuf_max, &len) == 0) {
        // 如果支持，显示最大值
    }
    
    // 尝试读取系统参数文件
    std::cout << "\nSystem parameters (if accessible):" << std::endl;
    FILE* fp = fopen("/proc/sys/net/core/rmem_default", "r");
    if (fp) {
        int val;
        if (fscanf(fp, "%d", &val) == 1) {
            std::cout << "  /proc/sys/net/core/rmem_default: " << val 
                      << " bytes (" << val / 1024.0 << " KB)" << std::endl;
        }
        fclose(fp);
    }
    
    fp = fopen("/proc/sys/net/core/wmem_default", "r");
    if (fp) {
        int val;
        if (fscanf(fp, "%d", &val) == 1) {
            std::cout << "  /proc/sys/net/core/wmem_default: " << val 
                      << " bytes (" << val / 1024.0 << " KB)" << std::endl;
        }
        fclose(fp);
    }

    close(sockfd);
    return 0;
}
