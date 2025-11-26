#include "stdio.h"
#include <pthread.h>
#include <cstdlib>
#include <string>
#include <ctime>
#include <chrono>
using namespace std;

#include"payloadSdkInterface.h"

#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {
    CONTROL_UART,
    payload_uart_port,
    payload_uart_baud
};
#else
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
#endif

PayloadSdkInterface* my_payload = nullptr;
bool time_to_exit = false;

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);
std::string getCurrentTimeString();

int main(int argc, char *argv[]){
    printf("[%s] 开始云台俯仰测试 - 将俯仰角设置为 -30 度并保持...\n", getCurrentTimeString().c_str());
    signal(SIGINT,quit_handler);

    // create payloadsdk object
    my_payload = new PayloadSdkInterface(s_conn);

    // init payload
    printf("[%s] 正在初始化云台连接...\n", getCurrentTimeString().c_str());
    my_payload->sdkInitConnection();
    printf("[%s] 等待云台信号!\n", getCurrentTimeString().c_str());

    // register callback function to monitor gimbal attitude
    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);

    // check connection
    my_payload->checkPayloadConnection();
    usleep(500000);

    printf("[%s] 设置云台模式为 FOLLOW...\n", getCurrentTimeString().c_str());
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
    usleep(1000000);

    // Set gimbal pitch to -30 degrees
    printf("[%s] === 设置云台俯仰角为 -30 度 ===\n", getCurrentTimeString().c_str());
    my_payload->setGimbalSpeed(-30, 0, 0, Gimbal_Protocol::INPUT_ANGLE);
    
    // Wait for movement to complete
    printf("[%s] 等待 6 秒以达到目标位置...\n", getCurrentTimeString().c_str());
    usleep(6000000);

    printf("[%s] === 已到达 -30 度位置，保持该姿态 ===\n", getCurrentTimeString().c_str());
    printf("[%s] 按 Ctrl+C 退出程序\n", getCurrentTimeString().c_str());

    // Keep the program running to maintain gimbal position
    while(!time_to_exit) {
        usleep(1000000); // Sleep for 1 second
    }

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    return 0;
}

void quit_handler( int sig ){
    printf("\n");
    printf("[%s] 用户请求终止程序\n", getCurrentTimeString().c_str());
    printf("\n");

    time_to_exit = true;

    // close payload interface
    try {
        my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}

void onPayloadStatusChanged(int event, double* param){
    switch(event){
        case PAYLOAD_GB_ATTITUDE:{
            // param[0]: pitch
            // param[1]: roll
            // param[2]: yaw
            printf("[%s] 云台姿态 - 俯仰: %.2f° - 横滚: %.2f° - 偏航: %.2f°\n", 
                   getCurrentTimeString().c_str(), param[0], param[1], param[2]);
            break;
        }
        default: break;
    }
}

std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&time_t));
    
    char result[150];
    sprintf(result, "%s.%03d", buffer, (int)ms.count());
    return std::string(result);
}

