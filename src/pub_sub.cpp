#if (_MSC_VER >= 1915)
#define no_init_all deprecated
#endif

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#include <time.h>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include "D2XX/ftd2xx.h"
#pragma comment(lib, "ftd2xx.lib")
using namespace std::chrono_literals;
using std::placeholders::_1;

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned long  uint32;

const uint16 L = 0x2D2C;
const uint16 H = L | 0x1010;
const uint16 TMS = L | 0x0202;
const uint16 TMS_H = TMS | H;
const uint16 OFF = 0x0D0C;
const uint16 WR = 0x0080;
const uint16 RD = 0x00C0;


void MoveIdle(FT_HANDLE ftHandle)
{
    uint32 send_size;
    uint16 buf[] = { TMS, TMS, TMS, TMS, TMS, L };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

void MoveIdleToShiftir(FT_HANDLE ftHandle)
{
    uint32 send_size;
    uint16 buf[] = { TMS, TMS, L, L };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

void WriteShiftdr(FT_HANDLE ftHandle, uint8 drCodeValue)
{
    uint32 send_size;
    uint16 buf[] = { (uint16)(((uint16)drCodeValue << 8) | WR | 0x0001) };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

void WriteShiftir(FT_HANDLE ftHandle, uint8 irCodeValue)
{
    uint32 send_size;
    uint16 buf[] = { (uint16)(((uint16)irCodeValue << 8) | WR | 0x0001), L };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

void MoveShiftirToShiftdr(FT_HANDLE ftHandle)
{
    uint32 send_size;
    uint16 buf[] = { TMS, TMS, TMS, L, L };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

void MoveShiftdrToShiftir(FT_HANDLE ftHandle)
{
    uint32 send_size;
    uint16 buf[] = { TMS_H, TMS, TMS, TMS, L, L };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

void DeviceClose(FT_HANDLE ftHandle)
{
    uint32 send_size;
    uint16 buf[] = { TMS_H, TMS, OFF };
    FT_Write(ftHandle, buf, sizeof(buf), &send_size);
}

uint8 check_sum(uint8* data, int size)
{
    uint8 sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum;
}

int BlasterSend(FT_HANDLE ftHandle, uint8* send_data, const int send_size)
{
    if (send_data == NULL) return -1;
    if (send_size <= 0) return -1;

    MoveIdle(ftHandle);
    MoveIdleToShiftir(ftHandle);
    WriteShiftir(ftHandle, 0x0E); // USER1
    MoveShiftirToShiftdr(ftHandle);
    WriteShiftdr(ftHandle, 0x41); // for counter reset_FPGA recv
    MoveShiftdrToShiftir(ftHandle);
    WriteShiftir(ftHandle, 0x0C); // USER0
    MoveShiftirToShiftdr(ftHandle);


    DWORD bytes_to_write = ((send_size - 1) / 63) + send_size + 1;


    int write_buf_size = (bytes_to_write + 1) / 2;

    write_buf_size = ((write_buf_size - 1) / 32 + 1) * 32;
    uint16* write_buf = new uint16[write_buf_size];
    int last_index = 0;
    for (int i = 0, d = 0; i < write_buf_size; i += 32, d += 63) {

        write_buf[i] = WR | 0x003F; 
        memcpy((uint8*)(write_buf + i) + 1, send_data + d, 63);

        last_index = i;
    }

    uint16 rem = send_size % 63;

    if (rem != 0) write_buf[last_index] = (write_buf[last_index] & 0xFF00) | WR | rem;

    DWORD st = timeGetTime();

    DWORD bytes_written;
    FT_Write(ftHandle, write_buf, bytes_to_write, &bytes_written);

    DWORD et = timeGetTime();
    int u = et - st;
    double s = u / 1000.0;
    printf("send sum  0x%02X\n", check_sum(send_data, send_size));
    printf("send %dms %0.1f kB/s\n\n", u, send_size / s / 1024.0);

    delete[] write_buf;

    DeviceClose(ftHandle);

    return 0;
}

int BlasterRecv(FT_HANDLE ftHandle, uint8* recv_data, const int recv_size, bool f_SLD_trans)
{
    if (recv_data == NULL) return -1;
    if (recv_size <= 0) return -1;

    if (f_SLD_trans) {
        MoveIdle(ftHandle);
        MoveIdleToShiftir(ftHandle);
        WriteShiftir(ftHandle, 0x0E); // USER1
        MoveShiftirToShiftdr(ftHandle);
        WriteShiftdr(ftHandle, 0x42); // for counter reset_FPGA send
        MoveShiftdrToShiftir(ftHandle);
        WriteShiftir(ftHandle, 0x0C); // USER0
        MoveShiftirToShiftdr(ftHandle);
    }
    DWORD bytes_to_write = ((recv_size - 1) / 63) + recv_size + 1;

    int write_buf_size = (bytes_to_write + 1) / 2;
    uint16* write_buf = new uint16[write_buf_size];
    int last_index = 0;
    memset(write_buf, 0x00, write_buf_size * 2);
    for (int i = 0; i < write_buf_size; i += 32) {
        write_buf[i] = RD | 0x003F;
        last_index = i;
    }
    uint16 rem = recv_size % 63;
    if (rem != 0) write_buf[last_index] = RD | rem;

    DWORD st = timeGetTime();

    DWORD bytes_written;
    FT_Write(ftHandle, write_buf, bytes_to_write, &bytes_written);

    DWORD bytes_read;
    FT_Read(ftHandle, recv_data, recv_size, &bytes_read);

    DWORD et = timeGetTime();
    int u = et - st;
    double s = u / 1000.0;
    if (f_SLD_trans) {
        printf("recv sum 0x%02X\n", check_sum(recv_data, recv_size));
        printf("recv %dms %0.1f kB/s\n", u, recv_size / s / 1024.0);
    }
    delete[] write_buf;

    if (f_SLD_trans) {
        DeviceClose(ftHandle);
    }
    return 0;
}

#define RECV_MAX (4000 * 63) 
int BlasterRecv(FT_HANDLE ftHandle, uint8* recv_data, const int recv_size)
{
    if (recv_data == NULL) return -1;
    if (recv_size <= 0) return -1;

    MoveIdle(ftHandle);
    MoveIdleToShiftir(ftHandle);
    WriteShiftir(ftHandle, 0x0E); // USER1
    MoveShiftirToShiftdr(ftHandle);
    WriteShiftdr(ftHandle, 0x42); // for counter reset_FPGA send
    MoveShiftdrToShiftir(ftHandle);
    WriteShiftir(ftHandle, 0x0C); // USER0
    MoveShiftirToShiftdr(ftHandle);

    DWORD st = timeGetTime();

    int count = recv_size / RECV_MAX;
    for (int i = 0; i < count; i++) {
        BlasterRecv(ftHandle, recv_data + RECV_MAX * i, RECV_MAX, false);
    }
    BlasterRecv(ftHandle, recv_data + RECV_MAX * count, recv_size % RECV_MAX, false);

    DWORD et = timeGetTime();
    int u = et - st;
    double s = u / 1000.0;
    printf("recv sum 0x%02X\n", check_sum(recv_data, recv_size));
    printf("recv %dms %0.1f kB/s\n", u, recv_size / s / 1024.0);

    DeviceClose(ftHandle);

    return 0;
}

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("ROS2_subscriber"), ftHandle(nullptr) {
        
        if (ftHandle != nullptr) {
            FT_Close(ftHandle);
        }
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "usb_data", 10,
            std::bind(&MySubscriber::topic_callback, this, _1));
    }

private:
    FT_HANDLE ftHandle;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        
        // init. USB 

        FT_STATUS ftStatus;
        ftStatus = FT_OpenEx((PVOID)"USB-Blaster", FT_OPEN_BY_DESCRIPTION, &ftHandle);
        if (ftStatus != FT_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed connection");
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received checksum Data");
            const int size = 32768; // data size range
            uint8* recv_data = new uint8[size];
            BlasterRecv(ftHandle, recv_data, size);
            delete[] recv_data;
            RCLCPP_INFO(this->get_logger(), "Received '%s' :Transmission completed!", msg->data.c_str());
        }

    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher() : Node("ROS2_publisher"), ftHandle(nullptr) {
        // init. publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("usb_data", 10);

        while (publisher_->get_subscription_count() < 1) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        timer_callback();

        //sending D2XX
        sendUSBData();
        if (ftHandle != nullptr) {
            FT_Close(ftHandle);
        }
    }

private:
    FT_HANDLE ftHandle;

    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "USB-Blaster";
        RCLCPP_INFO(this->get_logger(), "Connecting :'%s'", message.data.c_str());
        publisher_->publish(message);
    }
    
    void sendUSBData() {
        // send USB checksum Data
        FT_STATUS ftStatus;
        ftStatus = FT_OpenEx((PVOID)"USB-Blaster", FT_OPEN_BY_DESCRIPTION, &ftHandle);
        if (ftStatus != FT_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open USB port");
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully connected : Send checksum Data");
        }
        const int size = 32768; //data size range
        srand((uint32)time(NULL));
        uint8* send_data = new uint8[size];
        for (int i = 0; i < size; i++) {
            send_data[i] = rand();
        }
        BlasterSend(ftHandle, send_data, size);
        delete[] send_data;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto subscriber_node = std::make_shared<MySubscriber>();
    auto publisher_node = std::make_shared<MyPublisher>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
