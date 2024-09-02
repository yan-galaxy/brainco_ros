#include <ros/ros.h>
#include "softCRC.h"
#include "serial_self.h"
#include <stdio.h>
#include <thread>
#include <queue>
#include <mutex>

// std::queue<char> Queue;
std::mutex serial_mtx;//互斥锁
POS_union pos_u;
SPEED_union speed_u;
CURRENT_union current_u;

int send_pos_ctrl(int serial_fd,POS_union* pos_u,
uint8_t pos0,uint8_t pos1,uint8_t pos2,uint8_t pos3,uint8_t pos4,uint8_t pos5);
int send_speed_ctrl(int serial_fd,SPEED_union* speed_u,
uint8_t speed0,uint8_t speed1,uint8_t speed2,uint8_t speed3,uint8_t speed4,uint8_t speed5);
int send_current_ctrl(int serial_fd,CURRENT_union* current_u,
uint8_t current0,uint8_t current1,uint8_t current2,uint8_t current3,uint8_t current4,uint8_t current5);
int data_request(int serial_fd,uint8_t* pos,uint8_t* speed,uint8_t* current);

int main(int argc, char **argv) 
{
    int serial_fd=0;
    int rx_len=0;
    int tx_len=0;
    
    char port_str[]="/dev/ttyUSB0"; 
    serial_fd=init_serial(port_str,B115200);

    ros::init(argc, argv, "brainco_server");
    ros::NodeHandle n;

    // while(1)
    {
        tx_len=send_pos_ctrl(serial_fd,&pos_u,0,100,0,0,0,0);
        // tx_len=send_speed_ctrl(serial_fd,&speed_u,0,0,-10,-10,-20,-10);
        // tx_len=send_current_ctrl(serial_fd,&current_u,0,0,-30,20,20,10);

        uint8_t pos[6]={0};
        uint8_t speed[6]={0};
        uint8_t current[6]={0};
        
        rx_len=data_request(serial_fd,pos,speed,current);

        printf("pos    :%3d %3d %3d %3d %3d %3d\n",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
        printf("speed  :%3d %3d %3d %3d %3d %3d\n",(int8_t)speed[0],(int8_t)speed[1],(int8_t)speed[2],(int8_t)speed[3],(int8_t)speed[4],(int8_t)speed[5]);
        printf("current:%3d %3d %3d %3d %3d %3d\n",(int8_t)current[0],(int8_t)current[1],(int8_t)current[2],(int8_t)current[3],(int8_t)current[4],(int8_t)current[5]);

        usleep(1000*500);
        
    }
    close(serial_fd); 
    return 0; 
}

int send_pos_ctrl(int serial_fd,POS_union* pos_u,
uint8_t pos0,uint8_t pos1,uint8_t pos2,uint8_t pos3,uint8_t pos4,uint8_t pos5)
{
    int tx_len=0;
    uint32_t crc32=0;

    pos_u->value[0]=pos0;
    pos_u->value[1]=pos1;
    pos_u->value[2]=pos2;
    pos_u->value[3]=pos3;
    pos_u->value[4]=pos4;
    pos_u->value[5]=pos5;

    crc32 = softCRC_CRC32(pos_u, (sizeof(*pos_u) - 4) / 4, 0xffffffff, 0);

    pos_u->crc8[0] = crc32 >> 24 & 0xFF;
    pos_u->crc8[1] = crc32 >> 16 & 0xFF;
    pos_u->crc8[2] = crc32 >> 8  & 0xFF;
    pos_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("tx_crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)pos_u, sizeof(*pos_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}
int send_speed_ctrl(int serial_fd,SPEED_union* speed_u,
uint8_t speed0,uint8_t speed1,uint8_t speed2,uint8_t speed3,uint8_t speed4,uint8_t speed5)
{
    int tx_len=0;
    uint32_t crc32=0;

    speed_u->value[0]=speed0;
    speed_u->value[1]=speed1;
    speed_u->value[2]=speed2;
    speed_u->value[3]=speed3;
    speed_u->value[4]=speed4;
    speed_u->value[5]=speed5;

    crc32 = softCRC_CRC32(speed_u, (sizeof(*speed_u) - 4) / 4, 0xffffffff, 0);

    speed_u->crc8[0] = crc32 >> 24 & 0xFF;
    speed_u->crc8[1] = crc32 >> 16 & 0xFF;
    speed_u->crc8[2] = crc32 >> 8  & 0xFF;
    speed_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)speed_u, sizeof(*speed_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}
int send_current_ctrl(int serial_fd,CURRENT_union* current_u,
uint8_t current0,uint8_t current1,uint8_t current2,uint8_t current3,uint8_t current4,uint8_t current5)
{
    int tx_len=0;
    uint32_t crc32=0;

    current_u->value[0]=current0;
    current_u->value[1]=current1;
    current_u->value[2]=current2;
    current_u->value[3]=current3;
    current_u->value[4]=current4;
    current_u->value[5]=current5;

    crc32 = softCRC_CRC32(current_u, (sizeof(*current_u) - 4) / 4, 0xffffffff, 0);

    current_u->crc8[0] = crc32 >> 24 & 0xFF;
    current_u->crc8[1] = crc32 >> 16 & 0xFF;
    current_u->crc8[2] = crc32 >> 8  & 0xFF;
    current_u->crc8[3] = crc32 & 0xFF;

    // printf("crc32:%08X\n",crc32);
    // printf("crc8:%02X%02X%02X%02X\n",pos_u->crc8[0],pos_u->crc8[1],pos_u->crc8[2],pos_u->crc8[3]);

    serial_mtx.lock();
    tx_len=uart_send(serial_fd, (char *)current_u, sizeof(*current_u));//pos_ctrl
    serial_mtx.unlock();
    // usleep(sizeof(*pos_u)*100);
    return tx_len;
}
int data_request(int serial_fd,uint8_t* pos,uint8_t* speed,uint8_t* current)
{
    int tx_len=0;
    uint8_t rx_buf[512]={0}; 
    int rx_len=0;
    uint8_t request[16]={0x42,0x6E,0x43,0x50,0x02,0x01,0x00,0x04,0x4A,0x02,0x08,0x01,0xF2,0x7B,0xBD,0xE5};
    uint32_t calcu_crc32=0,recv_crc32=0;

    
    serial_mtx.lock();
    
    tx_len=uart_send(serial_fd, (char *)request, sizeof(request));//send data request
    usleep(sizeof(request)*300);
    usleep(88*160);
    memset(rx_buf,0,sizeof(rx_buf)); 
    rx_len=uart_recv(serial_fd, (char*)rx_buf, sizeof(rx_buf));
    serial_mtx.unlock();
    printf("1234\n");


    calcu_crc32 = softCRC_CRC32(rx_buf, (rx_len - 4) / 4, 0xffffffff, 0);
    // printf("calcu_crc32:%08X\n",calcu_crc32);
    recv_crc32 = (rx_buf[rx_len - 4] << 24) + (rx_buf[rx_len - 3] << 16) + (rx_buf[rx_len - 2] << 8) + rx_buf[rx_len - 1];
    // printf("recv_crc32:%08X\n",recv_crc32);

    if(calcu_crc32!=recv_crc32 && rx_len==88)
    {
        perror("CRC wrong!\n");
        return -1;
    }

    // for(int i=0;i<rx_len;i++)
    // {
    //     // Queue.push(rx_buf[i]);
    //     if(i<(rx_len-1))
    //         printf("0x%02X,",rx_buf[i]);
    //     else 
    //         printf("0x%02X\n",rx_buf[i]);
    // }

    memcpy(pos,&rx_buf[14],6);// 0x0A 0x06
    memcpy(speed,&rx_buf[22],6);// 0x12 0x06
    memcpy(current,&rx_buf[30],6);// 0x1A 0x06

    return rx_len;
}










// uint8_t recv[88]=
// {
// 0x42,0x6E,0x43,0x50,0x01,0x0A,0x00,0x4B,
// 0x12,0x49,0x0A,0x20,0x0A,0x06,0x00,0x63,
// 0x00,0x00,0x00,0x00,0x12,0x06,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x1A,0x06,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x22,0x06,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x10,0x6B,0x18,0xFE,//0x6A,0x18 0xFFFFFFFE
// 0x7F,0x22,0x20,0x0A,0x06,0x00,0x64,0x00,
// 0x00,0x00,0x00,0x12,0x06,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x1A,0x06,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x22,0x06,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0xE6,0x42,0xEA,0x5A //0xFFFFFFE6,0x42,0xFFFFFFEA,0x5A 
// };

// crc32 = softCRC_CRC32(recv, (sizeof(recv) - 4) / 4, 0xffffffff, 0);
// printf("crc32:%08X\n",crc32);
// exit(400);