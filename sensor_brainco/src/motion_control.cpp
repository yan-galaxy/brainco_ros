#include "motion_control.h"

sensor_brainco::brainco_ brainco_Ctrl_msg;
sensor_brainco::brainco_ brainco_FB_msg;
sensor_brainco::stm32data recv_stm32data;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "motion_control");

    ros::NodeHandle n1,n2,n3;

    ros::Subscriber sensor_info_sub = n1.subscribe("/stm32data_info", 128, sensorInfoCallback);//订阅传感器信息
    ros::Subscriber brainco_fb_info_sub = n2.subscribe("/brainco/brainco_FB_info", 38, brainco_fb_InfoCallback);//订阅brainco信息

    ros::Publisher brainco_Ctrl_info_pub = n3.advertise<sensor_brainco::brainco_>("/brainco/brainco_Ctrl_info", 38);//发布brainco ctrl话题控制
    ros::Rate loop_rate_pub(1000);
    std::thread ros_brainco_Ctrl_pub_thread(brainco_Ctrl_pub,brainco_Ctrl_info_pub,loop_rate_pub);//发布brainco控制信息  

    std::thread main_proj_thread(main_proj);

    ros::spin();

}
// 接收到订阅的消息后，会进入消息回调函数
void sensorInfoCallback(const sensor_brainco::stm32data::ConstPtr& msg)
{
    memcpy(&recv_stm32data,msg.get(),128);
    
    // printf("Subcribe sensor Info:\n");
    // printf("voltage:\n");
    // for(uint8_t i=0;i<14;i++)
    // {
    //     printf("%4d ",recv_stm32data.voltage[i]);
    // }
    // printf("\n");
    // printf("diff:\n");
    // for(uint8_t i=0;i<14;i++)
    // {
    //     printf("%4d ",recv_stm32data.diff[i]);
    // }
    // printf("\n");
    // printf("initial value:\n");
    // for(uint8_t i=0;i<14;i++)
    // {
    //     printf("%.2f ",recv_stm32data.initial_value[i]);
    // }
    // printf("\n");

}
// 接收到订阅的消息后，会进入消息回调函数
void brainco_fb_InfoCallback(const sensor_brainco::brainco_::ConstPtr& msg)
{
    memcpy(&brainco_FB_msg,msg.get(),38);
    // printf("Subcribe brainco_ Info:\n");
    // printf("choose:%d\n",brainco_FB_msg.which_choose);
    // printf("shutdown:%d\n",brainco_FB_msg.shutdown);
    // printf("pos_fb:\n");
    // for(uint8_t i=0;i<6;i++)
    // {
    //     printf("%4d ",brainco_FB_msg.pos_fb[i]);
    // }
    // printf("\n");
    // printf("speed_fb:\n");
    // for(uint8_t i=0;i<6;i++)
    // {
    //     printf("%4d ",brainco_FB_msg.speed_fb[i]);
    // }
    // printf("\n");
    // printf("current_fb:\n");
    // for(uint8_t i=0;i<6;i++)
    // {
    //     printf("%4d ",brainco_FB_msg.current_fb[i]);
    // }
    // printf("\n");

}
//强脑手控制函数
void dirctrl_brainco(uint8_t mode,int8_t * value)
{   
    brainco_Ctrl_msg.which_choose=mode;
    if(mode==sensor_brainco::brainco_::choose_pos)
    {
        for(uint8_t i=0;i<6;i++)
        {
            brainco_Ctrl_msg.pos_ctrl[i] = value[i];
        }
    }
    else if(mode==sensor_brainco::brainco_::choose_speed)
    {
        for(uint8_t i=0;i<6;i++)
        {
            brainco_Ctrl_msg.speed_ctrl[i] = value[i];
        }
    }
    else if(mode==sensor_brainco::brainco_::choose_current)
    {
        for(uint8_t i=0;i<6;i++)
        {
            brainco_Ctrl_msg.current_ctrl[i] = value[i];
        }
    }
    
}
void brainco_Ctrl_pub(ros::Publisher pub,ros::Rate rosrate)//向brainco请求反馈数据,然后发布反馈数据fb
{
    while(ros::ok())//keepRunning
    {
        // 发布消息
		pub.publish(brainco_Ctrl_msg);
        // 按照循环频率延时
        rosrate.sleep();

        
    }
}
void main_proj()//向brainco请求反馈数据,然后发布反馈数据fb
{
    uint8_t ctrl_flag=1;
    usleep(1000*600);
    int8_t pos[6]={0,100,0,0,0,0};
    int8_t speed[6]={0,0,0,0,0,0};
    int8_t current[6]={0,0,0,0,0,0};
    int8_t run_stat=0;

    int8_t start_pos[3]={0};//刚接触时的位置
    int8_t active_point=-1;
    int16_t press[5]={0};//刚接触时的压力
    int8_t thumb_value=25;//大拇指
    int8_t index_value=35;//食指

    
    sleep(2);
    pos[0]=47;pos[1]=93;pos[2]=57;//食指和大拇指合拢抓取物体
    // pos[0]=0;pos[1]=93;pos[2]=0;
    dirctrl_brainco(1,pos);
    while(1);

    // dirctrl_brainco(1,pos);
    // dirctrl_brainco(2,speed);
    // dirctrl_brainco(3,current);
    // ros::shutdown();
    while(ros::ok())//keepRunning
    { 
        // printf("\nrun_stat:%d\n\n",run_stat);
        switch(run_stat)
        {
            case 0:
                speed[0]=15;speed[2]=20;//先快速合拢
                dirctrl_brainco(2,speed);

                run_stat=1;
                break;

            case 1:
                // printf("\nbrainco_FB_msg.pos_fb[0]:%d\n",brainco_FB_msg.pos_fb[0]);
                // printf("\nbrainco_FB_msg.pos_fb[2]:%d\n",brainco_FB_msg.pos_fb[2]);
                // printf("\nrecv_stm32data.diff[2]:%d\n",recv_stm32data.diff[2]);
                if(brainco_FB_msg.pos_fb[0]>thumb_value)
                {
                    speed[0]=7;
                    dirctrl_brainco(2,speed);
                }
                if(brainco_FB_msg.pos_fb[2]>index_value)
                {
                    speed[2]=7;
                    dirctrl_brainco(2,speed);
                }
                if(brainco_FB_msg.pos_fb[0]>thumb_value && brainco_FB_msg.pos_fb[2]>index_value 
                || recv_stm32data.diff[2]>50 || recv_stm32data.diff[3]>50 || recv_stm32data.diff[1]>50)// && stm32data_msg.diff[15]<50  && stm32data_msg.diff[14]<50
                {
                    speed[0]=7;speed[2]=7;//靠近后速度放慢
                    dirctrl_brainco(2,speed);

                    // start_vol=recv_stm32data.voltage[2];//记录初始电压
                    run_stat=2;//正常感知
                    printf("\nbrainco_FB_msg.pos_fb[0]:%d,pos_fb[2]:%d\n",brainco_FB_msg.pos_fb[0],brainco_FB_msg.pos_fb[2]);
                    // run_stat=20;//测试电流
                }
                break;

            case 2:
                if(recv_stm32data.diff[2]>50 || recv_stm32data.diff[3]>50 || recv_stm32data.diff[1]>50)
                {
                    start_pos[0]=brainco_FB_msg.pos_fb[0];
                    start_pos[2]=brainco_FB_msg.pos_fb[2];

                    
                    pos[0]=start_pos[0];pos[2]=start_pos[2];
                    dirctrl_brainco(1,pos);

                    usleep(1000*300);//等待300ms 稳定
                    
                    if(recv_stm32data.diff[3]>50)     active_point=3;
                    else if(recv_stm32data.diff[2]>50)active_point=2;
                    else if(recv_stm32data.diff[1]>50)active_point=1;
                    else active_point=1;
                    
                    press[0]=recv_stm32data.diff[active_point];
                    
                    printf("\n active_point:%d,press[0]:%4d ,pos[0]:%3d,pos[2]:%3d\n\n",active_point,press[0],pos[0],pos[2]);

                    run_stat=3;
                }
                break;

            case 3:
                //依次控制位移 测量压力

                for(uint8_t i=1;i<5;i++)
                {
                    pos[0]=start_pos[0]+i;pos[2]=start_pos[2]+i;
                    dirctrl_brainco(1,pos);

                    usleep(1000*100);//等待100ms
                    double average_value = 0;

                    for(uint16_t cnt=0;cnt<400;cnt++)//400ms内数据取平均
                    {
                        average_value+=(double)recv_stm32data.diff[active_point]/400.0;
                        usleep(1000);
                    }
                    press[i]=(uint16_t)average_value;
                    printf("\npress[%d]:%4d ,pos[0]:%3d,pos[2]:%3d\n\n",i,press[i],pos[0],pos[2]);
                }

                pos[0]=start_pos[0]+2;pos[2]=start_pos[2]+2;
                dirctrl_brainco(1,pos);//退回刚接触时的位置  +2

                usleep(1000*500);//等待500ms

                //半透明塑料杯 press[4]：150~300
                //椰汁       press[4]: 400~800
                run_stat=4;
                break;

            case 4:
                current[0]=(int8_t)((float)press[4]/30.0);current[2]=(int8_t)((float)press[4]/30.0);//5 抓半透明塑料杯   20抓椰汁
                printf("\nfinal   %f  %d\n\n",(float)press[4]/30.0,(int8_t)((float)press[4]/30.0));
                dirctrl_brainco(3,current);
                run_stat=5;
                break;

            case 5:
                sleep(10);
                pos[0]=0;pos[2]=0;
                dirctrl_brainco(1,pos);//松开
                sleep(10);
                for(size_t m=0;m<16;m++)
                recv_stm32data.diff[m]=0;
                // run_stat=0;//重新抓下一个
                break;



            case 10:
                run_stat=11;
                break;

            

            case 20:
                if(recv_stm32data.diff[15]>50)
                {
                    current[0]=5;current[2]=5;//5 抓半透明塑料杯   20抓椰汁
                    dirctrl_brainco(3,current);
                    run_stat=21;
                }
                break;


            default:
                break;
                
        }
        usleep(100*1);
    }

}