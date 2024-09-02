#ifndef _MOTION_CONTROL_H
#define _MOTION_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include "sensor_brainco/stm32data.h" //sensor_brainco  uart_stm32
#include "sensor_brainco/brainco_.h"//msg
#include <thread>
//     <node pkg="sensor_brainco" type="motion_control" name="motion_control" output="screen" />

// 接收到订阅的消息后，会进入消息回调函数
void sensorInfoCallback(const sensor_brainco::stm32data::ConstPtr& msg);
// 接收到订阅的消息后，会进入消息回调函数
void brainco_fb_InfoCallback(const sensor_brainco::brainco_::ConstPtr& msg);
//强脑手控制函数
void dirctrl_brainco(uint8_t mode,int8_t * value);

void brainco_Ctrl_pub(ros::Publisher pub,ros::Rate rosrate);//向brainco请求反馈数据,然后发布反馈数据fb

void main_proj();



#endif