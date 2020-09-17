// ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <fstream> 

#include "serial/serial.h"
#include <math.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

std::string mapdata_path = "/home/rock/catkin_ws/src/snow_rtk/data/boundary/boundary.txt";

//////////////////////////////////////////////////////data_type.h////////////////////////////////////
/* exact-width integer types */
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;

/* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;

#define MSG_BUF_SIZE   2048
#define ESP32_BUF_SIZE 50

bool esp32_link = false;

typedef struct 
{
	uint8_t head;
	uint8_t product_type;
	uint8_t cmd;
	uint8_t data_type_H;
	uint8_t data_type_L;
	uint8_t data_lens_H;
	uint8_t data_lens_L;
	uint8_t data[12];
    uint8_t crc;
    uint8_t end;
}esp32_type;

esp32_type esp32;

typedef  struct 
{               
	uint8_t       *Start;                   
	uint8_t       *QEnd; 
	uint8_t       *OSQIn;
	uint8_t       *OSQOut; 
	uint16_t       OSQSize;  
	uint16_t     OSQEntries;
}buf_type;

struct data
{
    std::string longitude;//经度
    std::string latitude;//纬度
    // std::string id;
    int id;
}mapdata[355];

uint8_t data_buf[200]; //139
serial::Serial ser;

void mapdata_read()
{
    int n=0;
    string tempstr;
    std::ifstream map_in(mapdata_path.c_str(),std::ios::in);
    if (!map_in.is_open())
    {
        cout << "Error: opening file fail" << endl;
        return;
    }
    while (!map_in.eof()&&n<355)
    {
        getline(map_in,tempstr);
        
        mapdata[n].longitude = tempstr.substr(59,12);
        mapdata[n].latitude = tempstr.substr(79,12);
        mapdata[n].id = std::stoi(tempstr.substr(121,3));
        n++;
    }

    // for (int i = 0; i < n; i++)
    // {
    //     std::cout<<"longitude: "<<mapdata[i].longitude[0]<<" latitude: "<<mapdata[i].latitude<<" id: "<<mapdata[i].id<<std::endl;
    // }
    map_in.close();
}

void mapdata_send()
{
    int index = 0;
    data_buf[0] = 0xAA;
    data_buf[1] = 0xAA;
    data_buf[2] = 0xAA;
    data_buf[3] = 0x2A; //'*'帧起始符
    data_buf[4] = 0x20; // 产品类型1bytes
    data_buf[5] = 0x00; //控制码1bytes
    data_buf[6] = 0x02; //6和7数据类型
    data_buf[7] = 0x02;
    data_buf[8] = 0x00; //8和9数据长度
    data_buf[10] = 0x10;//10和11输入指令
    data_buf[11] = 0x08;
    data_buf[13] = 0x10;//13和14输出指令
    data_buf[14] = 0x58;

    if(index == 0)
    {
        data_buf[9] = 0x07;//数据长度为7
        data_buf[12] = 0x53;//S起始包
        data_buf[15] = 0x00;
        data_buf[16] = 0x00;
        uint8_t check_sum = 0;
        for(uint8_t i = 3; i<17;i++)
        {
            check_sum = check_sum + data_buf[i];
        }

        data_buf[17] = check_sum;
        data_buf[18] = 0x23;

        index++;
        ser.write(data_buf,19);
    }

    if(index>0&&index<72)
    {
        data_buf[9] = 0x7F;//数据长度127
        data_buf[16] = 0x78;//包长度

        for(int i = 0;i<71;i++)
        {
            data_buf[12] = uint8_t(i+1);//顺序号
            // data_buf[12] = ((i+1)&0xff);//顺序号
            data_buf[15] = uint8_t(i+1);//包序号
            for (uint8_t j = 0; j < 12; j++)
            {
                data_buf[17+j]=mapdata[i*5].longitude.at(j);//经度
                data_buf[29+j]=mapdata[i*5].latitude.at(j);//纬度
                data_buf[41+j]=mapdata[i*5+1].longitude.at(j);
                data_buf[53+j]=mapdata[i*5+1].latitude.at(j);
                data_buf[65+j]=mapdata[i*5+2].longitude.at(j);
                data_buf[77+j]=mapdata[i*5+2].latitude.at(j);
                data_buf[89+j]=mapdata[i*5+3].longitude.at(j);
                data_buf[101+j]=mapdata[i*5+3].latitude.at(j);
                data_buf[113+j]=mapdata[i*5+4].longitude.at(j);
                data_buf[125+j]=mapdata[i*5+4].latitude.at(j);//136
           }
           //CRC
           uint8_t check_sum = 0;
           for (uint8_t j = 3; j < 137; j++)
           {
               check_sum = check_sum + data_buf[j];
           }

           data_buf[137] = check_sum;//CRC
           data_buf[138] = 0x23;//结束符
           index++;
           ser.write(data_buf,139);
           usleep(2000);
        }   
    }
    if(index == 72)
    {
        data_buf[9] = 0x07;
        data_buf[12] = 0x45;//结束包‘E’
        data_buf[15] = 0x00;//包序号
        data_buf[16] = 0x00;//包长度
        uint8_t check_sum = 0;
        for (uint8_t i = 3; i < 17; i++)
        {
            check_sum = check_sum+data_buf[i]; 
        }
        data_buf[17] = check_sum;
        data_buf[18] = 0x23;
        
        //cout<<index<<endl;
        index = 0;
        //for(uint8_t j =0;j<19;j++)
        //{
             //cout<<data_buf[j]<<endl;
        //     printf("0x%.2x\t",data_buf[j]);
        //}
        ser.write(data_buf,19);
    }
}

typedef enum { EMPTY=3,OK=1,FULL=2}buf;

uint8_t      esp32_buf[MSG_BUF_SIZE];     
buf_type     esp32_buf_contr_op;  

void  buf_init(buf_type * _os,uint8_t  *_Event,uint16_t buf_size)  //
{    
      uint16_t i;
      for(i=0;i<buf_size;i++)
      {
	      _Event[i]=0;    
      }
     	
      _os->Start=  _Event;                       
      _os->QEnd=   _Event+buf_size;      
      _os->OSQIn=  _Event;            
      _os->OSQOut= _Event;            
      _os->OSQSize= buf_size;          
      _os->OSQEntries=0;              
}


static uint8_t  buf_rxnum(buf_type *p)
{
	if (p->OSQOut > p->OSQIn){ 
		return   (p->QEnd - p->OSQOut) + (p->OSQIn - p->Start) ;
	} 
	if (p->OSQOut < p->OSQIn){
  		return  p->OSQIn - p->OSQOut ;
	}
	return 0 ; 	 
}

uint8_t   buf_read(buf_type * _os,uint8_t *p) 
{     
	if ( buf_rxnum(_os) > 0){  //��D��y?Y?��D��?��
		*p = *((_os->OSQOut)++); 
		if (_os->OSQOut==_os->QEnd){ 
			_os->OSQOut = _os->Start ; 
			
		}
		return OK;
	}
	return EMPTY;
}

void  buf_write(buf_type * _os,uint8_t *p)
{  
	if (_os->OSQIn ==_os->QEnd)  
		_os->OSQIn = _os->Start;
	*((_os->OSQIn)++)= *p ;
	if(_os->OSQIn==_os->QEnd) 
		_os->OSQIn=_os->Start; 
}

bool CRC_Check(uint8_t *p, uint16_t len)
{
	uint8_t temp = 0;
	for(uint16_t i = 1; i < len; i++)
	{
		temp += p[i];
	}

	temp = temp ;
	uint8_t rcc = p[len] ;

	// ROS_INFO("temp = %d, rcc= %d",temp,rcc);

	if(rcc == temp)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static uint8_t buff[ESP32_BUF_SIZE+1] = {EMPTY};
uint8_t K;
uint8_t * read_esp32_buf(void)
{
	uint16_t i;
	uint8_t data,temp;
	static int index = 0;
	static int count = 0;
	 
    if ((*buff) == EMPTY )
	{  
  	 if (*(buff+1) != '*')
		{
			for (i=0;i<ESP32_BUF_SIZE;i++)
			{  
				temp = buf_read(& esp32_buf_contr_op,&data);
				if (temp == EMPTY ) return  buff;  
				  
				if(data =='*') 
				{
					*(buff+1) = data;
					count=0;
					index=ESP32_BUF_SIZE-1;
					break;
				}
				
			}
		}	

		for (;index>0;)
		{


			K=temp= buf_read(& esp32_buf_contr_op,&data);

			if(temp == EMPTY ) return  buff; 
			*(buff + count+2) = data;
			count++;
			index--;
			if(index == 0|| data=='#' )
			{
				// 解析buff长度
				 
				if(CRC_Check(buff, count))
				{ 
					*buff = FULL;
					return  buff; 
				} 
				for(i = 1; i <=ESP32_BUF_SIZE; i++)  buff[i] = 0;
				buff[0] = EMPTY;    
				count = 0;
				index = 0;
				return  buff; 
			}
		}
  } 
}

void esp32_analyse(void)
{
    uint8_t *buf;
    uint8_t fcode;
    buf=read_esp32_buf();
    if(*buf==FULL)
	{
	    if (*(buf+2) == 0xFF)
		{
	        esp32_type * frame = (esp32_type*)(buf+1);
	        fcode= frame->data[0];
            ROS_INFO("fcode %d", fcode);
    
            if(fcode == 0x02)
            {
                esp32_link = true;
                std::cout<<"ESP32 link the server success!"<<std::endl;
            }

            else
            {
                esp32_link = false;
                std::cout<<"ESP32 link the server failed!"<<std::endl;
            }

			for(uint16_t i=1;i<ESP32_BUF_SIZE;i++)
            {
                *(buf+i)=0;
            }  
			*buf=EMPTY;
		}
        else
		{
			for(uint16_t i=1;i<ESP32_BUF_SIZE;i++)  *(buf+i)=0;
			*buf=EMPTY;			
		} 
    }
}

int main(int argc, char *argv[])
{
    buf_init((buf_type *)&esp32_buf_contr_op ,(uint8_t  *)esp32_buf,MSG_BUF_SIZE);
    // 打开ESP32串口通讯
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(921600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    // 检测串口是否已经打开
    if (ser.isOpen())
    {
        ROS_INFO("Serial Port initialized.");
    }
    else
    {
        return -1;
    }

    string result;
	uint8_t  data;

    while(true)
    {
        result = ser.read(1);
        if (result.length())
        {
            data = result.at(0);
			buf_write(&esp32_buf_contr_op,&data);
        }

        esp32_analyse(); 

        if(esp32_link)
        {
            mapdata_read();
            mapdata_send();

            sleep(1);
        }
    }

    return 0;
}

