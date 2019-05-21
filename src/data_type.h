#ifndef _DATA_TYPE_H_
#define _DATA_TYPE_H_
#pragma pack(1)
enum tcp_data_type_t {
	RESERVED_DATA_TYPE = 0x0,
	SLAM_DATA_TYPE = 0x01,
	UP_DATA_TYPE,
	MAP_DATA_TYPE,
	IMG_DATA_TYPE,
	CMD_DATA_TYPE,
	LG_LIDAR_DATA_TYPE,
	CONFIG_FILE_DATA_TYPE,
	MAX_DATA_TYPE  
};

struct tcp_package_head {
    char flag[4]; 				//0x53 0x54 0x55 0x55
    int id; 					//包序号
    char type;		// pyload 数据类型
    unsigned int payload_len;					//payload 数据长度
    unsigned int payload_flag;
    short crc;  				// payload crc
};
typedef struct lg_lidar_data_t {
	char buff[42*60];
	long long timestamp;
}LG_LIDAR_DATA;

struct tcp_data_t {
    struct tcp_package_head head;
    void *payload;				//payload 数据指针
};
#pragma pack()

struct _img_feature_t{
	float x;
	float y;
};

// picture
typedef struct _img_tran_ {
	long long timestamp;
	char img_data[640*480];
	unsigned short tracking_features;
	unsigned short total_features;
	struct _img_feature_t img_feature[2000];
}_IMG_TRAN_DATA_T;


// map data
typedef struct _map_tran_
{
    long long timestamp;
    char map_data[600*600];
}_MAP_TRAN_DATA_T;

// slam data
typedef struct _slam_data_ {
	int State;		
	float X;
	float Y;
	float Yaw;
	long long Timestamp;
	int Features;
	unsigned char confidence;
}SLAM_DATA_T; 

// row underpan data
typedef struct _up_data_ {
    int RobotInfo;
    float LeftWheel;		// the increment (m)
    float RightWheel;
    float X;
    float Y;
    short RobotSensorinfo[6]; 	//ACC x y z gro
    float Yaw;			// right-hand coordinates (rad)
    long long Timestamp;
    unsigned char remap_index; //地盘坐标系转slam坐标系的remap下标 remap 见 updata_to_slam_remap_data
    unsigned char wholeInfo[144];
}up_data_t;

#endif
