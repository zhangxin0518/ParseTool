#include <iostream>
#include <opencv2/opencv.hpp>
#include "data_type.h"
#include "Alloc.h"
#include "LzmaDec.h"
#include "unistd.h"

#define DS_DIR "../data_sheet"
#define DATA_DIR "../data"
#define MAP_IMG_DIR "../data/map_img"

#define ODO_DATA_DS  "../data_sheet/odometry.ds"
#define IMG_DATA_DS  "../data_sheet/img.ds"
#define IMG_DATA_DS_NUM  "../data_sheet/img_%d.ds"
#define CONFIG_FILE_PATH  "../data_sheet/config_tco_production.yaml"
#define LG_LIDAR_DS  "../data_sheet/lg_lidar.ds"
#define LG_LIDAR_TS_CSV  "../data_sheet/lg_lidar_ts.csv"
#define MAP_DATA_DIR "../data/map_img/"

using namespace cv;
using namespace std;

static FILE *pTestFile;

int LzmaUncompress(unsigned char *dest, size_t *destLen, const unsigned char *src, size_t *srcLen,
		   const unsigned char *props, size_t propsSize)
{
	ELzmaStatus status;
	return LzmaDecode(dest, destLen, src, srcLen, props, (unsigned)propsSize, LZMA_FINISH_ANY, &status, &g_Alloc);
}

static unsigned char unCompress[600 * 1024];

int getOnePkg(char *data, int len, char type, int idx, int headFlag)
{
	static int cur_index[MAX_DATA_TYPE];
	if (cur_index[type] != idx) {
		printf("丢包 %d %d %d\n", idx, cur_index[type], idx-cur_index[type]);
		cur_index[type] = idx;
		//getchar();
	}
	cur_index[type]++;
	printf("get one pkg type %d %d \n", type, len);

	if (type == MAP_DATA_TYPE)
	{
		printf("MAP %d %d \n", type, len);
		static int map_cnt = 0;
		if (headFlag & 0x1 == 1)
		{ //need decompress
			unsigned char outProps[5];
			size_t outPropsSize = sizeof(outProps);
			size_t uncompresslen = sizeof(_MAP_TRAN_DATA_T);
			size_t needUncompressLen = len - 5;
			memcpy(outProps, data, 5);
			int ret = LzmaUncompress(unCompress, &uncompresslen, (const unsigned char *)data + 5, (size_t *)&needUncompressLen, outProps, outPropsSize);
			if (ret != 0)
			{
				printf("failed LZmaUnCompress %d\n", ret);
				return 0;
				exit(-1);
			}
			printf("map %d %d \n", type, len);

			_MAP_TRAN_DATA_T *map_data = (_MAP_TRAN_DATA_T *)unCompress;

			cv::Mat img(600, 600, CV_8UC1, map_data->map_data);
			char imgName[64];
			sprintf(imgName, MAP_DATA_DIR "map_%d.png", map_cnt++);
			imwrite(imgName, img);
		}
	}
	else if (type == IMG_DATA_TYPE)
	{
		printf("IMG %d %d struct size %zd\n", type, len, sizeof(_IMG_TRAN_DATA_T));

		if (headFlag & 0x1 == 1)
		{ //need decompress
			unsigned char outProps[5];
			size_t outPropsSize = sizeof(outProps);
			size_t uncompresslen = sizeof(_IMG_TRAN_DATA_T);
			size_t needUncompressLen = len - 5;
			memcpy(outProps, data, 5);
			int ret = LzmaUncompress(unCompress, &uncompresslen, (const unsigned char *)data + 5, (size_t *)&needUncompressLen, outProps, outPropsSize);
			if (ret != 0)
			{
				printf("failed LZmaUnCompress %d\n", ret);
				return 0;
				exit(-1);
			}
			printf("IMG %d %d \n", type, len);

			long long timestamp = *(long long *)unCompress;
			long long time_s = timestamp / 1000000;
			long long time_ms = timestamp - time_s * 1000000;
			double tframe = time_s + (double)time_ms * 0.000001f;
			cout << "-----------------------------------------  " << tframe << endl;

			//   Yuv420p2Rgb32((const BYTE*)unCompress + 8, rgbbuff, 640, 480);
			cv::Mat img(480, 640, CV_8UC1, unCompress + 8);
			char imgName[64];
			sprintf(imgName, "../data/img/%lld.png", timestamp);
			imwrite(imgName, img);
			static FILE *img_ds_fp = fopen(IMG_DATA_DS, "w");
			static int img_size = 0;
			if (img_ds_fp)
			{
				fwrite(unCompress, sizeof(char), sizeof(_IMG_TRAN_DATA_T), img_ds_fp);
				fflush(img_ds_fp);
				img_size += sizeof(_IMG_TRAN_DATA_T);
				if(img_size>1000*1000*1000)
				{
					static int file_num = 1;
					char buff[32];
					sprintf(buff, IMG_DATA_DS_NUM, file_num);
					img_size = 0;
					fclose(img_ds_fp);
					img_ds_fp = fopen(buff, "w");
					file_num ++;
				}
			}else {
				printf("创建 %s 失败，解析数据集出错\n",IMG_DATA_DS);
				exit(-1);
			}
		}
	}
	else if (type == SLAM_DATA_TYPE)
	{
		printf("SLAM %d %d \n", type, len);
		SLAM_DATA_T *p = (SLAM_DATA_T *)data;
		printf("SLAM F %f %f \n", p->X, p->Y);
		FILE *slamfp = fopen("../data/slamoutput.csv", "a+");
		fprintf(slamfp, "%lld,%f,%f,%f,%d,%d,%d\n", p->Timestamp, p->X, p->Y, p->Yaw, p->State, p->confidence,idx);
		fclose(slamfp);
	}
	else if (type == UP_DATA_TYPE)
	{
		printf("UP %d %d \n", type, len);
		if ((unsigned)len != sizeof(up_data_t)) {
			printf("tcp 中 underpan data 长度为 %d, 应该是 %zd\n",len ,sizeof(up_data_t));
			exit(-1);
		}
		up_data_t *p = (up_data_t *)data;
		printf("UP F %f %f \n", p->X, p->Y);
		FILE *upfp = fopen("../data/odometry.csv", "a+");
		fprintf(upfp, "%lld,%f,%f,%f,%d,%f,%f,%d\n", p->Timestamp, p->LeftWheel, p->RightWheel, p->Yaw, p->RobotInfo, p->X, p->Y, idx);
		fclose(upfp);

		static FILE *updata_ds_fp = fopen(ODO_DATA_DS, "w");
		if (updata_ds_fp)
		{
			fwrite(p, sizeof(char), sizeof(up_data_t), updata_ds_fp);
			fflush(updata_ds_fp);
		} else {
			printf("创建 %s 失败，解析数据集出错\n",ODO_DATA_DS);
			exit(-1);

		}
	} else if(type == LG_LIDAR_DATA_TYPE) {
		struct lg_lidar_data_t *p;
		if (headFlag & 0x1 == 1) {
			unsigned char outProps[5];
			size_t outPropsSize = sizeof(outProps);
			size_t uncompresslen = sizeof(_IMG_TRAN_DATA_T);
			size_t needUncompressLen = len - 5;
			memcpy(outProps, data, 5);
			int ret = LzmaUncompress(unCompress, &uncompresslen, (const unsigned char *)data + 5, (size_t *)&needUncompressLen, outProps, outPropsSize);
			if (ret != 0) {
				printf("failed LZmaUnCompress %d\n", ret);
				return 0;
				exit(-1);
			}
			p = (struct lg_lidar_data_t *)unCompress;
		} else {
			if (sizeof(struct lg_lidar_data_t) != len) {
				printf("tcp 中 lidar data 长度为 %d, 应该是 %zd\n",len ,sizeof(struct lg_lidar_data_t));
				exit(-2);
			}
			p = (struct lg_lidar_data_t *)data;
		}
		static FILE *fp = fopen(LG_LIDAR_DS, "w");
		if (fp) {
			fwrite(p, sizeof(char), sizeof(lg_lidar_data_t), fp);
			fflush(fp);
		} else {
			printf("创建 %s 失败，解析数据集出错\n",LG_LIDAR_DS);
		}
		static FILE *csv_fp = fopen(LG_LIDAR_TS_CSV, "w");
		static long long last_ts = 0;
		if (csv_fp) {
			fprintf(csv_fp, "%lld, %lld\n",p->timestamp, p->timestamp - last_ts);
			last_ts = p->timestamp;
			fflush(csv_fp);
		} else {
			printf("创建 %s 失败，解析数据集出错\n",LG_LIDAR_TS_CSV);
		}

	}else if(type == CONFIG_FILE_DATA_TYPE){
	
		printf("parse config file len %d\n", len);
		const char *p = (const char *) data;
		printf("%c %c %c\n",p[0],p[1],p[2]);
		FILE *fp = fopen(CONFIG_FILE_PATH, "wb");
		if (fp) {
			fwrite(p, 1, len, fp);
			fclose(fp);
		} else {
			printf("创建 %s 失败，解析数据集出错\n",CONFIG_FILE_PATH);
		}

	}  else if (type >= MAX_DATA_TYPE && type < 100){
		static char ch[100];
		if (ch[type] != 'y') {
			printf("Unkonwn Type %d len %d\n", type, len);
			printf("do not save it and continue... ?(y/n)");
			ch[type] = getchar();
		}
		if (ch[type] == 'n') {
			exit(-3);
		}
	}

	return 0;
}

int recv_data_by_len(int sock, void *buff, size_t need_len)
{
	ssize_t readed = 0;
	ssize_t len;
	do
	{
		//len = read(sock, (char *)buff + readed, need_len - readed);
		len = fread((char *)buff + readed, 1, need_len - readed, pTestFile);
		if (len <= 0)
		{
			return -1;
		}
		readed += len;
	} while (need_len > readed);
	return 0;
}

void package_tcp_data_flag(char *flag)
{
	flag[0] = 0x53;
	flag[1] = 0x54;
	flag[2] = 0x55;
	flag[3] = 0x56;
}

int check_tcp_data_flag(char *flag)
{
	struct tcp_package_head head;
	package_tcp_data_flag(head.flag);
	return memcmp(flag, head.flag, sizeof(head.flag));
}

int sync_file(int fd, struct tcp_data_t* tmp)
{
	uint8_t sync_char;
	uint32_t sync_data =0;
	int ret;
	while(1)
	{
		ret = recv_data_by_len(fd, &(sync_char), sizeof(sync_char));
		if (ret != 0)
		{
			printf("open sync_file error!!!\n");
			exit(-1);
		}
		sync_data = sync_data >> 8 | ((sync_char << 24) & 0xff000000);
		if(check_tcp_data_flag((char*)&sync_data)==0)
		{
			printf("check sync_data successful!!!\n");
			ret = recv_data_by_len(fd, &(tmp->head.id), sizeof(tmp->head)-sizeof(tmp->head.flag));
			return 0;
		}
		printf("sync_data %x \t, sync_char %x\n", sync_data, sync_char);
		usleep(5000);
	}
}

void *tcp_recv_data_pth(void *arg)
{
	int sock = ((int *)arg)[0];
	struct tcp_data_t tmp;
	static int ret = 0;
	ret = 0;
	while (1)
	{
		memset(&tmp, 0, sizeof(struct tcp_data_t));
		ret = recv_data_by_len(sock, &(tmp.head), sizeof(tmp.head));
		
		if (ret != 0)
		{
			printf("recv_data_by_len failed %d\n", ret);
			pthread_exit((void *)&ret);
		}
		if (check_tcp_data_flag(tmp.head.flag) != 0)
		{
			printf("check_tcp_data_flag failed \n");
			sync_file(sock, &tmp);
			// pthread_exit((void *)&ret);
		}
		tmp.payload = malloc(tmp.head.payload_len);
		if (tmp.payload == NULL)
		{
			printf("malloc failed\n");
			pthread_exit((void *)&ret);
		}
		ret = recv_data_by_len(sock, tmp.payload, tmp.head.payload_len);
		if (ret != 0)
		{
			printf("recv_data_by_len failed %d\n", ret);
			free(tmp.payload);
			pthread_exit((void *)&ret);
		}
		//parse_recv_tcp_data(&tmp);
		getOnePkg((char *)tmp.payload, tmp.head.payload_len, tmp.head.type, tmp.head.id, tmp.head.payload_flag);
		free(tmp.payload);
	}
}
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

int create_dir(const char *path , int mode)
{
	int ret = 0;
	if (access(path, W_OK|X_OK) != 0) {
		ret = mkdir(path, mode);
		if (ret != 0) {
			printf("create %s failed\n", path);
			return -1;
		}
	}
	return 0;
}
int main(int argc, char *argv[])
{
	const char *path;

	if (argc >= 2) {
		path = argv[1];
	} else {
		printf("Usage: %s tcpStore.data\n",argv[0]);
		return -1;
	}
	create_dir(DS_DIR, 0755);
	create_dir(DATA_DIR, 0755);
	create_dir(MAP_IMG_DIR, 0755);

	if (access(MAP_DATA_DIR, W_OK|X_OK) != 0) {
		printf("目录%s,不存在或没有写权限\n",MAP_DATA_DIR);
		return -1;
	}
	
	if (access(DS_DIR, W_OK|X_OK) != 0) {
		printf("目录%s,不存在或没有写权限\n",DS_DIR);
		return -1;
	}
	
	pTestFile = fopen(path, "rb");
	if (pTestFile == NULL)
	{
		fprintf(stderr,"File error: %s\n", path);
		exit(1);
	}
	/*清除配置文件*/
	FILE *fp = fopen(CONFIG_FILE_PATH, "wb");
	if (fp) {
		fclose(fp);
	}
	int result = 0;

	int sock = 0;
	pthread_t read_th_id;
	pthread_create(&read_th_id, NULL, tcp_recv_data_pth, &sock);
	pthread_join(read_th_id, (void **)NULL);
	char cur_dir[1024] = {0};
	getcwd(cur_dir, sizeof(cur_dir));
	printf("Parse Res:\n");
	printf("\t%s/%s\n",cur_dir, DS_DIR);
	printf("\t%s/%s\n", cur_dir, MAP_IMG_DIR);
	return 0;
}
