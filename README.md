### 编译命令  
    在build目录下运行
    cmake ..
    make 
### 执行命令解析原加密文件
    在build目录下运行(很重要)
    执行命令：
    cd build/
    ./ParseTool   ../log/tcpdata.data
    
    解压的数据集图片存在/ParseTool/data/img/文件夹下
    解压的配置文件存在/ParseTool/data_sheet/文件夹下（包含odometry.ds, config_tco_production.yaml和一些img.ds文件）
    解压的数据集地图存在/ParseTool/data/map_img/文件夹下
    将 data_sheet 下的文件放在 /mnt/SDCARD/data_sheet里 

### 脚本执行解析原加密文件
    将数据集文件放在ParseTool/log/tcpStore.data
    在ParseTool/根目录中
    执行命令：
    ./Parse_bash.sh
