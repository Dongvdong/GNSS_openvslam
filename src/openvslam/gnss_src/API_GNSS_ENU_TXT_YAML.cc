
#ifndef API_GNSS_ENU_YAML_cpp
#define API_GNSS_ENU_YAML_cpp

#include "API_GNSS_ENU_TXT_YAML.h"

 


 bool Get_GNSS_INTI_YAML( std::string read_path,gnss_data &gnss_data_int0) {
    try {
        // 读取YAML文件
        YAML::Node config = YAML::LoadFile(read_path);
        //std::cout << "读取gnss 初始点位置" << std::endl;
 
        // 访问YAML中的数据
        std::string lat0 = config["Initial.lat"].as<std::string>();
        std::string lon0 = config["Initial.lon"].as<std::string>();
        std::string alt0 = config["Initial.alt"].as<std::string>();
        double time0=0.0;

               // // 打印读取的数据
        
        //std::cout  << fixed << setprecision(10)<< endl;
        // std::cout << "原点 纬度: " << lat0 << std::endl;
        // std::cout << "原点 经度: " << lon0 << std::endl;
        // std::cout << "原点 高度: " << alt0 << std::endl;

        gnss_data_int0.Set_Gnss(time0,stod(lat0),stod(lon0),stod(alt0));
        std::cout << "GNSS原始点加载完毕" << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "YAML Exception: " << e.what() << std::endl;
        return 0;
    }
 
    return 1;
}
 


 std::string Get_YAML(std::string read_path,char *name) {
    try {
        // 读取YAML文件
        YAML::Node config = YAML::LoadFile(read_path);
 
        // 访问YAML中的数据
        std::string name_value = config[name].as<std::string>();
       
        std::cout << "yaml 读取数据： "<<name <<" " <<  name_value << std::endl;
        return name_value;

    } catch (const YAML::Exception& e) {
        std::cerr << "YAML Exception: " << e.what() << std::endl;
        return "error";
    }
 
  
}
 
 
// 函数用于按照指定分隔符分割字符串
 std::vector<std::string> splitString(const std::string &s, char delim) {

    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

 
// int main() {
 
//     //string path_="/home/r9000k/v2_project/data/NWPU/";
//     string path_="../config/";
//     string path_config=path_+"FHY_config.yaml";// 初始点
//     string path_GNSS=path_+"FHY_gps.txt";// 原始数据
    

//     // 1 获取参考点
//     gnss_data gnss_data_int0(-1,-1,-1,-1);
//     Get_GNSS_INTI_YAML(path_config,gnss_data_int0);
//     std::cout  << fixed << setprecision(10)<< endl;
//     std::cout << "原点 纬度: " << gnss_data_int0.lat << endl;
//     std::cout << "原点 经度: " << gnss_data_int0.lon << endl;
//     std::cout << "原点 高度: " << gnss_data_int0.high << endl;
    
    
//     // 2 当前点的经纬度和高度，作为局部坐标系的原点
//     double origin_latitude =  gnss_data_int0.lat;   // 纬度
//     double origin_longitude = gnss_data_int0.lon; // 经度
//     double origin_height = gnss_data_int0.high;   // 高度
 
//     // 转化为enu，并设置原点
//     GeographicLib::LocalCartesian geoConverter;
//     geoConverter.Reset(origin_latitude, origin_longitude, origin_height);

//     // 转化为ecef，使用WGS84椭球模型
//     GeographicLib::Geocentric wgs84(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());//  6378137  298.257223563LL
//     //GeographicLib::Geocentric cgcs2000(6378137.0, 1.0 / 298.257222101);


//     //3 获取数据点
//     GNSS_TextFileReader reader(path_GNSS, ' '); // 读取路径 分隔符号
    
//     std::vector<gnss_data> gnss_Lists ;
//     if (reader.readFile()) {
//         //reader.printData();
//         gnss_Lists =  reader.get_gnss_List();
//         for(int i=0;i<gnss_Lists.size();i++){
            
//             gnss_Lists[i].Set_orinGnss(gnss_data_int0.lat,gnss_data_int0.lon,gnss_data_int0.high);// 设置原始GNSS点

//             double target_latitude = gnss_Lists[i].lat;
//             double target_longitude = gnss_Lists[i].lon;
//             double target_height = gnss_Lists[i].high;


//             // gnss转化为enu
//             double x, y, z;
//             geoConverter.Forward(target_latitude, target_longitude, target_height, x, y, z);

//             gnss_Lists[i].x=x;
//             gnss_Lists[i].y=y;
//             gnss_Lists[i].z=z;


//             //  WGS84 gnss转化为ecef
//             wgs84.Forward(target_latitude, target_longitude, target_height, x, y, z);
//             gnss_Lists[i].ecef_x=x;
//             gnss_Lists[i].ecef_y=y;
//             gnss_Lists[i].ecef_z=z;      

//             gnss_data gnss_List_i=gnss_Lists[i];
            
//             cout << fixed << setprecision(10)<< endl;
//             cout<< "编号 " << i 
//             << " 时间戳 "<< gnss_List_i.time
//             << " \n纬度 " << gnss_List_i.lat 
//             << " 经度 " << gnss_List_i.lon
//             << " 高度 "<<  gnss_List_i.high 
//             << " \nenu-x " << gnss_List_i.x 
//             << " enu-y " << gnss_List_i.y
//             << " enu-z "<<  gnss_List_i.z 
//             << " \necef_x " << gnss_List_i.ecef_x
//             << " ecef_y " << gnss_List_i.ecef_y
//             << " ecef_z "<<  gnss_List_i.ecef_z
//             << endl;
//         }
//     }
 
 
//     return 0;
// }

#endif