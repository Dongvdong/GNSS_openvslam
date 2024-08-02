
#ifndef API_GNSS_ENU_TXT_YAML
#define API_GNSS_ENU_TXT_YAML

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
 
#include<iomanip>


#include <yaml-cpp/yaml.h>
#include <GeographicLib/LocalCartesian.hpp>
 
using namespace std;
 
 
class gnss_data
{
 
public:
   // 原始数据
   double time;
   double lat;
   double lon;
   double high;
     
   // ENU坐标GNSS的原点
   double lat0;
   double lon0;
   double high0;
 
   // 在ENU坐标下
   double x;
   double y;
   double z;
 
   double ecef_x;
   double ecef_y;
   double ecef_z;

public:
   // 初始化 赋予原始数据
   gnss_data(double time_,double lat_,double lon_,double high_):time(time_), lat(lat_),lon(lon_),high(high_) {}
   
   void Set_Gnss(double time_,double lat_,double lon_,double high_){
        time=time_;
        lat=lat_;
        lon=lon_;
        high=high_;
   };
 
   void Set_orinGnss(double lat0_,double lon0_,double high0_){
     lat0=lat0_;
     lon0=lon0_;
     high0=high0_;
   }
 
   void Set_ENU(double x_,double y_,double z_){
     x=x_;
     y=y_;
     z=z_;
   }
 
};
 
 
class GNSS_TextFileReader {
public:
    std::string filename;
    char delimiter;
    std::vector<std::vector<std::string>> data; // 二维向量，存储每一行拆分后的数据
    std::vector<gnss_data> gnss_List;
 
 
  
public:
    GNSS_TextFileReader(const std::string &filename, char delimiter)
        : filename(filename), delimiter(delimiter) {}
 
    bool readFile() {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file " << filename << std::endl;
            return false;
        }
 
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> row;
 
            while (std::getline(ss, token, delimiter)) { // delimiter 分割
                row.push_back(token);
            }
            data.push_back(row);//  保存string数据double
 
            gnss_data gnss_data_i( stod(row[0]),stod(row[1]),stod(row[2]),stod(row[3])); //保存double数据
            gnss_List.push_back(gnss_data_i);
        }
 
        file.close();
 
 
 
        return true;
    }
 

 
    void printData() {
        // for (const auto &row : data) {
        //     for (const auto &col : row) {
        //         std::cout << col << " ";
        //     }
        //     std::cout << std::endl;
        // }
 
       //for (const auto &gnss_List_i : gnss_List) { 
       for(int i=0;i<gnss_List.size();i++){
        gnss_data gnss_List_i=gnss_List[i];
        cout<< "编号 " << i << " 时间戳 "<< gnss_List_i.time<< " 纬度 " << gnss_List_i.lat <<  "  经度 " << gnss_List_i.lon<< "  高度 "<<  gnss_List_i.high << fixed << setprecision(10)<< endl;
       }
 
    }
 
    const std::vector<std::vector<std::string>>& getData() const {
        return data;
    }
 
    const std::vector<gnss_data>& get_gnss_List() const {
        return gnss_List;
    }
};


int Get_GNSS_INTI_YAML( std::string read_path,gnss_data &gnss_data_int0) {
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
        return 1;
    }
 
    return 0;
}
 


std::string  Get_YAML(std::string read_path,char *name) {
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
 
    return 0;
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