#include <cv.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>  //包含基本可视化类
#include <iostream>
#include "gflags/gflags.h"
// #include "cyber/cyber.h"
// #include "sensor_image.pb.h"

DEFINE_int32(minpts, 3, "");
DEFINE_double(neighbourhood, 4, "");
pcl::PointCloud<pcl::PointXYZ>::Ptr part(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr core(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(
    cloud, 0, 255, 0);  // green
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(core, 255,
                                                                    0,
                                                                    0);  // red
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(
    part, 255, 255, 255);  // red

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
    new pcl::visualization::PCLVisualizer(
        "test"));  //创建可视化窗口，名字叫做`test`

pcl::PointCloud<pcl::PointXYZ>::Ptr obs;
//设置键盘交互函数,按下`space`键，某事发生
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,
                   void *nothing) {
  static int i = 0;
  if (event.getKeySym() == "space" && event.keyDown()) {
    auto pt = cloud->points[i];
    std::cout << i << std::endl;
    part->push_back(pt);
    i++;
    std::cout << "space" << part->size() << std::endl;
  }
  if (event.getKeySym() == "b" && event.keyDown()) {
    if(i)--i;
part=clusters[i];

  }
    if (event.getKeySym() == "n" && event.keyDown()) {
    ++i;
part=clusters[i];

    }
    viewer->updatePointCloud<pcl::PointXYZ>(part, "part increase");
}

int CloudAdd() {
  // 定义一个 pcl::PointXYZ 类型的点云共享指针，并初始化

  // 从磁盘加载点云数据
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) ==
      -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  // 输出点云数据到屏幕
  std::cout << "Loaded " << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
    std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y
              << " " << cloud->points[i].z << std::endl;
  std::cout << cloud->size() << std::endl;
  return (0);
}
// void MessageCallback(
//     const std::shared_ptr<adu::common::sensor::Image>& msg) {
//   AINFO << "Received message seq-> " << msg->ShortDebugString();

// }
float Distance(pcl::PointXYZ &first,pcl::PointXYZ &second){
  float x=first.x-second.x;
  float y=first.y-second.y;
 return x*x+y*y;
}

void Dbscan() {
  for (auto &point_it : cloud->points) {
    int count = 0;
    for (auto &point_compare : cloud->points) {
      if (Distance(point_it , point_compare) <
          FLAGS_neighbourhood) {
        count++;
      }
    }
    if (count > FLAGS_minpts) core->points.push_back(point_it);
  }

  std::cout << "core points: " << core->size() << std::endl;



  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < core->size(); ++i) {
    if (cluster->points.empty()) cluster->points.push_back(core->points[i]);
    if (Distance(cluster->points.back() , core->points[i])  <
        FLAGS_neighbourhood) {

      cluster->push_back(core->points[i]);

    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr new_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      clusters.push_back(cluster);
      cluster = new_cluster;
    }
  }

  std::cout << "clusters: " << clusters.size() << std::endl;
  for(auto &cluster:clusters)
  std::cout<<"cluster"<<cluster->size()<<std::endl;
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  // apollo::cyber::Init(argv[0]);
  // create talker node
  // auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  // auto listener =
  //     listener_node->CreateReader<adu::common::sensor::Image>(
  //         "channel/chatter", MessageCallback);

  CloudAdd();
  Dbscan();
  std::cout << core->points.size() << std::endl;
  viewer->setBackgroundColor(0.0, 0, 0);  //设置背景色为黑色
  viewer->addCoordinateSystem(1.0);       //建立空间直角坐标系
  //  viewer->setCameraPosition(0,0,200); //设置坐标原点
  viewer->initCameraParameters();  //初始化相机参数

  viewer->registerKeyboardCallback(&keyboardEvent,
                                   (void *)NULL);  //设置键盘回调函数

  viewer->addPointCloud<pcl::PointXYZ>(cloud, green, "origin");
  viewer->addPointCloud<pcl::PointXYZ>(core, red, "core point");
  viewer->addPointCloud<pcl::PointXYZ>(part, white, "part increase");
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);  //显示
    boost::this_thread::sleep(
        boost::posix_time::microseconds(100000));  //随时间
  }
}