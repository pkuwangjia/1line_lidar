#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>  //包含基本可视化类
#include <iostream>
#include <string>
// #include <highgui.hpp>
// #include <opencv2/opencv.hpp>

#include "cyber/cyber.h"
#include "gflags/gflags.h"
#include "sensor_image.pb.h"
#include "sensor_pointcloud.pb.h"
#include <X11/Xlib.h>
#include <X11/Xutil.h>

DEFINE_int32(minpts, 3, "");
DEFINE_double(neighbourhood, 4, "");
DEFINE_double(width, 1.12, "");
DEFINE_double(tohead, 0.3, "");
DEFINE_double(distance, 3, "");

pcl::PointCloud<pcl::PointXYZ>::Ptr part(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr core(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
pcl::PointCloud<pcl::PointXYZ>::Ptr obs;

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

// X11 window
#define WIDTH 1280
#define HEIGHT 1024
int win_b_color;
int win_w_color;
XEvent xev;
Window window;
Visual *visual;
XImage *ximage;
GC gc;
char *buffer = (char *)malloc(WIDTH * HEIGHT * 4 * sizeof(char));
Display *display = XOpenDisplay(NULL);

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
    if (i) --i;
    part = clusters[i];
  }
  if (event.getKeySym() == "n" && event.keyDown()) {
    ++i;
    part = clusters[i];
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

float Distance(pcl::PointXYZ &first, pcl::PointXYZ &second) {
  float x = first.x - second.x;
  float y = first.y - second.y;
  return x * x + y * y;
}

void Dbscan(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  core->clear();
  clusters.clear();
  for (auto &point_it : cloud->points) {
    int count = 0;
    for (auto &point_compare : cloud->points) {
      if (Distance(point_it, point_compare) < FLAGS_neighbourhood) count++;
      if (count > FLAGS_minpts) {
        core->points.push_back(point_it);
        break;
      }
    }
  }

  std::cout << "core points: " << core->size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < core->size(); ++i) {
    if (cluster->points.empty()) cluster->points.push_back(core->points[i]);
    if (Distance(cluster->points.back(), core->points[i]) <
        FLAGS_neighbourhood) {
      cluster->push_back(core->points[i]);
      if (i == core->size() - 1) {
        clusters.push_back(cluster);
        cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
        break;
      }
    } else {
      clusters.push_back(cluster);
      cluster.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
  }

  std::cout << "clusters: " << clusters.size() << std::endl;
  for (auto &cluster : clusters)
    std::cout << "cluster" << cluster->size() << std::endl;
}

void ImageCallback(const std::shared_ptr<adu::common::sensor::Image> &msg) {
  AINFO << "Received message " << msg->width();
  // cv::Mat img = cv::Mat(cv::Size(msg->width(), msg->height()), CV_8UC1,
  //                       const_cast<char *>(msg->data().c_str()));
  // cv::imshow("Depthimage", img);
  for (int i = 0; i < WIDTH * HEIGHT; i++) {
    buffer[4 * i] = msg->data()[i];
    buffer[4 * i + 1] = msg->data()[i];
    buffer[4 * i + 2] = msg->data()[i];
    buffer[4 * i + 3] = msg->data()[i];
  }

  ximage = XCreateImage(display, visual, 24, ZPixmap, 0, buffer, WIDTH, HEIGHT,
                        32, 0);
  XPutImage(display, window, gc, ximage, 0, 0, 0, 0, WIDTH, HEIGHT);
}
void DrawRect() {
  static uint last = 0;
  viewer->removeShape("line");
  for (uint i = 0; i < clusters.size(); ++i) {
    pcl::PointXYZ min;  //用于存放三个轴的最小值
    pcl::PointXYZ max;  //用于存放三个轴的最大值
    pcl::getMinMax3D(*clusters[i], min, max);
    char str[25];
    pcl::PointXYZ lf, rf, lb, rb;
    lb.x = lf.x = max.x;
    rf.x = rb.x = min.x;
    rf.y = lf.y = max.y;
    rb.y = lb.y = min.y;
    sprintf(str, "%df",
            i);  //将数字转化为字符串，addLine需要，addLine函数定义在下面
    viewer->removeShape(str);
    viewer->addLine<pcl::PointXYZ>(lf, rf, str);
    sprintf(str, "%db", i);
    viewer->removeShape(str);
    viewer->addLine<pcl::PointXYZ>(lb, rb, str);
    sprintf(str, "%dl", i);
    viewer->removeShape(str);
    viewer->addLine<pcl::PointXYZ>(lf, lb, str);
    sprintf(str, "%dr", i);
    viewer->removeShape(str);
    viewer->addLine<pcl::PointXYZ>(rf, rb, str);
  }
  char str[25];
  for (uint i = clusters.size(); i < last; ++i) {
    sprintf(str, "%df", i);
    viewer->removeShape(str);
    sprintf(str, "%db", i);
    viewer->removeShape(str);
    sprintf(str, "%dl", i);
    viewer->removeShape(str);
    sprintf(str, "%dr", i);
    viewer->removeShape(str);
  }
  last = clusters.size();
}

void PointsCallback(
    const std::shared_ptr<adu::common::sensor::PointCloud> &msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cut(new pcl::PointCloud<pcl::PointXYZ>);
  frame->resize(msg->point_size());
  for (int i = 0; i < msg->point_size(); ++i) {
    const auto &pt = msg->point(i);
    (*frame)[i].x = pt.x();
    (*frame)[i].y = pt.y();
    (*frame)[i].z = pt.z();
  }
  // cut
  for (auto &pt : *frame) {
    if (pt.y < -FLAGS_tohead) cut->push_back(pt);
  }
  Dbscan(cut);
  viewer->removePointCloud("origin");
  viewer->removePointCloud("core_point");
  viewer->addPointCloud<pcl::PointXYZ>(cut, green, "origin");
  viewer->addPointCloud<pcl::PointXYZ>(core, red, "core_point");

  DrawRect();
  viewer->spinOnce(100);
}
int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::Init(argv[0]);

  auto listener_node = apollo::cyber::CreateNode("listener");

  auto image_callback = listener_node->CreateReader<adu::common::sensor::Image>(
      "/sensor/camera/smartereye/image", ImageCallback);

  auto pcl_callback =
      listener_node->CreateReader<adu::common::sensor::PointCloud>(
          "/sensor/lslidar/n301/PointCloud2", PointsCallback);
  CloudAdd();
  Dbscan(cloud);

  win_b_color = BlackPixel(display, DefaultScreen(display));
  win_w_color = BlackPixel(display, DefaultScreen(display));
  window = XCreateSimpleWindow(display, DefaultRootWindow(display), 0, 0, WIDTH,
                               HEIGHT, 0, win_b_color, win_w_color);
  visual = DefaultVisual(display, 0);
  // XSelectInput(display, window, ExposureMask | KeyPressMask);
  XMapWindow(display, window);
  XFlush(display);
  gc = XCreateGC(display, window, 0, NULL);
  // cv::namedWindow("Depthimage", CV_WINDOW_NORMAL);

  std::cout << core->points.size() << std::endl;
  viewer->setBackgroundColor(0.0, 0, 0);  //设置背景色为黑色
  viewer->addCoordinateSystem(1.0);       //建立空间直角坐标系
  //  viewer->setCameraPosition(0,0,200); //设置坐标原点
  viewer->initCameraParameters();  //初始化相机参数

  viewer->registerKeyboardCallback(&keyboardEvent,
                                   (void *)NULL);  //设置键盘回调函数
  pcl::PointXYZ lf, rf, lb, rb;
  lb.x = lf.x = FLAGS_width / 2;
  rf.x = rb.x = -FLAGS_width / 2;
  rf.y = lf.y = FLAGS_tohead;
  rb.y = lb.y = -FLAGS_tohead;
  viewer->addLine<pcl::PointXYZ>(lf, rf, "f");
  viewer->addLine<pcl::PointXYZ>(lb, rb, "b");
  viewer->addLine<pcl::PointXYZ>(lf, lb, "l");
  viewer->addLine<pcl::PointXYZ>(rf, rb, "r");
  viewer->addPointCloud<pcl::PointXYZ>(cloud, green, "origin");
  viewer->addPointCloud<pcl::PointXYZ>(core, red, "core_point");
  viewer->addPointCloud<pcl::PointXYZ>(part, white, "part increase");
  while (!viewer->wasStopped()) {
    // viewer->spinOnce(100);  //显示
    boost::this_thread::sleep(
        boost::posix_time::microseconds(100000));  //随时间
  }
}