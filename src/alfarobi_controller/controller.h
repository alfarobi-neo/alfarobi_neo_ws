#ifndef Controller_H
#define Controller_H

// #include <QSerialPort>
// #include <QSerialPortInfo>


// #include <libserial/SerialPort.h>
// #include <libserial/SerialStream.h>
// #include "./serial_lib/serialib.h"
#include <QThread>
#include <QMutex>
#include <QDebug>

#include <iostream>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

// #include <QApplication>

#include <robotis_math/robotis_math.h>
//#include <tf2_geometry_msgs/

class Controller : public QObject
{
  Q_OBJECT

public:
  explicit Controller(int argc, char** argv);
  ~Controller();

  enum rx_arduino_state {
    Ardu_Start_U,
    Ardu_Start_G,
    Ardu_Start_M,
    Ardu_Start_D,
    Ardu_Start_I,
    Ardu_Start_Y,
    Ardu_Start_P,
    Ardu_Start_C,
    Ardu_Start_T,
    Ardu_Start_B,
  };


signals:
    void finished();
    void start();

public slots:
  void run();

private:
  QSerialPort* serial;
  QSerialPortInfo devicedata;

  // serialib* serial;
  // // serialib 
  QMutex mutex;
  bool currentPortNameChanged;
  QString portName, currentPortName;
  int baudRate;
  int waitTimeout;
  int timeOut_ctr;
  bool broadcast;
  bool reconnect;
  bool dev_connected;

  double offset;

  int init_argc_;
  char** init_argv_;
  ros::Publisher imu_pub;
  ros::Publisher button_pub;
  ros::Publisher button_pub_mainService;

  bool data_ready;
  rx_arduino_state arduino_state;
  QString roll,pitch,yaw,gyroRoll,gyroPitch,gyroYaw,accelX,accelY,accelZ, temp_data;
  int button, prev_button;
  bool readyToSend;
  std::map<int, std::string> button_mapping;

  void debugDevice();
  void publishData();
  bool initialize();
  void deviceCheck();
  void deviceLoop();
  void loadButton(const std::string path);


};

#endif // Controller_H

