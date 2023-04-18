#include "controller.h"

Controller::Controller(int argc, char** argv) :
  init_argc_(argc),
  init_argv_(argv),
  data_ready(false),
  reconnect(false),
  broadcast(false),
  timeOut_ctr(0),
  dev_connected(false),
  readyToSend(false)
{
  serial = new QSerialPort(this);

  if(!initialize())
    return;
}

Controller::~Controller()
{
  //  terminate();

}

bool Controller::initialize()
{
  ROS_INFO("Initialize Arduino Controller");
  ros::init(init_argc_, init_argv_, "alfarobi_controller_node");

  if (!ros::master::check())
  {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle node_;

  std::string port_name;
  node_.param<std::string>("/arduino_controller/portname", port_name, "ttyACM0");
  node_.param<int>("/arduino_controller/baudrate", baudRate, 1000000);
  node_.param<int>("/arduino_controller/waitTimeout", waitTimeout, 500);
  node_.param<bool>("/arduino_controller/broadcast", broadcast, true);


  ROS_INFO("Selected Port: %s Rate: %d", port_name.c_str(), baudRate);
  portName = QString::fromStdString(port_name);

  deviceCheck();

  std::string default_path;
  default_path = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
  loadButton(default_path);

  return true;

}

void Controller::loadButton(const std::string path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load offset config yaml.");
    return;
  }

  // parse button_setting
  YAML::Node button_mode = doc["button_setting"];
  for (YAML::iterator yaml_it = button_mode.begin(); yaml_it != button_mode.end(); ++yaml_it)
  {
    int index;
    std::string mode;

    index = yaml_it->second.as<int>();
    mode = yaml_it->first.as<std::string>();

    button_mapping.insert(std::make_pair(index, mode));
  }

  YAML::Node ardu = doc["arduino_controller"];

  offset = ardu["offset"].as<double>();

}

void Controller::debugDevice()
{
  ROS_INFO("Baud Rate: %d", baudRate);
  ROS_INFO("Port Name: %s", devicedata.portName().toStdString().c_str());
}

void Controller::deviceCheck()
{
  auto ports = QSerialPortInfo::availablePorts();
  for(auto p : ports)
  {
    if(portName == p.portName() && (QString::number(p.productIdentifier(), 16) != QString("6014")))
    {
      ROS_INFO("Found Port: %s" , p.portName().toStdString().c_str());
      devicedata= p;
      debugDevice();
      dev_connected = true;
      break;
    }
    else if(QString::number(p.productIdentifier(), 16) != QString("6014"))
    {
      portName = p.portName();
      devicedata = p;
      ROS_WARN("Found Suitable Arduino Port: %s" , p.portName().toStdString().c_str());
      dev_connected = true;
      break;
    }
  }

  mutex.lock();

  if (currentPortName != portName) {
    currentPortName = portName;
    currentPortNameChanged = true;
    dev_connected = false;
  }

  mutex.unlock();

  emit start();
}

void Controller::run()
{
  ros::NodeHandle ros_node;

  imu_pub = ros_node.advertise<sensor_msgs::Imu>("arduino_controller/imu", 1);
  button_pub = ros_node.advertise<std_msgs::String>("arduino_controller/button", 1);
  button_pub_mainService = ros_node.advertise<std_msgs::String>("arduino_controller/buttonLED", 1);

  ROS_INFO("ERROR Running Loop");

  // ros::Rate loop_rate(91);

  while (ros::ok())
  {
    deviceLoop();
    if(data_ready)
      publishData();
    ros::spinOnce();
    // loop_rate.sleep();
  }

  ROS_WARN("Ros shutdown, proceeding to close the port");
  emit finished();
  QApplication::quit();
}

void Controller::deviceLoop()
{
  if (currentPortNameChanged || reconnect)
  {
    if(reconnect)
      ROS_WARN("Arduino Reconnecting");
    serial->close();
    auto port = QSerialPortInfo::availablePorts();

    for(auto p : port)
    {
      if((p.productIdentifier() == devicedata.productIdentifier()) && (QString::number(p.productIdentifier(), 16) != QString("6014")) )
      {
        ROS_WARN("Arduino Port Changed");
        ROS_INFO("Arduino Port: %s" , p.portName().toStdString().c_str());
        serial->setPortName(p.portName());
        serial->setBaudRate((quint32)baudRate);
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        devicedata = p;
        portName = p.portName();
        currentPortNameChanged = false;
        debugDevice();
      }
    }

    if (!serial->open(QIODevice::ReadOnly)) {
      ROS_ERROR("Arduino Error open device %s", serial->portName().toStdString().c_str());
      dev_connected = false;
      return;
    }
    dev_connected = true;
    reconnect = false;
    timeOut_ctr = 0;
    ROS_WARN("Arduino Ready to Read");
  }


  if (serial->waitForReadyRead(waitTimeout)) {
    reconnect = false;
    timeOut_ctr = 0;
    // read request
    QByteArray requestData = serial->readAll();



    for(auto data: requestData)
    {
      switch (arduino_state)
      {
      case Ardu_Start_U :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'G')
        {
          roll = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_G;
          temp_data.clear();
        }
        break;
      }

      case Ardu_Start_G :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'M')
        {
          pitch = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_M;
          temp_data.clear();
        }
        break;

      }

      case Ardu_Start_M :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'D')
        {
          yaw = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_D;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_D :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'I')
        {
          gyroRoll = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_I;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_I :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'Y')
        {
          gyroPitch = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_Y;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_Y :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'P')
        {
          gyroYaw = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_P;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_P :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'C')
        {
          accelX = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_C;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_C :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'T')
        {
          accelY = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_T;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_T :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'B')
        {
          accelZ = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          arduino_state = Ardu_Start_B;
          temp_data.clear();
        }
        break;
      }
      case Ardu_Start_B :
      {
        data_ready = false;
        temp_data.append(data);
        if(data == 'A')
        {
          QString button_ = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
          button = button_.toInt();
          arduino_state = Ardu_Start_U;
          data_ready = true;
          temp_data.clear();
        }
        break;
      }
// //No Accel Case
      // case Ardu_Start_B :
      // {
      //   data_ready = false;
      //   temp_data.append(data);
      //   if(data == 'A')
      //   {
      //     QString button_ = temp_data.remove(QRegExp("[UGMDIYPCTBA]"));
      //     button = button_.toInt();
      //     arduino_state = Ardu_Start_U;
      //     data_ready = true;
      //     temp_data.clear();
      //   }
      //   break;
      // }
      }
    }


  } else {
    ROS_WARN("Timeout");
    timeOut_ctr++;
    if(timeOut_ctr>100)
    {
      reconnect = true;
      dev_connected = false;
    }

  }
  mutex.lock();

  auto s = QSerialPortInfo::availablePorts();
  for (auto p : s)
  {
    if(p.portName() != portName && p.productIdentifier() == devicedata.productIdentifier())
    {currentPortNameChanged = true;     dev_connected = false;}

    else if(p.portName() == portName && p.productIdentifier() == devicedata.productIdentifier())
      currentPortNameChanged = false;
  }

  mutex.unlock();

}

void Controller::publishData()
{
  // double roll_ = roll.toDouble() * DEGREE2RADIAN; double pitch_ = -pitch.toDouble() * DEGREE2RADIAN; double yaw_ = -yaw.toDouble() * DEGREE2RADIAN;
  double roll_ = -roll.toDouble() * DEGREE2RADIAN; double pitch_ = pitch.toDouble() * DEGREE2RADIAN; double yaw_ = -yaw.toDouble() * DEGREE2RADIAN;
  roll.clear(); pitch.clear(); yaw.clear();
  // double gyroRoll_ = -gyroRoll.toDouble() * DEGREE2RADIAN; double gyroPitch_ = gyroPitch.toDouble() * DEGREE2RADIAN; double gyroYaw_ = gyroYaw.toDouble() * DEGREE2RADIAN;
  double gyroRoll_ = gyroRoll.toDouble() * DEGREE2RADIAN; double gyroPitch_ = -gyroPitch.toDouble() * DEGREE2RADIAN; double gyroYaw_ = gyroYaw.toDouble() * DEGREE2RADIAN;
  gyroRoll.clear(); gyroPitch.clear(); gyroYaw.clear();
  double accelX_ = accelX.toDouble(); double accelY_ = accelY.toDouble(); double accelZ_ = accelZ.toDouble();

  // yaw_ -= offset;
  // if(yaw_ >= 360) yaw_-=360;
  // else if (yaw_ < 0) yaw_ += 360;

  if(!dev_connected)
  {
    roll_ = pitch_ = yaw_ = gyroRoll_ = gyroPitch_ = gyroYaw_ = 0;
  }


  Eigen::Quaterniond quaternion = robotis_framework::convertRPYToQuaternion(roll_, pitch_, yaw_);
  Eigen::Vector3d gyro(gyroRoll_, gyroPitch_, gyroYaw_);
  Eigen::Vector3d accel(accelX_, accelY_, accelZ_);
  sensor_msgs::Imu imu;

  tf::StampedTransform transform;
  tf::TransformBroadcaster br;

  tf::quaternionEigenToMsg(quaternion, imu.orientation);
  tf::vectorEigenToMsg(gyro, imu.angular_velocity);
  tf::vectorEigenToMsg(accel, imu.linear_acceleration);
  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "imu_link";

  imu_pub.publish(imu);

  std_msgs::String button_str;

  if(button != prev_button)
  {
    std::map<int, std::string>::iterator iter = button_mapping.find(button);
    if (iter != button_mapping.end())
    {
      std::string data_ = iter->second;
      button_str.data = data_;
    }
    else button_str.data = std::string("None");

readyToSend = true;

  }

  prev_button = button;

  if(readyToSend)
{
  readyToSend=false;
  // if(button_str.data == "L2" || button_str.data == "L3")
  //   button_pub_mainService.publish(button_str);
  // else
    button_pub.publish(button_str);
}
  if(!dev_connected || reconnect)
    button_str.data = std::string("None");

  //check bentuk imu di rviz
  if(broadcast)
  {
    transform.frame_id_= "base_link";
    transform.child_frame_id_ = "imu_link";
    transform.stamp_ = ros::Time::now();
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
    transform.setRotation(q);
    br.sendTransform(transform);
  }


}
