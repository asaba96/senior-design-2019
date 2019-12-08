/*
 * Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira
 * <gpereira@ufmg.br>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

// S30 Laser rangefinder node- Guilherme Pereira, May, 2016
// You must set the sensor using the windows program first. The bound rate must
// be 115200 and the frequency larger than 50Hz

#include <errno.h>
#include <fcntl.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

int fdes;
bool exit_;

void SigintHandler(int sig) {
  exit_ = true;
  close(fdes);
  ros::Duration(0.5).sleep();
  ros::shutdown();
}

int set_interface_attribs(int fd, int speed, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    ROS_ERROR("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;  // disable break processing
  tty.c_lflag = 0;         // no signaling chars, no echo,
                           // no canonical processing
  tty.c_oflag = 0;         // no remapping, no delays
  tty.c_cc[VMIN] = 0;      // read doesn't block
  tty.c_cc[VTIME] = 5;     // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                      // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ROS_ERROR("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void set_blocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    ROS_ERROR("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;  // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
    ROS_ERROR("error %d setting term attributes", errno);
}

bool isValidNumber(char a) {
  return ((a >= 48) && (a <= 57));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sf30_node");
  ros::NodeHandle n;

  signal(SIGINT, SigintHandler);

  // load params
  std::string s;
  std::string topic_name;
  std::string frame_id = "sf30";
  bool debug_;
  bool publish_as_pose;
  int update_rate;
  double variance_;

  n.getParam("sf30_node/portname", s);
  n.getParam("sf30_node/debug", debug_);
  n.getParam("sf30_node/publish_as_pose", publish_as_pose);
  n.getParam("sf30_node/update_rate", update_rate);
  n.getParam("sf30_node/topic_name", topic_name);
  n.getParam("sf30_node/frame_id", frame_id);
  n.getParam("sf30_node/altitude_variance", variance_);

  ros::Rate loop_rate(update_rate);
  ros::Publisher pub_;

  if (publish_as_pose) {
    pub_ =
        n.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, 10);
  } else {
    pub_ = n.advertise<sensor_msgs::LaserScan>(topic_name, 10);
  }

  const char *portname = s.c_str();

  fdes = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fdes < 0) {
    ROS_ERROR("Error %d opening %s: %s", errno, portname, strerror(errno));
    return 1;
  }

  // set speed to 115200 bps, 8n1 (no parity)
  set_interface_attribs(fdes, B115200, 0);
  // set blocking
  set_blocking(fdes, 1);

  char buf[1];
  int nc, i;
  float num = 0.0;
  int confidence = 0;
  tcflush(fdes, TCIFLUSH);
  ros::Time last_time = ros::Time::now();
  exit_ = false;

  // Main loop
  while (ros::ok() && !exit_) {
    tcflush(fdes, TCIFLUSH);  // to get the most recent data, flush the buffer

    while (ros::ok() && !exit_) {  // format of the data 00.00 m\n\n

      nc = read(fdes, buf, 1);
      ROS_INFO_COND(debug_, " char: %d ", (int)buf[0]);

      if ((nc == 1) && (buf[0] == 'm')) {  // Look for the 'm'

        num = 0.0;
        confidence = 0;

        nc = read(fdes, buf, 1);  // should be a Linebreak
        ROS_INFO_COND(debug_, " char: %d ", (int)buf[0]);
        if ((nc != 1) || (buf[0] != 10 && buf[0] != 13)) {
          ROS_ERROR("First line break not found : %d", buf[0]);
          break;
        }
        nc = read(fdes, buf, 1);  // should be another Linebreak
        ROS_INFO_COND(debug_, " char: %d ", (int)buf[0]);
        if ((nc != 1) || buf[0] != 10) {
          ROS_ERROR("Second line break not found : %d", buf[0]);
          break;
        }
        i = 1;
        do {  // Process the number before the point.
          nc = read(fdes, buf, 1);
          ROS_INFO_COND(debug_, " char: %d ", (int)buf[0]);
          if ((nc == 1) && (isValidNumber(buf[0]))) {
            num = num * i + atof(buf);
            i = i * 10;
          } else {
            break;
          }
        } while (1);

        if ((nc != 1) || (buf[0] != '.')) {
          // In normal situation, only exit the loop if
          // find the decimal point
          num = 0.0;
          ROS_ERROR("Not decimal point: %c", buf[0]);
          break;
        }

        nc = read(fdes, buf, 1);  // should be the first decimal digit
        ROS_INFO_COND(debug_, " char: %d ", (int)buf[0]);
        if ((nc != 1) || (!isValidNumber(buf[0]))) {
          num = 0.0;
          ROS_ERROR("(2) Not valid number: %c", buf[0]);
          break;
        }
        num = num + atof(buf) / 10.0;

        nc = read(fdes, buf, 1);  // should be the second decimal digit
        ROS_INFO_COND(debug_, " char: %d ", (int)buf[0]);
        if ((nc != 1) || (!isValidNumber(buf[0]))) {
          num = 0.0;
          ROS_ERROR("(3) Not valid number: %c", buf[0]);
          break;
        }
        num = num + atof(buf) / 100.0;

        confidence = 1;  // If we got so far, we can thrust the number in num!

        break;
      }  // if

    }  // while

    ros::Time now = ros::Time::now();
    ros::Duration duration = now - last_time;
    last_time = now;

    if (publish_as_pose) {
      geometry_msgs::PoseWithCovarianceStamped data;
      data.header.frame_id = frame_id;
      data.header.stamp = ros::Time::now();
      data.pose.pose.position.z = num;
      data.pose.covariance[14] = variance_;
      pub_.publish(data);
    } else {
      sensor_msgs::LaserScan data;
      data.header.frame_id = "sf30";
      data.header.stamp = ros::Time::now();
      data.scan_time = duration.toSec();
      data.range_max = 100.0;
      data.ranges.push_back(num);

      // Confidence = 0, range = 0 cannot be thrusted
      data.intensities.push_back((double)confidence);
      pub_.publish(data);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
