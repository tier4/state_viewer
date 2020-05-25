/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __STATE_VIEWER_HPP__
#define __STATE_VIEWER_HPP__
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <QApplication>
#include <QDesktopWidget>
#include <QDialog>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLocale>
#include <QMatrix>
#include <QMovie>
#include <QPixmap>
#include <QPoint>
#include <QSize>
#include <QVBoxLayout>

namespace state_viewer
{

class StateViewer : public QDialog
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber text_sub, upper_img_sub, lower_img_sub;

  // screen settings
  int screen_pos_x_;
  int screen_pos_y_;
  int font_size_;
  double screen_size_width_;
  double screen_size_height_;
  bool full_screen_;
  std::string font_style_;
  // topics
  std::string state_topic_name_;
  std::string state_text_;
  std::string upper_state_image_path_;
  std::string lower_state_image_path_;

  QVBoxLayout* state_layout_;
  QHBoxLayout* status_layout_;
  QHBoxLayout* heartbeat_layout_;
  QLabel* state_label_;
  QLabel* upper_state_label_;
  QLabel* lower_state_label_;
  QPixmap* pixmap_;
  QImage heartbeat_image_;

  void initStatusBarLayout();
  void initStateTextLayout();
  void initHeartbeatLayout();
  void updateStatusBarLayout();
  void updateStateTextLayout();
  void updateHeartbeatLayout();

  void callbackFromStateText(const std_msgs::String& msg);
  void callbackFromUpperStateImgPath(const std_msgs::String& msg);
  void callbackFromLowerStateImgPath(const std_msgs::String& msg);

  QPixmap rostateQpixmapImage(QPixmap img, double angle);

public:
  StateViewer(QWidget* window)
    : QDialog(window)
    , private_nh_("~")
    , screen_pos_x_(0)
    , screen_pos_y_(0)
    , font_size_(50)
    , screen_size_width_(1920)
    , screen_size_height_(1080)
    , full_screen_(false)
    , font_style_("Arial")
    , state_topic_name_("/state_interpreter/state_text")
    , state_text_("")
    , upper_state_image_path_("")
    , lower_state_image_path_("")
  {
    this->resize(screen_size_width_, screen_size_height_);
    this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint | Qt::WindowMinMaxButtonsHint);

    private_nh_.getParam("screen_pos_x", screen_pos_x_);
    private_nh_.getParam("screen_pos_y", screen_pos_y_);
    private_nh_.getParam("font_size", font_size_);
    private_nh_.getParam("screen_size_width", screen_size_width_);
    private_nh_.getParam("screen_size_height", screen_size_height_);
    private_nh_.getParam("full_screen", full_screen_);
    private_nh_.getParam("font_style", font_style_);
    private_nh_.getParam("state_topic_name", state_topic_name_);

    text_sub = nh_.subscribe(state_topic_name_, 1, &StateViewer::callbackFromStateText, this);
    upper_img_sub = nh_.subscribe("/state_interpreter/upper_state_img_path", 1, &StateViewer::callbackFromUpperStateImgPath, this);
    lower_img_sub = nh_.subscribe("/state_interpreter/lower_state_img_path", 1, &StateViewer::callbackFromLowerStateImgPath, this);

    initStatusBarLayout();
    initStateTextLayout();
    initHeartbeatLayout();

    QVBoxLayout* main_layout_ = new QVBoxLayout;
    main_layout_->addLayout(status_layout_);
    main_layout_->addLayout(state_layout_);
    main_layout_->addLayout(heartbeat_layout_);

    this->setLayout(main_layout_);

    this->move(screen_pos_x_, screen_pos_y_);
    if(full_screen_)
      this->showFullScreen();
    else
      this->show();
  };
  // virtual ~StateViewer ();

  void updateText();
  void updateImage();
};

} // namespace state_viewer
#endif
