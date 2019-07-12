#include "state_viewer.hpp"

namespace state_viewer
{

void StateViewer::initStatusBarLayout()
{
  status_layout_ = new QHBoxLayout;
  // status_layout_->addWidget(new QLabel("statebar_test 1"));
}

void StateViewer::initStateTextLayout()
{
  state_layout_ = new QVBoxLayout;

  upper_state_label_ = new QLabel("Test Window!");
  lower_state_label_ = new QLabel("Test Window!");

  state_layout_->addWidget(upper_state_label_);
  state_layout_->addWidget(lower_state_label_);
}

void StateViewer::initHeartbeatLayout()
{
  heartbeat_layout_ = new QHBoxLayout;
  // heartbeat_layout_->addWidget(new QLabel("heartbeat_test 1"));
}

void StateViewer::updateStatusBarLayout()
{

}
void StateViewer::updateStateTextLayout()
{

}
void StateViewer::updateHeartbeatLayout()
{
  // static int angle = 0;

  // QImage _img = QImage(QString::fromStdString(image_file_name_));
  // QImage _scaled_img = heartbeat_image_.scaled(QSize(200, 200));
  // QImage rotatedImg = _scaled_img.transformed(QMatrix().rotate(angle));
  // QPixmap dstPix = QPixmap::fromImage(rotatedImg);
  // dstPix.translate(QPoint(30, 30));
  // label_image_->setPixmap(dstPix);

  // angle += 10;
}

QPixmap StateViewer::rostateQpixmapImage(QPixmap img, double angle)
{
  QImage srcImg = img.toImage();
  QPoint center = srcImg.rect().center();
  QMatrix matrix;
  matrix.translate(center.x(), center.y());
  matrix.rotate(angle);
  QImage dstImg = srcImg.transformed(matrix);
  QPixmap dstPix = QPixmap::fromImage(dstImg); //New pixmap rotated

  return dstPix;
}

void StateViewer::callbackFromStateText(const std_msgs::String& msg)
{
  state_text_ = msg.data;
}

void StateViewer::callbackFromUpperStateImgPath(const std_msgs::String& msg)
{
  upper_state_image_path_ = msg.data;
}

void StateViewer::callbackFromLowerStateImgPath(const std_msgs::String& msg)
{
  lower_state_image_path_ = msg.data;
}

void StateViewer::updateText()
{
  QDesktopWidget* desktop;
  std::cout << "width: " << this->geometry().width() << "\nheight: " << this->geometry().height() << "\n"
            << "position: " << this->x() << ", " << this->y() << "\n";

  state_label_->setAlignment(Qt::AlignCenter);
  state_label_->setFont(QFont(QString::fromStdString(font_style_), font_size_));
  state_label_->setText(QString::fromStdString(state_text_));
}

void StateViewer::updateImage()
{
  std::cout << "width: " << this->geometry().width() << "\nheight: " << this->geometry().height() << "\n"
            << "position: " << this->x() << ", " << this->y() << "\n";

  // gif file
  // static QMovie* movie;
  // static bool enable_gif = false;
  //
  // if(state_image_path_.find(".gif") != std::string::npos && (enable_gif == false || movie->state() == QMovie::NotRunning))
  // {
  //   movie = new QMovie(QString::fromStdString(state_image_path_));
  //   movie->setScaledSize(QSize(300, 300));
  //   state_label_->setMovie(movie);
  //   movie->start();
  //   enable_gif = true;
  // }

  // state_label_->setPixmap((QPixmap(QString::fromStdString(state_image_path_))).scaled(QSize(300, 300), Qt::KeepAspectRatio));
  const bool stretch_img = true;
  if (!stretch_img)
  {
    upper_state_label_->setPixmap((QPixmap(QString::fromStdString(upper_state_image_path_))));
    lower_state_label_->setPixmap((QPixmap(QString::fromStdString(lower_state_image_path_))));
  }
  else
  {
    QSize label_size(upper_state_label_->geometry().width(), upper_state_label_->geometry().height());
    upper_state_label_->setPixmap((QPixmap(QString::fromStdString(upper_state_image_path_))).scaled(label_size, Qt::KeepAspectRatio));
    lower_state_label_->setPixmap((QPixmap(QString::fromStdString(lower_state_image_path_))).scaled(label_size, Qt::KeepAspectRatio));
  }

  // state_label_->setAlignment(Qt::AlignCenter);
  upper_state_label_->setAlignment(Qt::AlignCenter);
  lower_state_label_->setAlignment(Qt::AlignCenter);
}

} // namespace state_viewer
