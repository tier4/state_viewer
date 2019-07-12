#include "state_viewer.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "state_viewer");

  QApplication app(argc, argv);
  QWidget* window = new QWidget;
  state_viewer::StateViewer* state_viewer_node = new state_viewer::StateViewer(window);
  // state_viewer_node->show();

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    // state_viewer_node->updateText();
    state_viewer_node->updateImage();
    app.processEvents();
    rate.sleep();
  }
  delete state_viewer_node;
  delete window;
  return 0;
}
