#ifndef MOVEIT_WHOLE_BODY_IK_CSV_WHOLE_BODY_IK_PANEL_HPP
#define MOVEIT_WHOLE_BODY_IK_CSV_WHOLE_BODY_IK_PANEL_HPP

#include <moveit_whole_body_ik/SolveSampledWholeBodyIK.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <visualization_msgs/MarkerArray.h>

#include <QVector>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTimer;

namespace moveit_whole_body_ik {

class CsvWholeBodyIkPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit CsvWholeBodyIkPanel(QWidget* parent = nullptr);
  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;

 private Q_SLOTS:
  void browseCsv();
  void loadCsv();
  void previousPoint();
  void nextPoint();
  void solveCurrent();
  void togglePlayback();
  void playbackTick();

 private:
  struct CsvPose {
    geometry_msgs::Pose pose;
    QString label;
  };

  bool parseCsv(const QString& path);
  void updatePointUi();
  void updateMetricsTable();
  void setStatus(const QString& text, bool error = false);
  void publishCsvMarkers();

  QLineEdit* path_edit_ = nullptr;
  QPushButton* browse_button_ = nullptr;
  QPushButton* load_button_ = nullptr;
  QComboBox* point_box_ = nullptr;
  QSpinBox* index_spin_ = nullptr;
  QPushButton* previous_button_ = nullptr;
  QPushButton* next_button_ = nullptr;
  QPushButton* solve_button_ = nullptr;
  QPushButton* play_button_ = nullptr;
  QLabel* status_label_ = nullptr;
  QTableWidget* metrics_table_ = nullptr;
  QTimer* playback_timer_ = nullptr;

  QVector<CsvPose> poses_;
  int current_index_ = -1;
  ros::NodeHandle nh_;
  ros::ServiceClient solve_client_;
  ros::Publisher marker_pub_;
  QString reference_frame_ = "world";
  QVector<int> point_status_;  // -1 pending, 0 failed, 1 succeeded
  QVector<double> position_error_;
  QVector<double> orientation_error_;
  QVector<double> solve_time_ms_;
  QVector<QString> result_messages_;
};

}  // namespace moveit_whole_body_ik

#endif
