#include "moveit_whole_body_ik/csv_whole_body_ik_panel.hpp"

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>
#include <fstream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace moveit_whole_body_ik {
namespace {

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"'); ++i;
      } else {
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      fields.push_back(field); field.clear();
    } else {
      field.push_back(c);
    }
  }
  fields.push_back(field);
  return fields;
}

double number(const std::vector<std::string>& row,
              const std::unordered_map<std::string, size_t>& columns,
              const std::string& name, double fallback = 0.0) {
  const auto it = columns.find(name);
  if (it == columns.end() || it->second >= row.size()) return fallback;
  try { return std::stod(row[it->second]); } catch (...) { return fallback; }
}

}  // namespace

CsvWholeBodyIkPanel::CsvWholeBodyIkPanel(QWidget* parent) : rviz::Panel(parent) {
  path_edit_ = new QLineEdit(this);
  path_edit_->setPlaceholderText("CSV file containing ee_x...ee_qw");
  browse_button_ = new QPushButton("Browse", this);
  load_button_ = new QPushButton("Load CSV", this);
  point_box_ = new QComboBox(this);
  index_spin_ = new QSpinBox(this);
  previous_button_ = new QPushButton("Previous", this);
  next_button_ = new QPushButton("Next", this);
  solve_button_ = new QPushButton("Solve Current", this);
  play_button_ = new QPushButton("Play", this);
  status_label_ = new QLabel("Load a CSV to begin", this);
  metrics_table_ = new QTableWidget(this);
  playback_timer_ = new QTimer(this);
  playback_timer_->setInterval(500);
  status_label_->setWordWrap(true);
  index_spin_->setMinimum(0);
  index_spin_->setEnabled(false);
  point_box_->setEnabled(false);
  previous_button_->setEnabled(false);
  next_button_->setEnabled(false);
  solve_button_->setEnabled(false);
  play_button_->setEnabled(false);
  metrics_table_->setColumnCount(6);
  metrics_table_->setHorizontalHeaderLabels(
      {"Point", "Status", "Position Error (m)", "Orientation Error (rad)",
       "Solve Time (ms)", "Message"});
  metrics_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  metrics_table_->horizontalHeader()->setStretchLastSection(true);
  metrics_table_->verticalHeader()->setVisible(false);
  metrics_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  metrics_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  metrics_table_->setMinimumHeight(160);

  auto* path_layout = new QHBoxLayout;
  path_layout->addWidget(path_edit_);
  path_layout->addWidget(browse_button_);
  auto* nav_layout = new QHBoxLayout;
  nav_layout->addWidget(previous_button_);
  nav_layout->addWidget(next_button_);
  nav_layout->addWidget(solve_button_);
  nav_layout->addWidget(play_button_);
  auto* layout = new QVBoxLayout;
  layout->addLayout(path_layout);
  layout->addWidget(load_button_);
  layout->addWidget(point_box_);
  layout->addWidget(index_spin_);
  layout->addLayout(nav_layout);
  layout->addWidget(status_label_);
  layout->addWidget(metrics_table_);
  layout->addStretch(1);
  setLayout(layout);

  solve_client_ = nh_.serviceClient<SolveSampledWholeBodyIK>(
      "/whole_body_moveit_ik/solve");
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/whole_body_ik/csv_points", 1, true);
  connect(browse_button_, &QPushButton::clicked, this, &CsvWholeBodyIkPanel::browseCsv);
  connect(load_button_, &QPushButton::clicked, this, &CsvWholeBodyIkPanel::loadCsv);
  connect(previous_button_, &QPushButton::clicked, this, &CsvWholeBodyIkPanel::previousPoint);
  connect(next_button_, &QPushButton::clicked, this, &CsvWholeBodyIkPanel::nextPoint);
  connect(solve_button_, &QPushButton::clicked, this, &CsvWholeBodyIkPanel::solveCurrent);
  connect(play_button_, &QPushButton::clicked, this, &CsvWholeBodyIkPanel::togglePlayback);
  connect(point_box_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int index) { current_index_ = index; index_spin_->setValue(index); updatePointUi(); });
  connect(index_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, [this](int index) { if (index >= 0 && index < poses_.size()) { current_index_ = index; point_box_->setCurrentIndex(index); updatePointUi(); } });
  connect(playback_timer_, &QTimer::timeout, this, &CsvWholeBodyIkPanel::playbackTick);
}

void CsvWholeBodyIkPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("csv_path", path_edit_->text());
}

void CsvWholeBodyIkPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString path;
  if (config.mapGetString("csv_path", &path)) {
    if (path.isEmpty()) {
      path = QString::fromStdString(ros::package::getPath("moveit_whole_body_ik")) +
             "/data/template_current_HXD1D_ee_pose_moveit_fk.csv";
    }
    path_edit_->setText(path);
    parseCsv(path);
  }
}

void CsvWholeBodyIkPanel::browseCsv() {
  const QString path = QFileDialog::getOpenFileName(this, "Select CSV", QString(), "CSV (*.csv);;All files (*)");
  if (!path.isEmpty()) path_edit_->setText(path);
}

bool CsvWholeBodyIkPanel::parseCsv(const QString& path) {
  std::ifstream file(path.toStdString());
  if (!file) { setStatus("Cannot open CSV", true); return false; }
  std::string line; if (!std::getline(file, line)) { setStatus("CSV is empty", true); return false; }
  auto header = splitCsv(line);
  if (!header.empty() && header[0].size() >= 3 &&
      static_cast<unsigned char>(header[0][0]) == 0xEF) header[0] = header[0].substr(3);
  std::unordered_map<std::string, size_t> columns;
  for (size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  for (const char* name : {"ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"}) {
    if (!columns.count(name)) { setStatus(QString("Missing CSV column: %1").arg(name), true); return false; }
  }
  QVector<CsvPose> poses;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    const auto row = splitCsv(line);
    CsvPose item;
    item.pose.position.x = number(row, columns, "ee_x");
    item.pose.position.y = number(row, columns, "ee_y");
    item.pose.position.z = number(row, columns, "ee_z");
    item.pose.orientation.x = number(row, columns, "ee_qx");
    item.pose.orientation.y = number(row, columns, "ee_qy");
    item.pose.orientation.z = number(row, columns, "ee_qz");
    item.pose.orientation.w = number(row, columns, "ee_qw", 1.0);
    item.label = QString("%1: (%2, %3, %4)").arg(poses.size())
        .arg(item.pose.position.x, 0, 'f', 3).arg(item.pose.position.y, 0, 'f', 3)
        .arg(item.pose.position.z, 0, 'f', 3);
    poses.push_back(item);
  }
  poses_ = poses;
  point_status_.fill(-1, poses_.size());
  const double unknown = std::numeric_limits<double>::quiet_NaN();
  position_error_.fill(unknown, poses_.size());
  orientation_error_.fill(unknown, poses_.size());
  solve_time_ms_.fill(unknown, poses_.size());
  result_messages_.fill(QString(), poses_.size());
  current_index_ = poses_.isEmpty() ? -1 : 0;
  point_box_->clear(); for (const auto& item : poses_) point_box_->addItem(item.label);
  index_spin_->setRange(0, std::max(0, poses_.size() - 1));
  point_box_->setEnabled(!poses_.isEmpty()); index_spin_->setEnabled(!poses_.isEmpty());
  previous_button_->setEnabled(!poses_.isEmpty()); next_button_->setEnabled(!poses_.isEmpty());
  solve_button_->setEnabled(!poses_.isEmpty()); play_button_->setEnabled(!poses_.isEmpty());
  if (!poses_.isEmpty()) { point_box_->setCurrentIndex(0); updatePointUi(); }
  updateMetricsTable();
  publishCsvMarkers();
  setStatus(QString("Loaded %1 CSV poses").arg(poses_.size()));
  return !poses_.isEmpty();
}

void CsvWholeBodyIkPanel::loadCsv() { parseCsv(path_edit_->text()); }

void CsvWholeBodyIkPanel::updatePointUi() {
  if (current_index_ >= 0 && current_index_ < poses_.size()) {
    index_spin_->setValue(current_index_);
    if (metrics_table_) {
      metrics_table_->selectRow(current_index_);
      metrics_table_->scrollToItem(metrics_table_->item(current_index_, 0));
    }
  }
}

void CsvWholeBodyIkPanel::updateMetricsTable() {
  if (!metrics_table_) return;
  metrics_table_->setRowCount(poses_.size());
  const auto metricText = [](double value) {
    return std::isfinite(value) ? QString::number(value, 'g', 7) : QString("—");
  };
  for (int i = 0; i < poses_.size(); ++i) {
    const QString status = point_status_[i] < 0 ? "Pending" :
        (point_status_[i] > 0 ? "Success" : "Failed");
    metrics_table_->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
    metrics_table_->setItem(i, 1, new QTableWidgetItem(status));
    metrics_table_->setItem(i, 2, new QTableWidgetItem(metricText(position_error_[i])));
    metrics_table_->setItem(i, 3, new QTableWidgetItem(metricText(orientation_error_[i])));
    metrics_table_->setItem(i, 4, new QTableWidgetItem(metricText(solve_time_ms_[i])));
    metrics_table_->setItem(i, 5, new QTableWidgetItem(result_messages_[i]));
  }
  updatePointUi();
}

void CsvWholeBodyIkPanel::previousPoint() { if (!poses_.isEmpty()) point_box_->setCurrentIndex(std::max(0, current_index_ - 1)); }
void CsvWholeBodyIkPanel::nextPoint() { if (!poses_.isEmpty()) point_box_->setCurrentIndex(std::min(poses_.size() - 1, current_index_ + 1)); }

void CsvWholeBodyIkPanel::solveCurrent() {
  if (current_index_ < 0 || current_index_ >= poses_.size()) return;
  SolveSampledWholeBodyIK srv; srv.request.target_pose = poses_[current_index_].pose;
  srv.request.reference_frame = reference_frame_.toStdString(); srv.request.timeout_sec = 5.0;
  srv.request.base_samples = 100; srv.request.base_radius = 1.5;
  srv.request.base_yaw_step = 0.2617993878;
  if (!solve_client_.call(srv)) {
    point_status_[current_index_] = 0; publishCsvMarkers();
    result_messages_[current_index_] = "Service call failed";
    updateMetricsTable();
    setStatus("Service call failed", true); return;
  }
  point_status_[current_index_] = srv.response.success ? 1 : 0;
  position_error_[current_index_] = srv.response.position_error;
  orientation_error_[current_index_] = srv.response.orientation_error;
  solve_time_ms_[current_index_] = srv.response.solve_time_ms;
  result_messages_[current_index_] = QString::fromStdString(srv.response.message);
  updateMetricsTable();
  publishCsvMarkers();
  setStatus(QString("%1/%2: %3, error=%.3g m, %.3g rad")
      .arg(current_index_ + 1).arg(poses_.size()).arg(QString::fromStdString(srv.response.message))
      .arg(srv.response.position_error).arg(srv.response.orientation_error), !srv.response.success);
}

void CsvWholeBodyIkPanel::publishCsvMarkers() {
  visualization_msgs::MarkerArray array;
  for (int i = 0; i < poses_.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame_.toStdString();
    marker.header.stamp = ros::Time::now();
    marker.ns = "csv_whole_body_ik_points";
    marker.id = i;
    // Use an arrow so each CSV quaternion is visible in RViz.  Marker::SPHERE
    // stores the orientation but cannot render it, which made all target
    // poses look like unoriented dots.
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = poses_[i].pose;
    marker.scale.x = (i == current_index_) ? 0.50 : 0.30;
    marker.scale.y = marker.scale.z = (i == current_index_) ? 0.08 : 0.05;
    marker.color.a = 1.0;
    if (point_status_[i] > 0) { marker.color.g = 1.0; }
    else if (point_status_[i] == 0) { marker.color.r = 1.0; }
    else { marker.color.r = 1.0; marker.color.g = 1.0; }
    array.markers.push_back(marker);
  }
  marker_pub_.publish(array);
}

void CsvWholeBodyIkPanel::togglePlayback() {
  if (playback_timer_->isActive()) { playback_timer_->stop(); play_button_->setText("Play"); }
  else { playback_timer_->start(); play_button_->setText("Stop"); }
}

void CsvWholeBodyIkPanel::playbackTick() {
  if (current_index_ < 0 || current_index_ >= poses_.size()) { playback_timer_->stop(); play_button_->setText("Play"); return; }
  solveCurrent();
  if (current_index_ + 1 >= poses_.size()) { playback_timer_->stop(); play_button_->setText("Play"); }
  else nextPoint();
}

void CsvWholeBodyIkPanel::setStatus(const QString& text, bool error) {
  status_label_->setText(text); status_label_->setStyleSheet(error ? "color: red" : "color: green");
}

}  // namespace moveit_whole_body_ik

PLUGINLIB_EXPORT_CLASS(moveit_whole_body_ik::CsvWholeBodyIkPanel, rviz::Panel)
