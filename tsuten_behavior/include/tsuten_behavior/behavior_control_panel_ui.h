#ifndef Q_MOC_RUN
#include <unordered_map>

#include <ros/package.h>
#endif

#include <QCheckBox>
#include <QLayout>
#include <QMessageBox>
#include <QPushButton>

namespace tsuten_behavior
{
  class BehaviorControlPanelUI
  {
  public:
    enum class TableID : int
    {
      DUAL_TABLE_UPPER = 0,
      DUAL_TABLE_LOWER = 1,
      MOVABLE_TABLE_1200 = 2,
      MOVABLE_TABLE_1500 = 3,
      MOVABLE_TABLE_1800 = 4
    };

    BehaviorControlPanelUI(QWidget *parent) : parent_(parent)
    {
      std::string image_dir_path = ros::package::getPath("tsuten_behavior") + "/images";

      auto tables_layout = new QGridLayout;
      auto dual_table_layout = new QVBoxLayout;
      dual_table_layout->setSpacing(0);
      tables_layout->addLayout(dual_table_layout, 1, 0, Qt::AlignHCenter | Qt::AlignBottom);

      for (auto &table_name_pair : TABLE_NAMES)
      {
        auto &table_id = table_name_pair.first;
        auto &table_name = table_name_pair.second;

        QCheckBox *table_checkbox = new QCheckBox;
        table_checkbox->setStyleSheet(QString::fromStdString(
            "QCheckBox::indicator:checked {image: url(" +
            image_dir_path + "/" +
            table_name + "_selected.png);}"
                         "QCheckBox::indicator:checked:hover {image: url(" +
            image_dir_path + "/" +
            table_name + "_selected_hover.png);}"
                         "QCheckBox::indicator:checked:pressed {image: url(" +
            image_dir_path + "/" +
            table_name + "_selected_pressed.png);}"
                         "QCheckBox::indicator:unchecked {image: url(" +
            image_dir_path + "/" +
            table_name + "_unselected.png);}"
                         "QCheckBox::indicator:unchecked:hover {image: url(" +
            image_dir_path + "/" +
            table_name + "_unselected_hover.png);}"
                         "QCheckBox::indicator:unchecked:pressed {image: url(" +
            image_dir_path + "/" +
            table_name + "_unselected_pressed.png);}"));

        table_check_boxes_.insert({table_id, table_checkbox});

        QPushButton *shoot_bottle_button = new QPushButton("Shoot");

        shoot_bottle_buttons_.insert({table_id, shoot_bottle_button});

        if (table_id == TableID::DUAL_TABLE_UPPER || table_id == TableID::DUAL_TABLE_LOWER)
        {
          dual_table_layout->insertWidget(static_cast<int>(table_id), table_checkbox);
        }
        else
        {
          tables_layout->addWidget(table_checkbox, 1, static_cast<int>(table_id) - 1,
                                   Qt::AlignCenter | Qt::AlignBottom);
        }

        if (table_id == TableID::DUAL_TABLE_UPPER)
        {
          tables_layout->addWidget(shoot_bottle_button, 0, 0, Qt::AlignCenter);
        }
        else
        {
          tables_layout->addWidget(shoot_bottle_button, 2, static_cast<int>(table_id) - 1,
                                   Qt::AlignCenter);
        }
      }

      reset_all_shooters_button_ = new QPushButton("Reset all shooters");
      tables_layout->addWidget(reset_all_shooters_button_, 0, 2, 1, 2, Qt::AlignCenter);

      start_performance_button_ = new QPushButton("Start performance");
      stop_performance_button_ = new QPushButton("Stop performance");
      stop_performance_button_->setEnabled(false);

      auto perform_buttons_layout = new QHBoxLayout;
      perform_buttons_layout->addWidget(start_performance_button_);
      perform_buttons_layout->addWidget(stop_performance_button_);

      base_layout_ = new QVBoxLayout;
      base_layout_->addLayout(tables_layout);
      base_layout_->addLayout(perform_buttons_layout);
    }

    bool confirm(std::string content)
    {
      return QMessageBox::question(
                 parent_, "Are you sure?",
                 QString::fromStdString(
                     "Do you really want to " + content + "?")) == QMessageBox::Yes;
    }

    void warn(std::string content)
    {
      QMessageBox::warning(parent_, "Warning", QString::fromStdString(content));
    }

    QVBoxLayout *getLayout() { return base_layout_; }

    const std::unordered_map<TableID, QCheckBox *>
        &getTableCheckBoxes() { return table_check_boxes_; }

    const std::unordered_map<TableID, QPushButton *>
        &getShootBottleButtons() { return shoot_bottle_buttons_; }

    const QPushButton *getResetAllShootersButton() { return reset_all_shooters_button_; }

    const QPushButton *getStartPerformanceButton() { return start_performance_button_; }

    const QPushButton *getStopPerformanceButton() { return stop_performance_button_; }

    const std::unordered_map<TableID, std::string> TABLE_NAMES = {
        {TableID::DUAL_TABLE_LOWER, "dual_table_lower"},
        {TableID::DUAL_TABLE_UPPER, "dual_table_upper"},
        {TableID::MOVABLE_TABLE_1200, "movable_table_1200"},
        {TableID::MOVABLE_TABLE_1500, "movable_table_1500"},
        {TableID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  private:
    QVBoxLayout *base_layout_;

    std::unordered_map<TableID, QCheckBox *> table_check_boxes_;
    std::unordered_map<TableID, QPushButton *> shoot_bottle_buttons_;
    QPushButton *reset_all_shooters_button_;

    QPushButton *start_performance_button_;
    QPushButton *stop_performance_button_;

    QWidget *parent_;
  };
} // namespace tsuten_behavior