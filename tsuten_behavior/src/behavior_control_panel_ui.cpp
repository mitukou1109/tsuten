#include <tsuten_behavior/behavior_control_panel_ui.h>

#include <ros/package.h>

namespace tsuten_behavior
{
  BehaviorControlPanelUI::BehaviorControlPanelUI(QWidget *parent) : parent_(parent)
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

      widgets_.table_check_boxes.insert({table_id, table_checkbox});

      QPushButton *shoot_bottle_button = new QPushButton("Shoot");

      widgets_.shoot_bottle_buttons.insert({table_id, shoot_bottle_button});

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

    widgets_.reset_all_shooters_button = new QPushButton("Reset all shooters");
    tables_layout->addWidget(widgets_.reset_all_shooters_button, 0, 2, 1, 2, Qt::AlignCenter);

    widgets_.start_performance_button = new QPushButton("Start performance");
    widgets_.stop_performance_button = new QPushButton("Stop performance");
    widgets_.stop_performance_button->setEnabled(false);

    auto perform_buttons_layout = new QHBoxLayout;
    perform_buttons_layout->addWidget(widgets_.start_performance_button);
    perform_buttons_layout->addWidget(widgets_.stop_performance_button);

    base_layout_ = new QVBoxLayout;
    base_layout_->addLayout(tables_layout);
    base_layout_->addLayout(perform_buttons_layout);
  }
} // namespace tsuten_behavior