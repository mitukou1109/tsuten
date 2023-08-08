#include <tsuten_behavior/behavior_control_panel_ui.h>

#include <ros/package.h>

#include <QLabel>
#include <QLineEdit>
namespace tsuten_behavior
{
  const std::map<TableID, std::string> BehaviorControlPanelUI::DUAL_TABLE_UPPER_COMBO_BOX_ITEMS =
      {{TableID::DUAL_TABLE_UPPER_F, "Front"},
       {TableID::DUAL_TABLE_UPPER_R, "Right"},
       {TableID::DUAL_TABLE_UPPER_B, "Back"},
       {TableID::DUAL_TABLE_UPPER_L, "Left"}};

  const std::unordered_map<TableID, int>
      BehaviorControlPanelUI::MOVABLE_TABLE_CHECK_BOX_GRID_COLUMNS =
          {{TableID::MOVABLE_TABLE_1200, 1},
           {TableID::MOVABLE_TABLE_1500, 2},
           {TableID::MOVABLE_TABLE_1800, 3}};

  const std::unordered_map<TableID, int>
      BehaviorControlPanelUI::SHOOT_BOTTLE_BUTTON_LAYOUT_COLUMNS =
          {{TableID::DUAL_TABLE_LOWER, 0},
           {TableID::DUAL_TABLE_UPPER, 0},
           {TableID::MOVABLE_TABLE_1200, 1},
           {TableID::MOVABLE_TABLE_1500, 2},
           {TableID::MOVABLE_TABLE_1800, 3}};

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

      if (table_id == TableID::HOME ||
          table_id == TableID::DUAL_TABLE_UPPER_F ||
          table_id == TableID::DUAL_TABLE_UPPER_R ||
          table_id == TableID::DUAL_TABLE_UPPER_B ||
          table_id == TableID::DUAL_TABLE_UPPER_L)
      {
        continue;
      }

      QCheckBox *table_check_box = new QCheckBox;
      table_check_box->setStyleSheet(QString::fromStdString(
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

      widgets_.table_check_boxes.insert({table_id, table_check_box});

      QPushButton *shoot_bottle_button = new QPushButton("Shoot");

      widgets_.shoot_bottle_buttons.insert({table_id, shoot_bottle_button});

      switch (table_id)
      {
      case TableID::DUAL_TABLE_UPPER:
        dual_table_layout->insertWidget(0, table_check_box);
        break;
      case TableID::DUAL_TABLE_LOWER:
        dual_table_layout->insertWidget(-1, table_check_box);
        break;
      default:
        tables_layout->addWidget(table_check_box,
                                 1,
                                 MOVABLE_TABLE_CHECK_BOX_GRID_COLUMNS.at(table_id),
                                 Qt::AlignCenter | Qt::AlignBottom);
        break;
      }

      tables_layout->addWidget(
          shoot_bottle_button, (table_id == TableID::DUAL_TABLE_UPPER) ? 0 : 2,
          SHOOT_BOTTLE_BUTTON_LAYOUT_COLUMNS.at(table_id), Qt::AlignCenter);
    }

    widgets_.dual_table_upper_combo_box_ = new QComboBox;
    for (const auto &dual_table_upper_combo_box_items_pair : DUAL_TABLE_UPPER_COMBO_BOX_ITEMS)
    {
      widgets_.dual_table_upper_combo_box_->addItem(
          QString::fromStdString(dual_table_upper_combo_box_items_pair.second));
    }
    widgets_.dual_table_upper_combo_box_->setStyleSheet(
        "QComboBox {padding: 2px 2px 2px 5px; margin-bottom: 10px;}");
    widgets_.dual_table_upper_combo_box_->setEnabled(false);
    dual_table_layout->insertWidget(0, widgets_.dual_table_upper_combo_box_);

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