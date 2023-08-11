#ifndef Q_MOC_RUN
#include <unordered_map>

#include <tsuten_behavior/constants.hpp>
#endif

#include <QCheckBox>
#include <QComboBox>
#include <QLayout>
#include <QMessageBox>
#include <QPushButton>

namespace tsuten_behavior
{
  class BehaviorControlPanelUI
  {
  public:
    struct Widgets
    {
      std::map<TableID, QCheckBox *> table_check_boxes;
      QComboBox *dual_table_upper_combo_box_;
      QPushButton *simulate_bumper_push_button;
      std::unordered_map<TableID, QPushButton *> shoot_bottle_buttons;
      QPushButton *reset_all_shooters_button;

      QPushButton *start_performance_button;
      QPushButton *stop_performance_button;
    };

    static const std::map<TableID, std::string> DUAL_TABLE_UPPER_COMBO_BOX_ITEMS;

    BehaviorControlPanelUI(QWidget *parent);

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

    const Widgets &getWidgets() { return widgets_; }

  private:
    static const std::vector<TableID> WIDGET_TABLE_IDS;

    static const std::unordered_map<TableID, int> MOVABLE_TABLE_CHECK_BOX_GRID_COLUMNS;

    static const std::unordered_map<TableID, int> SHOOT_BOTTLE_BUTTON_GRID_COLUMNS;

    QWidget *parent_;

    QVBoxLayout *base_layout_;

    Widgets widgets_;
  };
} // namespace tsuten_behavior