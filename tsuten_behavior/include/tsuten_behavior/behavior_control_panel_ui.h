#ifndef Q_MOC_RUN
#include <unordered_map>

#include <tsuten_behavior/constants.hpp>
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
    struct Widgets
    {
      std::unordered_map<TableID, QCheckBox *> table_check_boxes;
      std::unordered_map<TableID, QPushButton *> shoot_bottle_buttons;
      QPushButton *reset_all_shooters_button;

      QPushButton *start_performance_button;
      QPushButton *stop_performance_button;
    };

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
    static const std::unordered_map<TableID, int> TABLE_CHECK_BOX_LAYOUT_COLUMNS;

    static const std::unordered_map<TableID, int> SHOOT_BOTTLE_BUTTON_LAYOUT_COLUMNS;

    QWidget *parent_;

    QVBoxLayout *base_layout_;

    Widgets widgets_;
  };
} // namespace tsuten_behavior