#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include <tsuten_msgs/PerformFeedback.h>

namespace tsuten_behavior
{
  enum class ShooterID
  {
    DUAL_TABLE_UPPER_L,
    DUAL_TABLE_UPPER_R,
    DUAL_TABLE_LOWER,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800
  };

  enum class TableBaseID
  {
    DUAL_TABLE,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800
  };

  enum class TableID
  {
    NONE,
    DUAL_TABLE_UPPER,
    DUAL_TABLE_LOWER,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800
  };

  const std::unordered_map<ShooterID, std::string> SHOOTER_NAMES =
      {{{ShooterID::DUAL_TABLE_UPPER_L, "dual_table_upper_left_shooter"},
        {ShooterID::DUAL_TABLE_UPPER_R, "dual_table_upper_right_shooter"},
        {ShooterID::DUAL_TABLE_LOWER, "dual_table_lower_shooter"},
        {ShooterID::MOVABLE_TABLE_1200, "movable_table_1200_shooter"},
        {ShooterID::MOVABLE_TABLE_1500, "movable_table_1500_shooter"},
        {ShooterID::MOVABLE_TABLE_1800, "movable_table_1800_shooter"}}};

  const std::map<ShooterID, std::string> SHOOTER_TEXTS =
      {{ShooterID::DUAL_TABLE_UPPER_L, "Dual Table (Upper left)"},
       {ShooterID::DUAL_TABLE_UPPER_R, "Dual Table (Upper right)"},
       {ShooterID::DUAL_TABLE_LOWER, "Dual Table (Lower)"},
       {ShooterID::MOVABLE_TABLE_1200, "Movable Table (1200)"},
       {ShooterID::MOVABLE_TABLE_1500, "Movable Table (1500)"},
       {ShooterID::MOVABLE_TABLE_1800, "Movable Table (1800)"}};

  const std::unordered_map<TableID, ShooterID> TABLE_ID_TO_SHOOTER_ID =
      {{TableID::DUAL_TABLE_UPPER, ShooterID::DUAL_TABLE_UPPER_R},
       {TableID::DUAL_TABLE_LOWER, ShooterID::DUAL_TABLE_LOWER},
       {TableID::MOVABLE_TABLE_1200, ShooterID::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, ShooterID::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, ShooterID::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableID, TableBaseID> TABLE_ID_TO_TABLE_BASE_ID =
      {{TableID::DUAL_TABLE_UPPER, TableBaseID::DUAL_TABLE},
       {TableID::DUAL_TABLE_LOWER, TableBaseID::DUAL_TABLE},
       {TableID::MOVABLE_TABLE_1200, TableBaseID::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, TableBaseID::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, TableBaseID::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableBaseID, std::string> TABLE_BASE_NAMES =
      {{TableBaseID::DUAL_TABLE, "dual_table"},
       {TableBaseID::MOVABLE_TABLE_1200, "movable_table_1200"},
       {TableBaseID::MOVABLE_TABLE_1500, "movable_table_1500"},
       {TableBaseID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  const std::unordered_map<TableID, std::string> TABLE_NAMES =
      {{TableID::DUAL_TABLE_UPPER, "dual_table_upper"},
       {TableID::DUAL_TABLE_LOWER, "dual_table_lower"},
       {TableID::MOVABLE_TABLE_1200, "movable_table_1200"},
       {TableID::MOVABLE_TABLE_1500, "movable_table_1500"},
       {TableID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  const std::unordered_map<TableBaseID, std::array<double, 3>> TABLE_SIZES =
      {{TableBaseID::DUAL_TABLE, {800e-3, 800e-3, 2400e-3}},
       {TableBaseID::MOVABLE_TABLE_1200, {500e-3, 500e-3, 1200e-3}},
       {TableBaseID::MOVABLE_TABLE_1500, {500e-3, 500e-3, 1500e-3}},
       {TableBaseID::MOVABLE_TABLE_1800, {500e-3, 500e-3, 1800e-3}}};

  const std::map<TableID, std::string> TABLE_TEXTS =
      {{TableID::DUAL_TABLE_UPPER, "Dual Table (Upper)"},
       {TableID::DUAL_TABLE_LOWER, "Dual Table (Lower)"},
       {TableID::MOVABLE_TABLE_1200, "Movable Table (1200)"},
       {TableID::MOVABLE_TABLE_1500, "Movable Table (1500)"},
       {TableID::MOVABLE_TABLE_1800, "Movable Table (1800)"}};

  const std::unordered_map<int, std::string> PERFORM_STATUS_TEXTS =
      {{tsuten_msgs::PerformFeedback::MOVING_TO_DUAL_TABLE,
        "Moving to dual table"},
       {tsuten_msgs::PerformFeedback::ALIGNING_AT_DUAL_TABLE_UPPER,
        "Aligning at dual table (upper)"},
       {tsuten_msgs::PerformFeedback::SHOOTING_ON_DUAL_TABLE_UPPER,
        "Shooting on dual table (upper)"},
       {tsuten_msgs::PerformFeedback::ALIGNING_AT_DUAL_TABLE_LOWER,
        "Aligning at dual table (lower)"},
       {tsuten_msgs::PerformFeedback::SHOOTING_ON_DUAL_TABLE_LOWER,
        "Shooting on dual table (lower)"},
       {tsuten_msgs::PerformFeedback::MOVING_TO_MOVABLE_TABLE_1200,
        "Moving to movable table (1200)"},
       {tsuten_msgs::PerformFeedback::ALIGNING_AT_MOVABLE_TABLE_1200,
        "Aligning at movable table (1200)"},
       {tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1200,
        "Shooting on movable table (1200)"},
       {tsuten_msgs::PerformFeedback::MOVING_TO_MOVABLE_TABLE_1500,
        "Moving to movable table (1500)"},
       {tsuten_msgs::PerformFeedback::ALIGNING_AT_MOVABLE_TABLE_1500,
        "Aligning at movable table (1500)"},
       {tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1500,
        "Shooting on movable table (1500)"},
       {tsuten_msgs::PerformFeedback::MOVING_TO_MOVABLE_TABLE_1800,
        "Moving to movable table (1800)"},
       {tsuten_msgs::PerformFeedback::ALIGNING_AT_MOVABLE_TABLE_1800,
        "Aligning at movable table (1800)"},
       {tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1800,
        "Shooting on movable table (1800)"},
       {tsuten_msgs::PerformFeedback::RETURNING,
        "Returning to home"}};
}