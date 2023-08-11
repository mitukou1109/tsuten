#pragma once

#include <map>
#include <string>
#include <unordered_map>

#include <tf2/LinearMath/Quaternion.h>

#include <tsuten_msgs/PerformFeedback.h>
#include <tsuten_msgs/PerformGoal.h>

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
    DUAL_TABLE_UPPER,
    DUAL_TABLE_UPPER_F,
    DUAL_TABLE_UPPER_R,
    DUAL_TABLE_UPPER_B,
    DUAL_TABLE_UPPER_L,
    DUAL_TABLE_LOWER,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800
  };

  enum class GoalID
  {
    DUAL_TABLE_F,
    DUAL_TABLE_R,
    DUAL_TABLE_B,
    DUAL_TABLE_L,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800,
    HOME
  };

  const std::unordered_map<ShooterID, std::string> SHOOTER_NAMES =
      {{ShooterID::DUAL_TABLE_UPPER_L, "dual_table_upper_left_shooter"},
       {ShooterID::DUAL_TABLE_UPPER_R, "dual_table_upper_right_shooter"},
       {ShooterID::DUAL_TABLE_LOWER, "dual_table_lower_shooter"},
       {ShooterID::MOVABLE_TABLE_1200, "movable_table_1200_shooter"},
       {ShooterID::MOVABLE_TABLE_1500, "movable_table_1500_shooter"},
       {ShooterID::MOVABLE_TABLE_1800, "movable_table_1800_shooter"}};

  const std::map<ShooterID, std::string> SHOOTER_TEXTS =
      {{ShooterID::DUAL_TABLE_UPPER_L, "Dual Table (Upper left)"},
       {ShooterID::DUAL_TABLE_UPPER_R, "Dual Table (Upper right)"},
       {ShooterID::DUAL_TABLE_LOWER, "Dual Table (Lower)"},
       {ShooterID::MOVABLE_TABLE_1200, "Movable Table (1200)"},
       {ShooterID::MOVABLE_TABLE_1500, "Movable Table (1500)"},
       {ShooterID::MOVABLE_TABLE_1800, "Movable Table (1800)"}};

  const std::unordered_map<TableID, ShooterID> TABLE_ID_TO_SHOOTER_ID =
      {{TableID::DUAL_TABLE_UPPER_F, ShooterID::DUAL_TABLE_UPPER_R},
       {TableID::DUAL_TABLE_UPPER_R, ShooterID::DUAL_TABLE_UPPER_R},
       {TableID::DUAL_TABLE_UPPER_B, ShooterID::DUAL_TABLE_UPPER_R},
       {TableID::DUAL_TABLE_UPPER_L, ShooterID::DUAL_TABLE_UPPER_R},
       {TableID::DUAL_TABLE_LOWER, ShooterID::DUAL_TABLE_LOWER},
       {TableID::MOVABLE_TABLE_1200, ShooterID::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, ShooterID::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, ShooterID::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableID, TableBaseID> TABLE_ID_TO_TABLE_BASE_ID =
      {{TableID::DUAL_TABLE_UPPER_F, TableBaseID::DUAL_TABLE},
       {TableID::DUAL_TABLE_UPPER_R, TableBaseID::DUAL_TABLE},
       {TableID::DUAL_TABLE_UPPER_B, TableBaseID::DUAL_TABLE},
       {TableID::DUAL_TABLE_UPPER_L, TableBaseID::DUAL_TABLE},
       {TableID::DUAL_TABLE_LOWER, TableBaseID::DUAL_TABLE},
       {TableID::MOVABLE_TABLE_1200, TableBaseID::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, TableBaseID::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, TableBaseID::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableID, GoalID> TABLE_ID_TO_GOAL_ID =
      {{TableID::DUAL_TABLE_UPPER_F, GoalID::DUAL_TABLE_F},
       {TableID::DUAL_TABLE_UPPER_R, GoalID::DUAL_TABLE_R},
       {TableID::DUAL_TABLE_UPPER_B, GoalID::DUAL_TABLE_B},
       {TableID::DUAL_TABLE_UPPER_L, GoalID::DUAL_TABLE_L},
       {TableID::DUAL_TABLE_LOWER, GoalID::DUAL_TABLE_F},
       {TableID::MOVABLE_TABLE_1200, GoalID::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, GoalID::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, GoalID::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableID, uint8_t> TABLE_ID_TO_PERFORM_GOAL_TABLE =
      {{TableID::DUAL_TABLE_UPPER_F, tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER_F},
       {TableID::DUAL_TABLE_UPPER_R, tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER_R},
       {TableID::DUAL_TABLE_UPPER_B, tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER_B},
       {TableID::DUAL_TABLE_UPPER_L, tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER_L},
       {TableID::DUAL_TABLE_LOWER, tsuten_msgs::PerformGoal::DUAL_TABLE_LOWER},
       {TableID::MOVABLE_TABLE_1200, tsuten_msgs::PerformGoal::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, tsuten_msgs::PerformGoal::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, tsuten_msgs::PerformGoal::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableID, std::string> TABLE_NAMES =
      {{TableID::DUAL_TABLE_UPPER, "dual_table_upper"},
       {TableID::DUAL_TABLE_UPPER_F, "dual_table_upper_front"},
       {TableID::DUAL_TABLE_UPPER_R, "dual_table_upper_right"},
       {TableID::DUAL_TABLE_UPPER_B, "dual_table_upper_back"},
       {TableID::DUAL_TABLE_UPPER_L, "dual_table_upper_left"},
       {TableID::DUAL_TABLE_LOWER, "dual_table_lower"},
       {TableID::MOVABLE_TABLE_1200, "movable_table_1200"},
       {TableID::MOVABLE_TABLE_1500, "movable_table_1500"},
       {TableID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  const std::unordered_map<TableID, std::string> TABLE_TEXTS =
      {{TableID::DUAL_TABLE_UPPER, "dual table (upper)"},
       {TableID::DUAL_TABLE_UPPER_F, "dual table (upper, front)"},
       {TableID::DUAL_TABLE_UPPER_R, "dual table (upper, right)"},
       {TableID::DUAL_TABLE_UPPER_B, "dual table (upper, back)"},
       {TableID::DUAL_TABLE_UPPER_L, "dual table (upper, left)"},
       {TableID::DUAL_TABLE_LOWER, "dual table (lower)"},
       {TableID::MOVABLE_TABLE_1200, "movable table (1200)"},
       {TableID::MOVABLE_TABLE_1500, "movable table (1500)"},
       {TableID::MOVABLE_TABLE_1800, "movable table (1800)"}};

  const std::unordered_map<TableBaseID, std::string> TABLE_BASE_NAMES =
      {{TableBaseID::DUAL_TABLE, "dual_table"},
       {TableBaseID::MOVABLE_TABLE_1200, "movable_table_1200"},
       {TableBaseID::MOVABLE_TABLE_1500, "movable_table_1500"},
       {TableBaseID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  const std::unordered_map<TableBaseID, tf2::Vector3> TABLE_SIZES =
      {{TableBaseID::DUAL_TABLE, {800e-3, 800e-3, 2400e-3}},
       {TableBaseID::MOVABLE_TABLE_1200, {500e-3, 500e-3, 1200e-3}},
       {TableBaseID::MOVABLE_TABLE_1500, {500e-3, 500e-3, 1500e-3}},
       {TableBaseID::MOVABLE_TABLE_1800, {500e-3, 500e-3, 1800e-3}}};

  const std::unordered_map<GoalID, TableBaseID> GOAL_ID_TO_TABLE_BASE_ID =
      {{GoalID::DUAL_TABLE_F, TableBaseID::DUAL_TABLE},
       {GoalID::DUAL_TABLE_R, TableBaseID::DUAL_TABLE},
       {GoalID::DUAL_TABLE_B, TableBaseID::DUAL_TABLE},
       {GoalID::DUAL_TABLE_L, TableBaseID::DUAL_TABLE},
       {GoalID::MOVABLE_TABLE_1200, TableBaseID::MOVABLE_TABLE_1200},
       {GoalID::MOVABLE_TABLE_1500, TableBaseID::MOVABLE_TABLE_1500},
       {GoalID::MOVABLE_TABLE_1800, TableBaseID::MOVABLE_TABLE_1800}};

  const std::unordered_map<GoalID, tf2::Quaternion> TABLE_CENTER_TO_GOAL_QUATS =
      {{GoalID::DUAL_TABLE_F, {{0, 0, 1}, 0}},
       {GoalID::DUAL_TABLE_R, {{0, 0, 1}, M_PI_2}},
       {GoalID::DUAL_TABLE_B, {{0, 0, 1}, M_PI}},
       {GoalID::DUAL_TABLE_L, {{0, 0, 1}, -M_PI_2}},
       {GoalID::MOVABLE_TABLE_1200, {{0, 0, 1}, 0}},
       {GoalID::MOVABLE_TABLE_1500, {{0, 0, 1}, 0}},
       {GoalID::MOVABLE_TABLE_1800, {{0, 0, 1}, 0}}};

  const std::unordered_map<GoalID, std::string> GOAL_NAMES =
      {{GoalID::DUAL_TABLE_F, "dual_table_front"},
       {GoalID::DUAL_TABLE_R, "dual_table_right"},
       {GoalID::DUAL_TABLE_B, "dual_table_back"},
       {GoalID::DUAL_TABLE_L, "dual_table_left"},
       {GoalID::MOVABLE_TABLE_1200, "movable_table_1200"},
       {GoalID::MOVABLE_TABLE_1500, "movable_table_1500"},
       {GoalID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  const std::unordered_map<GoalID, std::string> GOAL_TEXTS =
      {{GoalID::DUAL_TABLE_F, "dual table (front)"},
       {GoalID::DUAL_TABLE_R, "dual table (right)"},
       {GoalID::DUAL_TABLE_B, "dual table (back)"},
       {GoalID::DUAL_TABLE_L, "dual table (left)"},
       {GoalID::MOVABLE_TABLE_1200, "movable table (1200)"},
       {GoalID::MOVABLE_TABLE_1500, "movable table (1500)"},
       {GoalID::MOVABLE_TABLE_1800, "movable table (1800)"},
       {GoalID::HOME, "home"}};
}