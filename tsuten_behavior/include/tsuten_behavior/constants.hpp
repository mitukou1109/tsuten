#pragma once

#include <string>
#include <unordered_map>

namespace tsuten_behavior
{
  enum class ShooterID : std::size_t
  {
    DUAL_TABLE_UPPER_L,
    DUAL_TABLE_UPPER_R,
    DUAL_TABLE_LOWER,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800,
    NUM_OF_SHOOTERS
  };

  enum class TableID : int
  {
    DUAL_TABLE_UPPER = 0,
    DUAL_TABLE_LOWER = 1,
    MOVABLE_TABLE_1200 = 2,
    MOVABLE_TABLE_1500 = 3,
    MOVABLE_TABLE_1800 = 4
  };

  const std::unordered_map<TableID, std::string> TABLE_NAMES = {
      {TableID::DUAL_TABLE_LOWER, "dual_table_lower"},
      {TableID::DUAL_TABLE_UPPER, "dual_table_upper"},
      {TableID::MOVABLE_TABLE_1200, "movable_table_1200"},
      {TableID::MOVABLE_TABLE_1500, "movable_table_1500"},
      {TableID::MOVABLE_TABLE_1800, "movable_table_1800"}};
}