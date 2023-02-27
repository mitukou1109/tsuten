#pragma once

#include <string>
#include <unordered_map>

namespace tsuten_behavior
{
  enum class ShooterID
  {
    DUAL_TABLE_LOWER,
    DUAL_TABLE_UPPER_L,
    DUAL_TABLE_UPPER_R,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800
  };

  enum class TableID
  {
    DUAL_TABLE,
    DUAL_TABLE_LOWER,
    DUAL_TABLE_UPPER,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800
  };

  const std::unordered_map<TableID, std::string> TABLE_NAMES = {
      {TableID::DUAL_TABLE, "dual_table"},
      {TableID::DUAL_TABLE_LOWER, "dual_table_lower"},
      {TableID::DUAL_TABLE_UPPER, "dual_table_upper"},
      {TableID::MOVABLE_TABLE_1200, "movable_table_1200"},
      {TableID::MOVABLE_TABLE_1500, "movable_table_1500"},
      {TableID::MOVABLE_TABLE_1800, "movable_table_1800"}};

  const std::unordered_map<TableID, std::array<double, 3>> TABLE_SIZES = {
      {TableID::DUAL_TABLE, {800e-3, 800e-3, 2400e-3}},
      {TableID::MOVABLE_TABLE_1200, {500e-3, 500e-3, 1200e-3}},
      {TableID::MOVABLE_TABLE_1500, {500e-3, 500e-3, 1500e-3}},
      {TableID::MOVABLE_TABLE_1800, {500e-3, 500e-3, 1800e-3}}};
}