// Copyright 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POIROT_TUI__TUI_RENDERER_HPP_
#define POIROT_TUI__TUI_RENDERER_HPP_

#include <ncurses.h>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "poirot_tui/data_manager.hpp"

namespace poirot_tui {

/// Tab enumeration
enum class Tab {
  TABLE,
  GRAPH_WALL_TIME,
  GRAPH_CPU_TIME,
  GRAPH_MEMORY,
  GRAPH_IO_READ,
  GRAPH_IO_WRITE,
  GRAPH_CTX_SWITCHES,
  GRAPH_ENERGY,
  GRAPH_CO2
};

/// TUI Renderer using ncurses with mouse support
class TuiRenderer {
public:
  TuiRenderer();
  ~TuiRenderer();

  /// Initialize ncurses
  bool initialize();

  /// Shutdown ncurses
  void shutdown();

  /// Render the current view
  void render(const DataManager &data_manager);

  /// Handle input and return false if should quit
  bool handleInput(DataManager &data_manager);

  /// Get current tab
  Tab getCurrentTab() const { return this->current_tab_; }

  /// Check if initialized
  bool isInitialized() const { return this->initialized_; }

private:
  /// Render table view
  void renderTableView(const DataManager &data_manager);

  /// Render graph view
  void renderGraphView(const DataManager &data_manager,
                       GraphDataType data_type);

  /// Render header bar with tabs
  void renderHeader();

  /// Render footer with help
  void renderFooter();

  /// Render function selector for graphs
  void renderFunctionSelector(const DataManager &data_manager, int start_row,
                              int width);

  /// Draw ASCII graph
  void drawGraph(
      const std::vector<std::pair<std::string, std::vector<DataPoint>>> &data,
      GraphDataType data_type, int start_row, int start_col, int height,
      int width);

  /// Format number with appropriate units
  std::string formatNumber(double value, const std::string &unit) const;

  /// Format time in microseconds
  std::string formatTime(double us) const;

  /// Format bytes
  std::string formatBytes(int64_t bytes) const;

  /// Get column name
  std::string getColumnName(SortColumn col) const;

  /// Get tab name
  std::string getTabName(Tab tab) const;

  /// Get data type name
  std::string getDataTypeName(GraphDataType type) const;

  /// Get value from data point by type
  double getValueByType(const DataPoint &dp, GraphDataType type) const;

  /// Handle mouse input
  bool handleMouseInput(DataManager &data_manager, MEVENT &event);

  /// Get tab at x position
  Tab getTabAtPosition(int x) const;

  /// Get column at x position for sorting
  SortColumn getColumnAtPosition(int x) const;

  /// Cycle to next tab
  void nextTab();

  /// Cycle to previous tab
  void prevTab();

  /// Move selection up
  void moveUp();

  /// Move selection down
  void moveDown();

  /// Page up
  void pageUp();

  /// Page down
  void pageDown();

  /// Move to first row
  void moveToFirst();

  /// Move to last row
  void moveToLast(int total_rows);

  /// Cycle sort column
  void cycleSortColumn();

  /// Toggle sort order
  void toggleSortOrder();

  /// Set sort column directly
  void setSortColumn(SortColumn col);

  /// Select specific tab
  void selectTab(Tab tab);

  bool initialized_;
  bool mouse_enabled_;
  Tab current_tab_;
  SortColumn sort_column_;
  bool sort_ascending_;
  int selected_row_;
  int scroll_offset_;
  int graph_scroll_offset_;
  int max_rows_;
  int terminal_width_;
  int terminal_height_;

  // Tab positions for mouse click detection
  std::map<Tab, std::pair<int, int>> tab_positions_; // Tab -> (start_x, end_x)

  // Column positions for mouse click detection
  std::vector<std::pair<int, SortColumn>>
      column_positions_; // (start_x, column)

  // Color pairs
  static constexpr int COLOR_HEADER = 1;
  static constexpr int COLOR_SELECTED = 2;
  static constexpr int COLOR_NORMAL = 3;
  static constexpr int COLOR_TAB_ACTIVE = 4;
  static constexpr int COLOR_TAB_INACTIVE = 5;
  static constexpr int COLOR_GRAPH_1 = 6;
  static constexpr int COLOR_GRAPH_2 = 7;
  static constexpr int COLOR_GRAPH_3 = 8;
  static constexpr int COLOR_GRAPH_4 = 9;
  static constexpr int COLOR_ENABLED = 10;
  static constexpr int COLOR_DISABLED = 11;
  static constexpr int COLOR_HELP_KEY = 12;
  static constexpr int COLOR_GRAPH_5 = 13;
  static constexpr int COLOR_GRAPH_6 = 14;
  static constexpr int COLOR_GRAPH_7 = 15;
  static constexpr int COLOR_GRAPH_8 = 16;
};

} // namespace poirot_tui

#endif // POIROT_TUI__TUI_RENDERER_HPP_
