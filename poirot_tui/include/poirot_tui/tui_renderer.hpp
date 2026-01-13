// Copyright 2026 Miguel Ángel González Santamarta
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

/// @brief Tab enumeration
enum class Tab {
  TABLE,
  GRAPH_WALL_TIME,
  GRAPH_CPU_TIME,
  GRAPH_MEMORY,
  GRAPH_GPU_MEMORY,
  GRAPH_IO_READ,
  GRAPH_IO_WRITE,
  GRAPH_CTX_SWITCHES,
  GRAPH_CPU_ENERGY,
  GRAPH_GPU_ENERGY,
  GRAPH_ENERGY,
  GRAPH_CO2
};

/**
 * @class TuiRenderer
 * @brief Class responsible for rendering the TUI using ncurses
 */
class TuiRenderer {
public:
  /**
   * @brief Constructor
   */
  TuiRenderer();

  /**
   * @brief Destructor
   */
  ~TuiRenderer();

  /**
   * @brief Initialize ncurses
   * @return True if successful, false otherwise
   */
  bool initialize();

  /**
   * @brief Shutdown ncurses
   */
  void shutdown();

  /**
   * @brief Render the TUI
   * @param data_manager Reference to the DataManager
   */
  void render(const DataManager &data_manager);

  /**
   * @brief Handle user input
   * @param data_manager Reference to the DataManager
   * @return True if the application should continue running, false to exit
   */
  bool handle_input(DataManager &data_manager);

  /**
   * @brief Get current tab
   * @return Current Tab
   */
  Tab get_current_tab() const { return this->current_tab_; }

  /**
   * @brief Check if TUI is initialized
   * @return True if initialized, false otherwise
   */
  bool is_initialized() const { return this->initialized_; }

private:
  /**
   * @brief Render table view
   * @param data_manager Reference to the DataManager
   */
  void render_table_view(const DataManager &data_manager);

  /**
   * @brief Render graph view
   * @param data_manager Reference to the DataManager
   * @param data_type Type of data to graph
   */
  void render_graph_view(const DataManager &data_manager,
                         GraphDataType data_type);

  /**
   * @brief Render header
   */
  void render_header();

  /**
   * @brief Render footer
   */
  void render_footer();

  /**
   * @brief Render function selector
   * @param data_manager Reference to the DataManager
   * @param start_row Starting row for rendering
   * @param width Width of the selector
   */
  void render_function_selector(const DataManager &data_manager, int start_row,
                                int width);

  /**
   * @brief Draw graph for given data
   * @param data Vector of pairs of function name and their DataPoints
   * @param data_type Type of data to graph
   * @param start_row Starting row for rendering
   * @param start_col Starting column for rendering
   * @param height Height of the graph area
   * @param width Width of the graph area
   */
  void draw_graph(
      const std::vector<std::pair<std::string, std::vector<DataPoint>>> &data,
      GraphDataType data_type, int start_row, int start_col, int height,
      int width);

  /**
   * @brief Format number with unit
   * @param value Numeric value
   * @param unit Unit string
   * @return Formatted string
   */
  std::string format_number(double value, const std::string &unit) const;

  /**
   * @brief Format time in microseconds
   * @param us Time in microseconds
   * @return Formatted time string
   */
  std::string format_time(double us) const;

  /**
   * @brief Format bytes value
   * @param bytes Bytes value
   * @return Formatted bytes string
   */
  std::string format_bytes(int64_t bytes) const;

  /**
   * @brief Get column name
   * @param col Column enum
   * @return Column name string
   */
  std::string get_column_name(SortColumn col) const;

  /**
   * @brief Get tab name
   * @param tab Tab enum
   * @return Tab name string
   */
  std::string get_tab_name(Tab tab) const;

  /**
   * @brief Get data type name
   * @param type Data type enum
   * @return Data type name string
   */
  std::string get_data_type_name(GraphDataType type) const;

  /**
   * @brief Get value from data point by type
   * @param dp DataPoint
   * @param type Data type enum
   * @return Value as double
   */
  double get_value_by_type(const DataPoint &dp, GraphDataType type) const;

  /**
   * @brief Handle mouse input
   * @param data_manager Reference to the DataManager
   * @param event Mouse event
   * @return True if handled, false otherwise
   */
  bool handle_mouse_input(DataManager &data_manager, MEVENT &event);

  /**
   * @brief Get tab at x position
   * @param x X position
   * @return Tab enum
   */
  Tab get_tab_at_position(int x) const;

  /**
   * @brief Get column at x position for sorting
   * @param x X position
   * @return SortColumn enum
   */
  SortColumn get_column_at_position(int x) const;

  /**
   * @brief Cycle to next tab
   */
  void next_tab();

  /**
   * @brief Cycle to previous tab
   */
  void prev_tab();

  /**
   * @brief Move selection up
   */
  void move_up();

  /**
   * @brief Move selection down
   */
  void move_down();

  /**
   * @brief Page up
   */
  void page_up();

  /**
   * @brief Page down
   */
  void page_down();

  /**
   * @brief Move to first row
   */
  void move_to_first();

  /**
   * @brief Move to last row
   * @param total_rows
   */
  void move_to_last(int total_rows);

  /**
   * @brief Cycle sort column
   */
  void cycle_sort_column();

  /**
   * @brief Toggle sort order
   */
  void toggle_sort_order();

  /**
   * @brief Set sort column directly
   * @param col Column to set
   */
  void set_sort_column(SortColumn col);

  /**
   * @brief Select specific tab
   * @param tab Tab to select
   */
  void select_tab(Tab tab);

  /// @brief Flag indicating if ncurses is initialized
  bool initialized_;
  /// @brief Flag indicating if mouse support is enabled
  bool mouse_enabled_;
  /// @brief Current active tab
  Tab current_tab_;
  /// @brief Current sort column
  SortColumn sort_column_;
  /// @brief Sort order flag
  bool sort_ascending_;
  /// @brief Currently selected row in the table
  int selected_row_;
  /// @brief Scroll offset for table view
  int scroll_offset_;
  /// @brief Scroll offset for graph view
  int graph_scroll_offset_;
  /// @brief Maximum number of rows that can be displayed
  int max_rows_;
  /// @brief Terminal dimensions
  int terminal_width_;
  /// @brief Terminal dimensions
  int terminal_height_;

  /// @brief Tab positions for mouse click detection
  std::map<Tab, std::pair<int, int>> tab_positions_; // Tab -> (start_x, end_x)

  /// @brief Column positions for mouse click detection
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
