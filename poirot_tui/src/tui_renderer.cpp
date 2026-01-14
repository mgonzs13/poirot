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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "poirot_tui/tui_renderer.hpp"

using namespace poirot_tui;

TuiRenderer::TuiRenderer()
    : initialized_(false), mouse_enabled_(false), current_tab_(Tab::TABLE),
      sort_column_(SortColumn::WALL_TIME), sort_ascending_(false),
      selected_row_(0), scroll_offset_(0), graph_scroll_offset_(0),
      max_rows_(0), terminal_width_(0), terminal_height_(0) {}

TuiRenderer::~TuiRenderer() { this->shutdown(); }

bool TuiRenderer::initialize() {
  if (this->initialized_) {
    return true;
  }

  // Initialize ncurses
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);

  // Enable mouse support
  this->mouse_enabled_ =
      (mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, nullptr) != 0);
  if (this->mouse_enabled_) {
    mouseinterval(0); // No click delay
  }

  // Initialize colors if supported
  if (has_colors()) {
    start_color();
    use_default_colors();

    init_pair(COLOR_HEADER, COLOR_WHITE, COLOR_BLUE);
    init_pair(COLOR_SELECTED, COLOR_BLACK, COLOR_CYAN);
    init_pair(COLOR_NORMAL, -1, -1);
    init_pair(COLOR_TAB_ACTIVE, COLOR_WHITE, COLOR_BLUE);
    init_pair(COLOR_TAB_INACTIVE, COLOR_WHITE, COLOR_BLACK);
    init_pair(COLOR_GRAPH_1, COLOR_RED, -1);
    init_pair(COLOR_GRAPH_2, COLOR_GREEN, -1);
    init_pair(COLOR_GRAPH_3, COLOR_YELLOW, -1);
    init_pair(COLOR_GRAPH_4, COLOR_CYAN, -1);
    init_pair(COLOR_ENABLED, COLOR_GREEN, -1);
    init_pair(COLOR_DISABLED, COLOR_RED, -1);
    init_pair(COLOR_HELP_KEY, COLOR_YELLOW, COLOR_BLUE);
    init_pair(COLOR_GRAPH_5, COLOR_MAGENTA, -1);
    init_pair(COLOR_GRAPH_6, COLOR_WHITE, -1);
    init_pair(COLOR_GRAPH_7, COLOR_BLUE, -1);
    init_pair(COLOR_GRAPH_8, COLOR_RED, -1);
  }

  getmaxyx(stdscr, this->terminal_height_, this->terminal_width_);
  this->max_rows_ =
      this->terminal_height_ - 4; // Header + column headers + footer

  this->initialized_ = true;
  return true;
}

void TuiRenderer::shutdown() {
  if (this->initialized_) {
    endwin();
    this->initialized_ = false;
  }
}

void TuiRenderer::render(const DataManager &data_manager) {
  if (!this->initialized_) {
    return;
  }

  // Get terminal size
  getmaxyx(stdscr, this->terminal_height_, this->terminal_width_);
  this->max_rows_ = this->terminal_height_ - 4;

  // Erase screen (less flickering than clear())
  erase();

  // Render header
  this->render_header();

  // Render current view
  if (this->current_tab_ == Tab::TABLE) {
    this->render_table_view(data_manager);
  } else {
    GraphDataType data_type;
    switch (this->current_tab_) {
    case Tab::GRAPH_WALL_TIME:
      data_type = GraphDataType::WALL_TIME;
      break;
    case Tab::GRAPH_CPU_TIME:
      data_type = GraphDataType::CPU_TIME;
      break;
    case Tab::GRAPH_MEMORY:
      data_type = GraphDataType::MEMORY;
      break;
    case Tab::GRAPH_GPU_MEMORY:
      data_type = GraphDataType::GPU_MEMORY;
      break;
    case Tab::GRAPH_IO_READ:
      data_type = GraphDataType::IO_READ;
      break;
    case Tab::GRAPH_IO_WRITE:
      data_type = GraphDataType::IO_WRITE;
      break;
    case Tab::GRAPH_CTX_SWITCHES:
      data_type = GraphDataType::CTX_SWITCHES;
      break;
    case Tab::GRAPH_CPU_ENERGY:
      data_type = GraphDataType::CPU_ENERGY;
      break;
    case Tab::GRAPH_GPU_ENERGY:
      data_type = GraphDataType::GPU_ENERGY;
      break;
    case Tab::GRAPH_ENERGY:
      data_type = GraphDataType::ENERGY;
      break;
    case Tab::GRAPH_CO2:
      data_type = GraphDataType::CO2;
      break;
    default:
      data_type = GraphDataType::WALL_TIME;
      break;
    }
    this->render_graph_view(data_manager, data_type);
  }

  // Render footer
  this->render_footer();

  // Refresh screen
  refresh();
}

void TuiRenderer::render_header() {
  attron(COLOR_PAIR(COLOR_HEADER));
  mvhline(0, 0, ' ', this->terminal_width_);

  std::string title = "Poirot TUI - ROS 2 Profiling Monitor";
  if (this->mouse_enabled_) {
    title += " [Mouse: ON]";
  }
  mvprintw(0, (this->terminal_width_ - static_cast<int>(title.length())) / 2,
           "%s", title.c_str());

  attroff(COLOR_PAIR(COLOR_HEADER));

  // Render tabs and store positions for mouse click detection
  this->tab_positions_.clear();
  int tab_x = 0;
  std::vector<Tab> tabs = {Tab::TABLE,
                           Tab::GRAPH_WALL_TIME,
                           Tab::GRAPH_CPU_TIME,
                           Tab::GRAPH_MEMORY,
                           Tab::GRAPH_GPU_MEMORY,
                           Tab::GRAPH_IO_READ,
                           Tab::GRAPH_IO_WRITE,
                           Tab::GRAPH_CTX_SWITCHES,
                           Tab::GRAPH_CPU_ENERGY,
                           Tab::GRAPH_GPU_ENERGY,
                           Tab::GRAPH_ENERGY,
                           Tab::GRAPH_CO2};

  for (const auto &tab : tabs) {
    std::string tab_name = this->get_tab_name(tab);
    int tab_width = static_cast<int>(tab_name.length()) + 2;

    // Store tab position for mouse detection
    this->tab_positions_[tab] = {tab_x, tab_x + tab_width};

    if (tab == this->current_tab_) {
      attron(COLOR_PAIR(COLOR_TAB_ACTIVE) | A_BOLD);
    } else {
      attron(COLOR_PAIR(COLOR_TAB_INACTIVE));
    }
    mvprintw(1, tab_x, " %s ", tab_name.c_str());
    if (tab == this->current_tab_) {
      attroff(COLOR_PAIR(COLOR_TAB_ACTIVE) | A_BOLD);
    } else {
      attroff(COLOR_PAIR(COLOR_TAB_INACTIVE));
    }
    tab_x += tab_width;
  }
}

void TuiRenderer::render_table_view(const DataManager &data_manager) {
  auto rows =
      data_manager.get_sorted_rows(this->sort_column_, this->sort_ascending_);

  // Column header
  attron(COLOR_PAIR(COLOR_HEADER) | A_BOLD);
  mvhline(2, 0, ' ', this->terminal_width_);

  // Define column widths
  int col_pid = 15;
  int col_func = 43;
  int col_calls = 13;
  int col_wall = 17;
  int col_cpu = 17;
  int col_mem = 15;
  int col_gpu_mem = 15;
  int col_ior = 15;
  int col_iow = 15;
  int col_ctx = 13;
  int col_cpu_energy = 16;
  int col_gpu_energy = 16;
  int col_energy = 16;
  int col_co2 = 15;

  // Store column positions for mouse click sorting
  this->column_positions_.clear();
  int x = 0;

  auto print_header = [this, &x](const std::string &name, int width,
                                 SortColumn col) {
    // Store column position
    this->column_positions_.push_back({x, col});

    std::string indicator = "";
    if (this->sort_column_ == col) {
      indicator = this->sort_ascending_ ? " ^" : " v";
    }
    std::string header = name + indicator;
    if (static_cast<int>(header.length()) > width - 1) {
      header = header.substr(0, width - 1);
    }
    mvprintw(2, x, "%-*s", width, header.c_str());
    x += width;
  };

  print_header("PID", col_pid, SortColumn::PID);
  print_header("Function", col_func, SortColumn::FUNCTION_NAME);
  print_header("Calls", col_calls, SortColumn::CALL_COUNT);
  print_header("Wall(us)", col_wall, SortColumn::WALL_TIME);
  print_header("CPU(us)", col_cpu, SortColumn::CPU_TIME);
  print_header("Mem(KB)", col_mem, SortColumn::MEMORY);
  print_header("GPU-M(KB)", col_gpu_mem, SortColumn::GPU_MEMORY);
  print_header("IO-R(B)", col_ior, SortColumn::IO_READ);
  print_header("IO-W(B)", col_iow, SortColumn::IO_WRITE);
  print_header("CtxSw", col_ctx, SortColumn::CTX_SWITCHES);
  print_header("CPU-E(uJ)", col_cpu_energy, SortColumn::CPU_ENERGY);
  print_header("GPU-E(uJ)", col_gpu_energy, SortColumn::GPU_ENERGY);
  print_header("Enrg(uJ)", col_energy, SortColumn::ENERGY);
  print_header("CO2(ug)", col_co2, SortColumn::CO2);

  attroff(COLOR_PAIR(COLOR_HEADER) | A_BOLD);

  // Ensure scroll offset and selected row are valid
  int total_rows = static_cast<int>(rows.size());
  if (this->selected_row_ >= total_rows) {
    this->selected_row_ = std::max(0, total_rows - 1);
  }

  if (this->scroll_offset_ > this->selected_row_) {
    this->scroll_offset_ = this->selected_row_;
  }

  if (this->selected_row_ >= this->scroll_offset_ + this->max_rows_) {
    this->scroll_offset_ = this->selected_row_ - this->max_rows_ + 1;
  }

  // Render rows
  int row_y = 3;
  for (int i = this->scroll_offset_;
       i < total_rows && row_y < this->terminal_height_ - 1; ++i, ++row_y) {
    const auto &row = rows[i];

    if (i == this->selected_row_) {
      attron(COLOR_PAIR(COLOR_SELECTED) | A_BOLD);
    } else {
      attron(COLOR_PAIR(COLOR_NORMAL));
    }

    mvhline(row_y, 0, ' ', this->terminal_width_);

    x = 0;

    // PID
    mvprintw(row_y, x, "%-*d", col_pid, row.pid);
    x += col_pid;

    // Function name (truncate if needed)
    std::string func = row.function_name;
    if (func.empty()) {
      func = "<unknown>";
    }
    if (static_cast<int>(func.length()) > col_func - 1) {
      func = func.substr(0, col_func - 2) + "~";
    }
    mvprintw(row_y, x, "%-*s", col_func, func.c_str());
    x += col_func;

    // Call count
    mvprintw(row_y, x, "%-*d", col_calls, row.call_count);
    x += col_calls;

    // Wall time
    mvprintw(row_y, x, "%-*ld", col_wall, row.wall_time_us);
    x += col_wall;

    // CPU time
    mvprintw(row_y, x, "%-*ld", col_cpu, row.cpu_time_us);
    x += col_cpu;

    // Memory
    mvprintw(row_y, x, "%-*ld", col_mem, static_cast<long>(row.mem_kb));
    x += col_mem;

    // GPU Memory
    mvprintw(row_y, x, "%-*ld", col_gpu_mem, static_cast<long>(row.gpu_mem_kb));
    x += col_gpu_mem;

    // IO Read
    mvprintw(row_y, x, "%-*ld", col_ior, static_cast<long>(row.io_read_bytes));
    x += col_ior;

    // IO Write
    mvprintw(row_y, x, "%-*ld", col_iow, static_cast<long>(row.io_write_bytes));
    x += col_iow;

    // Context switches
    mvprintw(row_y, x, "%-*ld", col_ctx, static_cast<long>(row.ctx_switches));
    x += col_ctx;

    // CPU Energy
    mvprintw(row_y, x, "%-*.2f", col_cpu_energy, row.cpu_energy_uj);
    x += col_cpu_energy;

    // GPU Energy
    mvprintw(row_y, x, "%-*.2f", col_gpu_energy, row.gpu_energy_uj);
    x += col_gpu_energy;

    // Total Energy
    mvprintw(row_y, x, "%-*.2f", col_energy, row.energy_uj);
    x += col_energy;

    // CO2
    mvprintw(row_y, x, "%-*.2f", col_co2, row.co2_ug);

    if (i == this->selected_row_) {
      attroff(COLOR_PAIR(COLOR_SELECTED) | A_BOLD);
    } else {
      attroff(COLOR_PAIR(COLOR_NORMAL));
    }
  }

  // Show row count info
  if (total_rows > 0) {
    std::string info = "Row " + std::to_string(this->selected_row_ + 1) + "/" +
                       std::to_string(total_rows);
    attron(COLOR_PAIR(COLOR_HEADER));
    mvprintw(0, this->terminal_width_ - static_cast<int>(info.length()) - 2,
             "%s", info.c_str());
    attroff(COLOR_PAIR(COLOR_HEADER));
  }
}

void TuiRenderer::render_graph_view(const DataManager &data_manager,
                                    GraphDataType data_type) {
  // Split view: left side for function selector, right side for graph
  int selector_width = 60;
  int graph_start_col = selector_width + 1;
  int graph_width = this->terminal_width_ - graph_start_col - 1;
  int graph_height = this->terminal_height_ - 5;

  // Draw divider
  attron(COLOR_PAIR(COLOR_HEADER));

  for (int y = 2; y < this->terminal_height_ - 1; ++y) {
    mvaddch(y, selector_width, ACS_VLINE);
  }

  attroff(COLOR_PAIR(COLOR_HEADER));

  // Render function selector
  this->render_function_selector(data_manager, 2, selector_width);

  // Get enabled functions and their data
  auto enabled = data_manager.get_enabled_functions();
  std::vector<std::pair<std::string, std::vector<DataPoint>>> graph_data;

  for (const auto &func_key : enabled) {
    auto history = data_manager.get_history(func_key);
    if (!history.empty()) {
      graph_data.emplace_back(func_key, history);
    }
  }

  // Draw graph title with decorative box
  std::string graph_title = " " + this->get_data_type_name(data_type) + " ";
  int title_len = static_cast<int>(graph_title.length());
  int title_x = graph_start_col + (graph_width - title_len - 4) / 2;

  attron(COLOR_PAIR(COLOR_HEADER) | A_BOLD);
  mvaddch(2, title_x, ACS_ULCORNER);
  mvhline(2, title_x + 1, ACS_HLINE, title_len + 2);
  mvaddch(2, title_x + title_len + 3, ACS_URCORNER);
  mvaddch(3, title_x, ACS_VLINE);
  mvprintw(3, title_x + 1, " %s ", graph_title.c_str());
  mvaddch(3, title_x + title_len + 3, ACS_VLINE);
  mvaddch(4, title_x, ACS_LLCORNER);
  mvhline(4, title_x + 1, ACS_HLINE, title_len + 2);
  mvaddch(4, title_x + title_len + 3, ACS_LRCORNER);
  attroff(COLOR_PAIR(COLOR_HEADER) | A_BOLD);

  if (graph_data.empty()) {
    int center_y = this->terminal_height_ / 2;
    int center_x = graph_start_col + graph_width / 2;

    // Draw a nice info box
    std::string msg1 = "No functions selected";
    std::string msg2 = "Use Up/Down to navigate";
    std::string msg3 = "SPACE/Enter to toggle";
    std::string msg4 = "Or click with mouse";

    int box_width = 26;
    int box_height = 6;
    int box_x = center_x - box_width / 2;
    int box_y = center_y - box_height / 2;

    attron(COLOR_PAIR(COLOR_HEADER));
    // Draw box frame
    mvaddch(box_y, box_x, ACS_ULCORNER);
    mvhline(box_y, box_x + 1, ACS_HLINE, box_width - 2);
    mvaddch(box_y, box_x + box_width - 1, ACS_URCORNER);
    for (int i = 1; i < box_height - 1; ++i) {
      mvaddch(box_y + i, box_x, ACS_VLINE);
      mvhline(box_y + i, box_x + 1, ' ', box_width - 2);
      mvaddch(box_y + i, box_x + box_width - 1, ACS_VLINE);
    }
    mvaddch(box_y + box_height - 1, box_x, ACS_LLCORNER);
    mvhline(box_y + box_height - 1, box_x + 1, ACS_HLINE, box_width - 2);
    mvaddch(box_y + box_height - 1, box_x + box_width - 1, ACS_LRCORNER);
    attroff(COLOR_PAIR(COLOR_HEADER));

    // Draw messages
    attron(A_BOLD | COLOR_PAIR(COLOR_GRAPH_4));
    mvprintw(box_y + 1, center_x - static_cast<int>(msg1.length()) / 2, "%s",
             msg1.c_str());
    attroff(A_BOLD | COLOR_PAIR(COLOR_GRAPH_4));

    attron(A_DIM);
    mvprintw(box_y + 2, center_x - static_cast<int>(msg2.length()) / 2, "%s",
             msg2.c_str());
    mvprintw(box_y + 3, center_x - static_cast<int>(msg3.length()) / 2, "%s",
             msg3.c_str());
    mvprintw(box_y + 4, center_x - static_cast<int>(msg4.length()) / 2, "%s",
             msg4.c_str());
    attroff(A_DIM);
  } else {
    this->draw_graph(graph_data, data_type, 3, graph_start_col, graph_height,
                     graph_width);
  }
}

void TuiRenderer::render_function_selector(const DataManager &data_manager,
                                           int start_row, int width) {
  attron(A_BOLD | COLOR_PAIR(COLOR_HEADER));
  mvhline(start_row, 0, ' ', width - 1);
  mvprintw(start_row, 1, "Functions (Space/Enter/Click to toggle)");
  attroff(A_BOLD | COLOR_PAIR(COLOR_HEADER));

  auto keys = data_manager.get_all_function_keys();
  int total_funcs = static_cast<int>(keys.size());

  // Ensure scroll offset is valid
  int max_visible = this->terminal_height_ - start_row - 3;
  if (this->graph_scroll_offset_ > this->selected_row_) {
    this->graph_scroll_offset_ = this->selected_row_;
  }

  if (this->selected_row_ >= this->graph_scroll_offset_ + max_visible) {
    this->graph_scroll_offset_ = this->selected_row_ - max_visible + 1;
  }

  if (this->selected_row_ >= total_funcs) {
    this->selected_row_ = std::max(0, total_funcs - 1);
  }

  int row_y = start_row + 1;
  int color_idx = 0;
  std::vector<int> graph_colors = {COLOR_GRAPH_1, COLOR_GRAPH_2, COLOR_GRAPH_3,
                                   COLOR_GRAPH_4, COLOR_GRAPH_5, COLOR_GRAPH_6,
                                   COLOR_GRAPH_7, COLOR_GRAPH_8};

  for (int i = this->graph_scroll_offset_;
       i < total_funcs && row_y < this->terminal_height_ - 1; ++i, ++row_y) {
    const auto &key = keys[i];
    bool enabled = data_manager.is_function_enabled(key);

    // Extract function name from key (format: "pid|function_name")
    size_t last_pipe = key.rfind('|');
    std::string display_name =
        (last_pipe != std::string::npos) ? key.substr(last_pipe + 1) : key;

    if (static_cast<int>(display_name.length()) > width - 8) {
      display_name = display_name.substr(0, width - 9) + "~";
    }

    if (i == this->selected_row_) {
      attron(COLOR_PAIR(COLOR_SELECTED) | A_BOLD);
    }

    mvhline(row_y, 0, ' ', width - 1);

    // Show checkbox
    if (enabled) {
      attron(COLOR_PAIR(COLOR_ENABLED));
      mvprintw(row_y, 1, "[X]");
      attroff(COLOR_PAIR(COLOR_ENABLED));
    } else {
      attron(COLOR_PAIR(COLOR_DISABLED));
      mvprintw(row_y, 1, "[ ]");
      attroff(COLOR_PAIR(COLOR_DISABLED));
    }

    // Show color indicator if enabled
    if (enabled) {
      int color_pair = graph_colors[color_idx % graph_colors.size()];
      attron(COLOR_PAIR(color_pair) | A_BOLD);
      mvprintw(row_y, 5, "*");
      attroff(COLOR_PAIR(color_pair) | A_BOLD);
      color_idx++;
      mvprintw(row_y, 7, "%s", display_name.c_str());
    } else {
      mvprintw(row_y, 5, "o");
      attron(A_DIM);
      mvprintw(row_y, 7, "%s", display_name.c_str());
      attroff(A_DIM);
    }

    if (i == this->selected_row_) {
      attroff(COLOR_PAIR(COLOR_SELECTED) | A_BOLD);
    }
  }

  // Show scroll indicators
  if (this->graph_scroll_offset_ > 0) {
    attron(A_BOLD);
    mvprintw(start_row + 1, width - 3, "^");
    attroff(A_BOLD);
  }

  if (this->graph_scroll_offset_ + max_visible < total_funcs) {
    attron(A_BOLD);
    mvprintw(this->terminal_height_ - 2, width - 3, "v");
    attroff(A_BOLD);
  }
}

void TuiRenderer::draw_graph(
    const std::vector<std::pair<std::string, std::vector<DataPoint>>> &data,
    GraphDataType data_type, int start_row, int start_col, int height,
    int width) {
  if (data.empty() || height < 5 || width < 20) {
    return;
  }

  // Count total data points
  size_t total_points = 0;
  for (const auto &pair : data) {
    total_points += pair.second.size();
  }

  // If no data points, show animated waiting message
  if (total_points == 0) {
    int frame = static_cast<int>(time(nullptr)) % 4;
    std::string spinner = "◐◓◑◒";
    std::string dots[] = {"   ", ".  ", ".. ", "..."};
    std::string msg1 = "Awaiting data";
    std::string msg2 = dots[frame];
    int center_y = start_row + height / 2;
    int center_x =
        start_col + (width - static_cast<int>(msg1.length()) - 3) / 2;
    attron(A_BOLD | COLOR_PAIR(COLOR_GRAPH_4));
    mvprintw(center_y, center_x, "%s%s", msg1.c_str(), msg2.c_str());
    attroff(A_BOLD | COLOR_PAIR(COLOR_GRAPH_4));
    return;
  }

  // Find min/max values and time range
  double min_val = std::numeric_limits<double>::max();
  double max_val = std::numeric_limits<double>::lowest();
  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::lowest();

  for (const auto &pair : data) {
    for (const auto &dp : pair.second) {
      double val = this->get_value_by_type(dp, data_type);
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
      min_time = std::min(min_time, dp.timestamp);
      max_time = std::max(max_time, dp.timestamp);
    }
  }

  // Handle edge cases for value range
  if (min_val == max_val) {
    if (min_val == 0.0) {
      max_val = 1.0;
    } else {
      double range = std::abs(min_val) * 0.1;
      max_val = min_val + range;
      min_val = min_val - range;
    }
  }

  // Handle edge cases for time range
  if (max_time <= min_time) {
    max_time = min_time + 1.0;
  }

  // Ensure min_val is not negative for certain metrics
  if (min_val < 0.0 &&
      (data_type == GraphDataType::WALL_TIME ||
       data_type == GraphDataType::CPU_TIME ||
       data_type == GraphDataType::ENERGY || data_type == GraphDataType::CO2)) {
    min_val = 0.0;
  }

  // Leave space for Y axis labels and frame with comfortable margins
  int y_label_width = 12;
  int legend_height = 2;
  int left_margin = 3;   // Space between Y-axis labels and graph border
  int right_margin = 3;  // Space on the right side
  int top_margin = 5;    // Space at the top
  int bottom_margin = 3; // Space before X-axis labels

  int actual_graph_width =
      width - y_label_width - left_margin - right_margin - 2;
  int actual_graph_height =
      height - top_margin - bottom_margin - 5 - legend_height;

  if (actual_graph_width < 10 || actual_graph_height < 3) {
    return;
  }

  int graph_left = start_col + y_label_width + left_margin;
  int graph_right = graph_left + actual_graph_width - 1;
  int graph_top = start_row + top_margin;
  int graph_bottom = graph_top + actual_graph_height - 1;

  // Draw graph frame/border
  attron(COLOR_PAIR(COLOR_HEADER));

  // Top border
  mvaddch(graph_top - 1, graph_left - 1, ACS_ULCORNER);
  mvhline(graph_top - 1, graph_left, ACS_HLINE, actual_graph_width);
  mvaddch(graph_top - 1, graph_right + 1, ACS_URCORNER);

  // Left border (Y axis)
  for (int y = graph_top; y <= graph_bottom; ++y) {
    mvaddch(y, graph_left - 1, ACS_VLINE);
  }

  // Right border
  for (int y = graph_top; y <= graph_bottom; ++y) {
    mvaddch(y, graph_right + 1, ACS_VLINE);
  }

  // Bottom border (X axis)
  mvaddch(graph_bottom + 1, graph_left - 1, ACS_LLCORNER);
  mvhline(graph_bottom + 1, graph_left, ACS_HLINE, actual_graph_width);
  mvaddch(graph_bottom + 1, graph_right + 1, ACS_LRCORNER);
  attroff(COLOR_PAIR(COLOR_HEADER));

  // Draw horizontal grid lines (dotted style)
  int y_steps = std::min(5, actual_graph_height - 1);
  for (int i = 1; i < y_steps; ++i) {
    int y = graph_top + (actual_graph_height * i) / y_steps;
    attron(A_DIM);
    for (int x = graph_left; x <= graph_right; x += 2) {
      mvaddch(y, x, ACS_BULLET);
    }
    attroff(A_DIM);
  }

  // Draw vertical grid lines (dotted style)
  int x_steps = std::min(5, actual_graph_width / 10);
  for (int i = 1; i < x_steps; ++i) {
    int x = graph_left + (actual_graph_width * i) / x_steps;
    attron(A_DIM);
    for (int y = graph_top; y <= graph_bottom; y += 2) {
      mvaddch(y, x, ACS_BULLET);
    }
    attroff(A_DIM);
  }

  // Draw Y axis labels with tick marks
  for (int i = 0; i <= y_steps; ++i) {
    int y = graph_top + (actual_graph_height * i) / y_steps;
    double value = max_val - (max_val - min_val) * i / y_steps;

    // Format value smartly based on magnitude
    char val_buf[16];
    if (std::abs(value) >= 1000000.0) {
      snprintf(val_buf, sizeof(val_buf), "%8.1fM", value / 1000000.0);
    } else if (std::abs(value) >= 1000.0) {
      snprintf(val_buf, sizeof(val_buf), "%8.1fK", value / 1000.0);
    } else if (std::abs(value) < 0.01 && value != 0.0) {
      snprintf(val_buf, sizeof(val_buf), "%8.2e", value);
    } else {
      snprintf(val_buf, sizeof(val_buf), "%8.2f", value);
    }

    attron(COLOR_PAIR(COLOR_GRAPH_4));
    mvprintw(y, start_col, "%s", val_buf);
    attroff(COLOR_PAIR(COLOR_GRAPH_4));

    // Tick mark
    attron(COLOR_PAIR(COLOR_HEADER));
    mvaddch(y, graph_left - 1, ACS_LTEE);
    attroff(COLOR_PAIR(COLOR_HEADER));
  }

  // Draw X axis labels with tick marks
  int x_axis_y = graph_bottom + 2;
  double time_range = max_time - min_time;

  // Start time
  attron(COLOR_PAIR(COLOR_GRAPH_4));
  mvprintw(x_axis_y, graph_left, "%.2fs", min_time);

  // End time
  char max_time_buf[32];
  snprintf(max_time_buf, sizeof(max_time_buf), "%.2fs", max_time);
  int max_time_len = static_cast<int>(strlen(max_time_buf));
  mvprintw(x_axis_y, graph_right - max_time_len + 1, "%s", max_time_buf);

  // Middle time labels
  for (int i = 1; i < x_steps; ++i) {
    int x = graph_left + (actual_graph_width * i) / x_steps;
    double t = min_time + time_range * i / x_steps;
    char time_buf[16];
    snprintf(time_buf, sizeof(time_buf), "%.1fs", t);
    int label_len = static_cast<int>(strlen(time_buf));
    mvprintw(x_axis_y, x - label_len / 2, "%s", time_buf);

    // Tick mark
    attron(COLOR_PAIR(COLOR_HEADER));
    mvaddch(graph_bottom + 1, x, ACS_BTEE);
    attroff(COLOR_PAIR(COLOR_HEADER));
  }

  attroff(COLOR_PAIR(COLOR_GRAPH_4));

  // Draw info line with stats
  double avg_val = 0.0;
  for (const auto &pair : data) {
    for (const auto &dp : pair.second) {
      avg_val += this->get_value_by_type(dp, data_type);
    }
  }

  avg_val /= (total_points > 0 ? total_points : 1);

  char info_buf[128];
  snprintf(info_buf, sizeof(info_buf),
           "Range: %.2f - %.2f | Avg: %.2f | Samples: %zu", min_val, max_val,
           avg_val, total_points);
  attron(A_DIM);
  mvprintw(start_row, graph_left, "%s", info_buf);
  attroff(A_DIM);

  // Draw data points with improved line interpolation
  std::vector<int> graph_colors = {COLOR_GRAPH_1, COLOR_GRAPH_2, COLOR_GRAPH_3,
                                   COLOR_GRAPH_4, COLOR_GRAPH_5, COLOR_GRAPH_6,
                                   COLOR_GRAPH_7, COLOR_GRAPH_8};

  int color_idx = 0;
  double val_range = max_val - min_val;

  for (const auto &pair : data) {
    if (pair.second.empty()) {
      continue;
    }

    int color_pair = graph_colors[color_idx % graph_colors.size()];

    // Store all points for this series
    std::vector<std::pair<int, int>> points;

    for (const auto &dp : pair.second) {
      double val = this->get_value_by_type(dp, data_type);

      // Calculate position
      double time_ratio =
          (time_range > 0.0) ? (dp.timestamp - min_time) / time_range : 0.5;
      double val_ratio = (val_range > 0.0) ? (max_val - val) / val_range : 0.5;

      // Clamp ratios
      time_ratio = std::max(0.0, std::min(1.0, time_ratio));
      val_ratio = std::max(0.0, std::min(1.0, val_ratio));

      int px =
          graph_left + static_cast<int>(time_ratio * (actual_graph_width - 1));
      int py =
          graph_top + static_cast<int>(val_ratio * (actual_graph_height - 1));

      // Clamp to graph area
      px = std::max(graph_left, std::min(px, graph_right));
      py = std::max(graph_top, std::min(py, graph_bottom));

      points.emplace_back(px, py);
    }

    // Draw smooth lines between consecutive points using Bresenham algorithm
    attron(COLOR_PAIR(color_pair));
    for (size_t i = 1; i < points.size(); ++i) {
      int x0 = points[i - 1].first;
      int y0 = points[i - 1].second;
      int x1 = points[i].first;
      int y1 = points[i].second;

      // Bresenham's line algorithm for smooth lines
      int dx = std::abs(x1 - x0);
      int dy = -std::abs(y1 - y0);
      int sx = (x0 < x1) ? 1 : -1;
      int sy = (y0 < y1) ? 1 : -1;
      int err = dx + dy;

      int x = x0, y = y0;
      while (true) {
        // Skip the endpoints (we'll draw markers there)
        if (!((x == x0 && y == y0) || (x == x1 && y == y1))) {

          // Determine line character based on direction
          chtype line_char;
          if (dx == 0) {
            line_char = ACS_VLINE; // Vertical line: │

          } else if (dy == 0) {
            line_char = ACS_HLINE; // Horizontal line: ─

          } else {
            // Calculate local slope for character selection
            double local_slope = static_cast<double>(y1 - y0) / (x1 - x0);
            if (std::abs(local_slope) < 0.5) {
              line_char = ACS_HLINE; // Nearly horizontal: ─
            } else if (std::abs(local_slope) > 2.0) {
              line_char = ACS_VLINE; // Nearly vertical: │
            } else if (local_slope > 0) {
              line_char = '\\'; // Descending (in screen coords)
            } else {
              line_char = '/'; // Ascending (in screen coords)
            }
          }

          if (x >= graph_left && x <= graph_right && y >= graph_top &&
              y <= graph_bottom) {
            mvaddch(y, x, line_char);
          }
        }

        if (x == x1 && y == y1)
          break;
        int e2 = 2 * err;
        if (e2 >= dy) {
          err += dy;
          x += sx;
        }
        if (e2 <= dx) {
          err += dx;
          y += sy;
        }
      }
    }
    attroff(COLOR_PAIR(color_pair));

    // Draw data point markers on top of lines (more visible)
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    for (size_t i = 0; i < points.size(); ++i) {
      int px = points[i].first;
      int py = points[i].second;

      // Use different marker styles for different series
      chtype marker;
      switch (color_idx % 4) {
      case 0:
        marker = ACS_DIAMOND;
        break; // ◆
      case 1:
        marker = ACS_BULLET;
        break; // •
      case 2:
        marker = ACS_DEGREE;
        break; // °
      default:
        marker = '*';
        break;
      }
      mvaddch(py, px, marker);
    }
    attroff(COLOR_PAIR(color_pair) | A_BOLD);

    color_idx++;
  }

  // Draw enhanced legend box at bottom
  int legend_y = x_axis_y + 1;
  int legend_start_x = graph_left;

  // Legend title
  attron(A_BOLD | A_UNDERLINE);
  mvprintw(legend_y, legend_start_x, "Legend:");
  attroff(A_BOLD | A_UNDERLINE);

  int legend_x = legend_start_x + 9;
  color_idx = 0;

  for (const auto &pair : data) {
    if (legend_x >= graph_right - 12) {
      // Show ellipsis if more items
      attron(A_DIM);
      mvprintw(legend_y, legend_x, "...");
      attroff(A_DIM);
      break;
    }

    // Extract function name from key
    size_t last_pipe = pair.first.rfind('|');
    std::string name = (last_pipe != std::string::npos)
                           ? pair.first.substr(last_pipe + 1)
                           : pair.first;
    if (name.length() > 12) {
      name = name.substr(0, 11) + "~";
    }

    int color_pair = graph_colors[color_idx % graph_colors.size()];

    // Draw colored marker
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    mvaddch(legend_y, legend_x, ACS_DIAMOND);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);

    // Draw name
    mvprintw(legend_y, legend_x + 1, " %s", name.c_str());

    legend_x += static_cast<int>(name.length()) + 4;
    color_idx++;
  }
}

void TuiRenderer::render_footer() {
  attron(COLOR_PAIR(COLOR_HEADER));
  mvhline(this->terminal_height_ - 1, 0, ' ', this->terminal_width_);

  // Create help text with highlighted keys
  int x = 1;

  auto print_key = [this, &x](const std::string &key, const std::string &desc) {
    attron(COLOR_PAIR(COLOR_HELP_KEY) | A_BOLD);
    mvprintw(this->terminal_height_ - 1, x, "%s", key.c_str());
    attroff(COLOR_PAIR(COLOR_HELP_KEY) | A_BOLD);
    x += static_cast<int>(key.length());

    attron(COLOR_PAIR(COLOR_HEADER));
    mvprintw(this->terminal_height_ - 1, x, ":%s ", desc.c_str());
    x += static_cast<int>(desc.length()) + 2;
  };

  print_key("Q", "Quit");
  print_key("Tab", "Next Tab");

  if (this->current_tab_ == Tab::TABLE) {
    print_key("Up/Dn", "Navigate");
    print_key("Home/End", "First/Last");
    print_key("PgUp/Dn", "Page");
    print_key("S", "Sort");
    print_key("R", "Reverse");
    print_key("C", "Clear");
  } else {
    print_key("Up/Dn", "Select");
    print_key("Space/Enter", "Toggle");
    print_key("A", "All");
    print_key("N", "None");
  }

  if (this->mouse_enabled_) {
    print_key("Mouse", "Click");
  }

  attroff(COLOR_PAIR(COLOR_HEADER));
}

bool TuiRenderer::handle_input(DataManager &data_manager) {
  int ch = getch();
  if (ch == ERR) {
    return true; // No input, continue
  }

  // Handle mouse events
  if (ch == KEY_MOUSE) {
    MEVENT event;
    if (getmouse(&event) == OK) {
      return this->handle_mouse_input(data_manager, event);
    }
    return true;
  }

  switch (ch) {
  case 'q':
  case 'Q':
  case 27: // ESC
    return false;

  case '\t':
  case KEY_RIGHT:
    this->next_tab();
    break;

  case KEY_BTAB: // Shift+Tab
  case KEY_LEFT:
    this->prev_tab();
    break;

  case KEY_UP:
  case 'k':
  case 'K':
    this->move_up();
    break;

  case KEY_DOWN:
  case 'j':
  case 'J':
    this->move_down();
    break;

  case KEY_PPAGE:
    this->page_up();
    break;

  case KEY_NPAGE:
    this->page_down();
    break;

  case KEY_HOME:
  case 'g':
    this->move_to_first();
    break;

  case KEY_END:
  case 'G': {
    auto rows =
        data_manager.get_sorted_rows(this->sort_column_, this->sort_ascending_);
    this->move_to_last(static_cast<int>(rows.size()));
  } break;

  case 's':
  case 'S':
    if (this->current_tab_ == Tab::TABLE) {
      this->cycle_sort_column();
    }
    break;

  case 'r':
  case 'R':
  case 'o':
  case 'O':
    if (this->current_tab_ == Tab::TABLE) {
      this->toggle_sort_order();
    }
    break;

  case ' ':
  case '\n':
  case KEY_ENTER:
    if (this->current_tab_ != Tab::TABLE) {
      // Toggle function in graph view
      auto keys = data_manager.get_all_function_keys();
      if (this->selected_row_ >= 0 &&
          this->selected_row_ < static_cast<int>(keys.size())) {
        data_manager.toggle_function(keys[this->selected_row_]);
      }
    }
    break;

  case 'a':
  case 'A':
    if (this->current_tab_ != Tab::TABLE) {
      // Enable all functions
      auto keys = data_manager.get_all_function_keys();
      for (const auto &key : keys) {
        data_manager.enable_function(key);
      }
    }
    break;

  case 'n':
  case 'N':
    if (this->current_tab_ != Tab::TABLE) {
      // Disable all functions
      auto keys = data_manager.get_all_function_keys();
      for (const auto &key : keys) {
        data_manager.disable_function(key);
      }
    }
    break;

  case 'c':
  case 'C':
    data_manager.clear();
    this->selected_row_ = 0;
    this->scroll_offset_ = 0;
    this->graph_scroll_offset_ = 0;
    break;

  // Number keys for quick tab switching
  case '1':
    this->select_tab(Tab::TABLE);
    break;
  case '2':
    this->select_tab(Tab::GRAPH_WALL_TIME);
    break;
  case '3':
    this->select_tab(Tab::GRAPH_CPU_TIME);
    break;
  case '4':
    this->select_tab(Tab::GRAPH_MEMORY);
    break;
  case '5':
    this->select_tab(Tab::GRAPH_GPU_MEMORY);
    break;
  case '6':
    this->select_tab(Tab::GRAPH_IO_READ);
    break;
  case '7':
    this->select_tab(Tab::GRAPH_IO_WRITE);
    break;
  case '8':
    this->select_tab(Tab::GRAPH_CTX_SWITCHES);
    break;
  case '9':
    this->select_tab(Tab::GRAPH_ENERGY);
    break;
  case '0':
    this->select_tab(Tab::GRAPH_CO2);
    break;

  default:
    break;
  }

  return true;
}

bool TuiRenderer::handle_mouse_input(DataManager &data_manager, MEVENT &event) {
  int mx = event.x;
  int my = event.y;

  if (event.bstate & BUTTON1_CLICKED || event.bstate & BUTTON1_PRESSED) {
    // Check if clicked on tabs (row 1)
    if (my == 1) {
      Tab clicked_tab = this->get_tab_at_position(mx);
      if (clicked_tab != this->current_tab_) {
        this->select_tab(clicked_tab);
      }
      return true;
    }

    // Check if clicked on column headers (row 2) in table view
    if (my == 2 && this->current_tab_ == Tab::TABLE) {
      SortColumn clicked_col = this->get_column_at_position(mx);
      if (clicked_col == this->sort_column_) {
        this->toggle_sort_order();
      } else {
        this->set_sort_column(clicked_col);
      }
      return true;
    }

    // Check if clicked on a row in table view or function selector
    if (my >= 3 && my < this->terminal_height_ - 1) {
      if (this->current_tab_ == Tab::TABLE) {
        // Clicked on a data row
        int clicked_row = this->scroll_offset_ + (my - 3);
        auto rows = data_manager.get_sorted_rows(this->sort_column_,
                                                 this->sort_ascending_);
        if (clicked_row >= 0 && clicked_row < static_cast<int>(rows.size())) {
          this->selected_row_ = clicked_row;
        }

      } else {
        // In graph view - check if clicked in function selector area (left
        // side)
        if (mx < 60) {
          int clicked_row = this->graph_scroll_offset_ + (my - 3);
          auto keys = data_manager.get_all_function_keys();
          if (clicked_row >= 0 && clicked_row < static_cast<int>(keys.size())) {
            this->selected_row_ = clicked_row;
            // Toggle the function
            data_manager.toggle_function(keys[clicked_row]);
          }
        }
      }

      return true;
    }
  }

  // Handle scroll wheel
  if (event.bstate & BUTTON4_PRESSED) { // Scroll up
    this->move_up();
    this->move_up();
    this->move_up();
    return true;
  }

  if (event.bstate & BUTTON5_PRESSED) { // Scroll down
    this->move_down();
    this->move_down();
    this->move_down();
    return true;
  }

  return true;
}

Tab TuiRenderer::get_tab_at_position(int x) const {
  for (const auto &entry : this->tab_positions_) {
    if (x >= entry.second.first && x < entry.second.second) {
      return entry.first;
    }
  }
  return this->current_tab_;
}

SortColumn TuiRenderer::get_column_at_position(int x) const {
  SortColumn result = this->sort_column_;
  for (size_t i = 0; i < this->column_positions_.size(); ++i) {
    int start_x = this->column_positions_[i].first;
    int end_x = (i + 1 < this->column_positions_.size())
                    ? this->column_positions_[i + 1].first
                    : this->terminal_width_;
    if (x >= start_x && x < end_x) {
      result = this->column_positions_[i].second;
      break;
    }
  }
  return result;
}

std::string TuiRenderer::get_tab_name(Tab tab) const {
  switch (tab) {
  case Tab::TABLE:
    return "1:Table";
  case Tab::GRAPH_WALL_TIME:
    return "2:Wall";
  case Tab::GRAPH_CPU_TIME:
    return "3:CPU";
  case Tab::GRAPH_MEMORY:
    return "4:Mem";
  case Tab::GRAPH_GPU_MEMORY:
    return "5:GPU-M";
  case Tab::GRAPH_IO_READ:
    return "6:IO-R";
  case Tab::GRAPH_IO_WRITE:
    return "7:IO-W";
  case Tab::GRAPH_CTX_SWITCHES:
    return "8:Ctx";
  case Tab::GRAPH_CPU_ENERGY:
    return "CPU-E";
  case Tab::GRAPH_GPU_ENERGY:
    return "GPU-E";
  case Tab::GRAPH_ENERGY:
    return "9:Enrg";
  case Tab::GRAPH_CO2:
    return "0:CO2";
  default:
    return "?";
  }
}

std::string TuiRenderer::get_data_type_name(GraphDataType type) const {
  switch (type) {
  case GraphDataType::WALL_TIME:
    return "Wall Time (us)";
  case GraphDataType::CPU_TIME:
    return "CPU Time (us)";
  case GraphDataType::MEMORY:
    return "Memory (KB)";
  case GraphDataType::GPU_MEMORY:
    return "GPU Memory (KB)";
  case GraphDataType::IO_READ:
    return "IO Read (bytes)";
  case GraphDataType::IO_WRITE:
    return "IO Write (bytes)";
  case GraphDataType::CTX_SWITCHES:
    return "Context Switches";
  case GraphDataType::CPU_ENERGY:
    return "CPU Energy (uJ)";
  case GraphDataType::GPU_ENERGY:
    return "GPU Energy (uJ)";
  case GraphDataType::ENERGY:
    return "Energy (uJ)";
  case GraphDataType::CO2:
    return "CO2 (ug)";
  default:
    return "Unknown";
  }
}

double TuiRenderer::get_value_by_type(const DataPoint &dp,
                                      GraphDataType type) const {
  switch (type) {
  case GraphDataType::WALL_TIME:
    return dp.wall_time_us;
  case GraphDataType::CPU_TIME:
    return dp.cpu_time_us;
  case GraphDataType::MEMORY:
    return static_cast<double>(dp.mem_kb);
  case GraphDataType::GPU_MEMORY:
    return static_cast<double>(dp.gpu_mem_kb);
  case GraphDataType::IO_READ:
    return static_cast<double>(dp.io_read_bytes);
  case GraphDataType::IO_WRITE:
    return static_cast<double>(dp.io_write_bytes);
  case GraphDataType::CTX_SWITCHES:
    return static_cast<double>(dp.ctx_switches);
  case GraphDataType::CPU_ENERGY:
    return dp.cpu_energy_uj;
  case GraphDataType::GPU_ENERGY:
    return dp.gpu_energy_uj;
  case GraphDataType::ENERGY:
    return dp.energy_uj;
  case GraphDataType::CO2:
    return dp.co2_ug;
  default:
    return 0.0;
  }
}

std::string TuiRenderer::get_column_name(SortColumn col) const {
  switch (col) {
  case SortColumn::PID:
    return "PID";
  case SortColumn::FUNCTION_NAME:
    return "Function";
  case SortColumn::CALL_COUNT:
    return "Calls";
  case SortColumn::WALL_TIME:
    return "Wall Time";
  case SortColumn::CPU_TIME:
    return "CPU Time";
  case SortColumn::MEMORY:
    return "Memory";
  case SortColumn::GPU_MEMORY:
    return "GPU Memory";
  case SortColumn::IO_READ:
    return "IO Read";
  case SortColumn::IO_WRITE:
    return "IO Write";
  case SortColumn::CTX_SWITCHES:
    return "Ctx Switches";
  case SortColumn::CPU_ENERGY:
    return "CPU Energy";
  case SortColumn::GPU_ENERGY:
    return "GPU Energy";
  case SortColumn::ENERGY:
    return "Energy";
  case SortColumn::CO2:
    return "CO2";
  default:
    return "Unknown";
  }
}

std::string TuiRenderer::format_number(double value,
                                       const std::string &unit) const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << value << " " << unit;
  return oss.str();
}

std::string TuiRenderer::format_time(double us) const {
  if (us >= 1000000.0) {
    return this->format_number(us / 1000000.0, "s");
  } else if (us >= 1000.0) {
    return this->format_number(us / 1000.0, "ms");
  } else {
    return this->format_number(us, "us");
  }
}

std::string TuiRenderer::format_bytes(int64_t bytes) const {
  if (bytes >= 1073741824) {
    return this->format_number(static_cast<double>(bytes) / 1073741824.0, "GB");
  } else if (bytes >= 1048576) {
    return this->format_number(static_cast<double>(bytes) / 1048576.0, "MB");
  } else if (bytes >= 1024) {
    return this->format_number(static_cast<double>(bytes) / 1024.0, "KB");
  } else {
    return this->format_number(static_cast<double>(bytes), "B");
  }
}

void TuiRenderer::next_tab() {
  int tab_int = static_cast<int>(this->current_tab_);
  tab_int = (tab_int + 1) % 12; // 12 tabs total
  this->current_tab_ = static_cast<Tab>(tab_int);
  this->selected_row_ = 0;
  this->scroll_offset_ = 0;
  this->graph_scroll_offset_ = 0;
}

void TuiRenderer::prev_tab() {
  int tab_int = static_cast<int>(this->current_tab_);
  tab_int = (tab_int - 1 + 12) % 12; // 12 tabs total
  this->current_tab_ = static_cast<Tab>(tab_int);
  this->selected_row_ = 0;
  this->scroll_offset_ = 0;
  this->graph_scroll_offset_ = 0;
}

void TuiRenderer::select_tab(Tab tab) {
  if (this->current_tab_ != tab) {
    this->current_tab_ = tab;
    this->selected_row_ = 0;
    this->scroll_offset_ = 0;
    this->graph_scroll_offset_ = 0;
  }
}

void TuiRenderer::move_up() {
  if (this->selected_row_ > 0) {
    this->selected_row_--;
  }
}

void TuiRenderer::move_down() { this->selected_row_++; }

void TuiRenderer::page_up() {
  this->selected_row_ = std::max(0, this->selected_row_ - this->max_rows_);
}

void TuiRenderer::page_down() { this->selected_row_ += this->max_rows_; }

void TuiRenderer::move_to_first() {
  this->selected_row_ = 0;
  this->scroll_offset_ = 0;
  this->graph_scroll_offset_ = 0;
}

void TuiRenderer::move_to_last(int total_rows) {
  if (total_rows > 0) {
    this->selected_row_ = total_rows - 1;
  }
}

void TuiRenderer::cycle_sort_column() {
  int col_int = static_cast<int>(this->sort_column_);
  col_int = (col_int + 1) % 16; // 16 columns
  this->sort_column_ = static_cast<SortColumn>(col_int);
}

void TuiRenderer::set_sort_column(SortColumn col) { this->sort_column_ = col; }

void TuiRenderer::toggle_sort_order() {
  this->sort_ascending_ = !this->sort_ascending_;
}
