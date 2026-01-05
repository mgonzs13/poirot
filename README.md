# POIROT (PrOfIling RObotics Tool)

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

---

**POIROT** is a comprehensive profiling and monitoring framework for ROS 2 applications that provides detailed insights into function performance, resource consumption, energy usage and CO2 emissions. With its htop-like terminal interface, real-time data visualization and CSV recording capabilities, POIROT enables developers to optimize their robotic systems for performance and sustainability, acting as a keen investigator of code inefficiencies.

---

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Demos](#demos)
  - [Quick Start Demo](#quick-start-demo)
  - [Running with TUI](#running-with-tui)
  - [Recording Data to CSV](#recording-data-to-csv)

## Features

### Core Profiling Library (`poirot`)

- **Automatic System Configuration**: Detects CPU model, RAM, OS, energy measurement capabilities (Intel RAPL, AMD RAPL, hwmon) and downloads CO2 emission factors by country
- **Comprehensive Metrics Collection**:
  - **Timing**: Wall time and CPU time (user + system) per function
  - **Memory**: RSS and VSZ memory usage tracking
  - **I/O Operations**: Read/write bytes monitoring
  - **System Metrics**: Context switches (voluntary/involuntary), page faults (minor/major)
  - **Energy**: Real-time energy consumption in Joules (when hardware support available)
  - **Sustainability**: CO2 emissions estimation in grams based on regional energy mix
- **Low Overhead**: Efficient RAII-based profiling with minimal impact on application performance
- **ROS 2 Integration**: Publishes profiling data via `/poirot/data` topic using custom message types
- **Thread-Safe**: Concurrent profiling across multiple threads and nodes

### Terminal User Interface (`poirot_tui`)

- **htop-like Table View**: Real-time display of all profiled functions with sortable columns
  - PID, Node Name, Function Name, Call Count
  - Wall Time, CPU Time, CPU%, Memory Usage
  - I/O Read/Write, Context Switches
  - Energy Consumption, CO2 Emissions
- **Real-time Graphs**: Interactive time-series visualization for each metric
  - Wall Time, CPU Time, Memory, I/O (Read/Write)
  - Context Switches, Energy, CO2
  - Toggle individual functions on/off
  - Color-coded function traces with legend
- **Mouse & Keyboard Support**:
  - Click to navigate, sort, and select
  - Keyboard shortcuts for all operations
  - Scroll wheel navigation
- **Multiple Views**: Quick tab switching between table and graph views (9 tabs total)

### CSV Recorder (`poirot_recorder`)

- **Persistent Data Storage**: Records all profiling data to CSV files for offline analysis
- **Comprehensive Data Export**:
  - Function statistics (total and last call metrics)
  - Process information (PID, CPU%, memory, I/O, threads)
  - System information (OS, CPU model, cores, TDP, country code, CO2 factors)
- **Configurable Output**: CSV file path via ROS 2 parameter
- **Thread-Safe Writing**: Automatic flushing with proper CSV escaping
- **Progress Logging**: Status updates every 100 records

### Demo Applications (`poirot_demos`)

- **Example Publisher/Subscriber**: Demonstrates profiling in typical ROS 2 patterns
- **Timer Functions**: Shows profiling of periodic callbacks
- **Realistic Workloads**: Simulates CPU-intensive operations and memory allocation

### Message Definitions (`poirot_msgs`)

- Custom ROS 2 message types for profiling data:
  - `ProfilingData`: Complete profiling snapshot
  - `FunctionStats`: Aggregated function metrics
  - `FunctionCall`: Individual call data
  - `Data`: Detailed metrics (time, memory, I/O, energy, CO2)
  - `ProcessInfo`: Process-level information
  - `SystemInfo`: System-level configuration

## Installation

## Usage

1. **Add dependency in `package.xml`**:

   ```xml
   <depend>poirot</depend>
   ```

2. **Update `CMakeLists.txt`**:

   ```cmake
   find_package(poirot REQUIRED)

   add_executable(your_node src/your_node.cpp)
   target_link_libraries(your_node
     poirot::poirot
   )
   ```

3. **Instrument your code**:

   ```cpp
   #include "poirot/poirot.hpp"

   using namespace poirot;

   class MyNode : public rclcpp::Node {
   public:
     MyNode() : Node("my_node") {
       // Create timer, subscribers, etc.
       timer_ = create_wall_timer(
         std::chrono::seconds(1),
         std::bind(&MyNode::timer_callback, this));
     }

   private:
     void timer_callback() {
       PROFILE_FUNCTION();  // Automatically profiles this function

       // Your function logic here...
     }

     rclcpp::TimerBase::SharedPtr timer_;
   };

   int main(int argc, char* argv[]) {
     rclcpp::init(argc, argv);

     auto node = std::make_shared<MyNode>();

     // Enable ROS 2 publishing for TUI/recorder
     Poirot::enable_publishing(node);

     rclcpp::spin(node);
     Poirot::disable_publishing();

     rclcpp::shutdown();
     return 0;
   }
   ```

## Demos

### Quick Start Demo

The fastest way to see POIROT in action is to run the demo application with the TUI monitor:

**Terminal 1** - Run the demo node with profiling:

```bash
ros2 run poirot_demos demo_node
```

**Terminal 2** - Launch the TUI monitor:

```bash
ros2 run poirot_tui poirot_tui
```

You should see a live table displaying profiling metrics for the demo node's timer and subscription callbacks, including wall time, CPU usage, memory consumption, energy, and CO2 emissions.

### Running with TUI

The TUI provides multiple views and interactive controls:

#### TUI Controls

**General Navigation**:

- `Q` / `ESC` - Quit
- `Tab` / `→` - Next tab
- `Shift+Tab` / `←` - Previous tab
- `1-9` - Jump to specific tab
- `C` - Clear all data

**Table View**:

- `↑` / `K` / `↓` / `J` - Navigate rows
- `PgUp` / `PgDn` - Page up/down
- `Home` / `G` - First row
- `End` / `Shift+G` - Last row
- `S` - Cycle sort column
- `R` / `O` - Toggle sort order

**Graph Views**:

- `↑` / `K` / `↓` / `J` - Navigate function list
- `Space` / `Enter` - Toggle selected function in graph
- `A` - Enable all functions
- `N` - Disable all functions

#### Available Tabs

1. **Table** - Main table view with all profiling data
2. **Wall** - Wall time graph over time
3. **CPU** - CPU time graph over time
4. **Mem** - Memory usage graph over time
5. **IO-R** - I/O read bytes graph over time
6. **IO-W** - I/O write bytes graph over time
7. **Ctx** - Context switches graph over time
8. **Enrg** - Energy consumption graph over time
9. **CO2** - CO2 emissions graph over time

### Recording Data to CSV

To record profiling data for offline analysis:

**Terminal 1** - Run your ROS 2 application:

```bash
ros2 run poirot_demos demo_node
```

**Terminal 2** - Start the recorder:

```bash
ros2 run poirot_recorder poirot_recorder_node --ros-args -p csv_file_path:=/tmp/profiling_data.csv
```

The CSV file will contain comprehensive profiling data including timestamps, function names, call counts, timing metrics, memory usage, I/O statistics, energy consumption, CO2 emissions and system information.
