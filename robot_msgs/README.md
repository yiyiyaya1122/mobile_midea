robot_msgs
=========

Canonical message definitions used across prototype and quest packages.

Included messages (organized by subsystem folders under msg/):
- quest/
  - Quest2Button
- yam/
  - YamCmd
  - YamStatus
- arx/
  - RobotCmd
  - RobotStatus
  - PosCmd
- galaxea/
  - MotorControl

Usage:
- Add an `<exec_depend>robot_msgs</exec_depend>` to your package.xml
- Import messages in Python with: `from robot_msgs.msg import <MessageName>`

Migration notes:
- All message packages from `quest/src/msg/` and `prototype/src/msg/` have been merged into this package.
- Older local message packages have been removed. Please update imports and dependencies to use this package.
- For C++: `#include "robot_msgs/msg/<message_name>.hpp"`
- For Python: `from robot_msgs.msg import <MessageName>`
