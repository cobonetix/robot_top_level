# Cobo Arm Controller - Improvement Suggestions

## Bugs to Fix Now

### 1. `tower_reset()` sends calibrate command instead of reset — arduino_comms.cpp:246

```cpp
void ArduinoComms::tower_reset()
{
    // ...
    ss << "c";  // BUG: sends calibrate, not reset
```

This sends `c` (calibrate) instead of `r` (reset). A user calling reset will get an unexpected full calibration sequence.

### 2. Double `\r` sent for arm status reads — arduino_comms.cpp:348

```cpp
std::string response = send_msg("s\r");
```

`send_msg()` already appends `\r` at line 98, so the Arduino receives `s\r\r`. The tower read at line 141 correctly sends just `"s"`. This could cause parsing failures depending on how the Arduino handles the extra CR.

### 3. Joint values used even when `sscanf` fails — arduino_comms.cpp:349-353

```cpp
const int ret = std::sscanf(response.c_str(), "%d%d%d%d%d%d%d%d%d%d", ...);
val_1 = degrees_to_radians(j1);  // Used unconditionally!
val_2 = degrees_to_radians(j2);
val_3 = degrees_to_radians(j3);
```

If `sscanf` fails (timeout, partial response, `*` debug prefix like the tower check), `j1/j2/j3` contain uninitialized stack values that get written to joint positions. The tower read correctly checks `ret == 8` before using values, but the arm read doesn't.

### 4. `export_state_interfaces` skips every other position joint — cobo_hardware.cpp:94-107

```cpp
int oddeven = 0;
for (const auto & joint_name : joint_interfaces["position"])
{
    if (oddeven++ & 1)  // Skips index 0, 2, 4, 6
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind]);
```

This `oddeven` filter also appears in `export_command_interfaces`. It looks like a workaround for duplicate entries from the URDF parsing, but it's fragile -- if the URDF changes or the duplication is fixed, half the joints silently stop working. At minimum this needs a comment explaining why, or better, fix the root cause of duplicate entries.

### 5. `on_activate` doesn't clean up left arm on right arm connect failure — cobo_hardware.cpp:229-235

When right arm connect fails, left arm is disconnected but its `set_left_arm()` was already sent. If reconnection is attempted, the left arm Arduino may be in an inconsistent state. Should also call `arm_lock(0)` or `arm_reset()` on cleanup.

---

## Reliability Improvements

### 6. Sequential blocking serial reads starve the control loop — cobo_hardware.cpp:272-308

At 3 Hz, you have ~333ms per cycle. Each `read_tower_info` / `read_arm_info` calls `send_msg` which does `FlushIOBuffers` + `Write` + `ReadLine` with a timeout. Three sequential calls means:
- If one Arduino is slow, the other two wait
- A single timeout burns the entire cycle budget

**Suggestion**: Read the three serial ports in parallel using `std::async` or a thread pool. Since each ArduinoComms instance has its own `SerialPort`, there's no contention:

```cpp
auto tower_future = std::async(std::launch::async, [&]() {
    TowerComms.read_tower_info(val1, val2, hw_states_);
});
auto left_future = std::async(std::launch::async, [&]() {
    ArmComms[LEFT_ARM_PORT].read_arm_info(...);
});
auto right_future = std::async(std::launch::async, [&]() {
    ArmComms[RIGHT_ARM_PORT].read_arm_info(...);
});
tower_future.get(); left_future.get(); right_future.get();
```

### 7. No error counting or recovery — arduino_comms.cpp:101-115

Timeouts and parse failures are logged but never acted on. If an Arduino disconnects or resets:
- `read()` silently returns stale data
- `write()` silently fails
- The system thinks everything is fine

**Suggestion**: Add consecutive error counting. After N failures, set `hw_states[ST_*_ERROR]` and optionally attempt reconnection:

```cpp
if (++consecutive_errors_ > MAX_CONSECUTIVE_ERRORS) {
    RCLCPP_ERROR(logger_, "Device unresponsive, attempting reconnect");
    disconnect();
    connect(device_path_, baud_rate_, timeout_ms_);
    consecutive_errors_ = 0;
}
```

### 8. `FlushIOBuffers()` discards data on every send — arduino_comms.cpp:97

Every call to `send_msg()` flushes both input and output buffers. If the Arduino sends unsolicited data (debug messages prefixed with `*`), this is necessary. But it also means that if `write()` sends multiple commands in one cycle (position + parameters), intermediate responses from the first command could be flushed before being read.

**Suggestion**: Only flush the input buffer, and only before status reads, not before every command:

```cpp
// In read_tower_info / read_arm_info:
serial_conn_.FlushInputBuffer();
// In command methods: don't flush
```

### 9. `send_msg` timeout defaults to 0 — arduino_comms.hpp:74

When `t_out = 0`, `ReadLine` uses 0ms timeout, which could fail immediately on slower responses. The `timeout_ms_` member is set during `connect()` but never used as the default -- it should be:

```cpp
std::string send_msg(const std::string &msg, bool print = false,
                     uint32_t t_out = 0);
// In implementation:
uint32_t effective_timeout = (t_out > 0) ? t_out : timeout_ms_;
serial_conn_.ReadLine(response, '\n', effective_timeout);
```

---

## Code Quality / Maintainability

### 10. Debug `std::cout` left in production IK code — ik_solver.cpp:86,140,169,196

```cpp
std::cout << "Distance: " << distance << ...
std::cout << "no solution " << cos_j2 << ...
std::cout << "j1/2 limits " << j1 << ...
std::cout << "Target pose unreachable" << ...
```

These bypass the ROS logging system entirely. Replace with `RCLCPP_DEBUG` or remove. Same issue in arduino_comms.cpp:193 (`std::cout << "TC "...`).

### 11. Hardcoded `if (1)` replacing logger checks — arduino_comms.cpp:240,265,275,482,495

```cpp
if (1) //logger_initialized_)
```

These were presumably changed during debugging. They cause unnecessary log spam. Revert to `if (logger_initialized_)`.

### 12. Hardcoded magic numbers for pi — ik_solver.cpp:27-40

```cpp
joint_limits_[...] = {1.58, 4.79};
joint_conversions_[...] = 1.57;
joint_conversions_[...] = 3.14;
```

Use `M_PI`, `M_PI_2`, etc. for clarity:
```cpp
joint_limits_[...] = {M_PI_2, 3.0 * M_PI_2};
joint_conversions_[...] = M_PI_2;
joint_conversions_[...] = M_PI;
```

### 13. Pose update only processes one arm per cycle — cobo_controller.cpp:364-378

```cpp
if (left_new_pose_msg_ || right_new_pose_msg_) {
    if (left_new_pose_msg_) {
        // process left
    } else {
        // process right
    }
```

If both arms receive pose commands simultaneously, the right arm waits until the next cycle (333ms). Process both in the same cycle:

```cpp
if (left_new_pose_msg_) {
    // process left
    left_new_pose_msg_ = false;
}
if (right_new_pose_msg_) {
    // process right
    right_new_pose_msg_ = false;
}
```

### 14. `left_new_pose_msg_` / `right_new_pose_msg_` are not atomic — cobo_controller.cpp:229,238

These booleans are set in subscription callbacks (different threads) and read in `update()`. Without `std::atomic<bool>`, this is a data race. The `RealtimeBuffer` handles the pose data safely, but the flag itself is unprotected.

### 15. The `Config.timeout_ms` is never initialized from parameters — cobo_hardware.hpp:90

```cpp
uint32_t timeout_ms = 0;
```

`on_init()` reads `baud_rate`, `initial_rotation`, etc. from hardware parameters, but never reads `timeout_ms`. It stays at 0 and gets passed to `connect()`, which passes it to `send_msg()` as the default. Add it to the URDF parameters and parse it, or set a sensible default (e.g., 200ms).

---

## Summary Priority Table

| Priority | Issue | Risk |
|---|---|---|
| **Fix now** | `tower_reset()` sends `c` not `r` | Wrong hardware action |
| **Fix now** | Arm read uses uninitialized values on parse fail | Random joint positions |
| **Fix now** | Double `\r` on arm reads | Intermittent parse failures |
| **High** | `timeout_ms` never set (stays 0) | All reads may timeout immediately |
| **High** | Parallel serial reads | 3x latency per cycle |
| **High** | No serial error recovery | Silent failure on disconnect |
| **Medium** | Pose processes only one arm per cycle | 333ms delay for dual-arm moves |
| **Medium** | Non-atomic pose flags | Data race |
| **Medium** | `oddeven` hack in export interfaces | Fragile, breaks on URDF change |
| **Low** | `std::cout` in IK solver | Bypasses ROS logging |
| **Low** | `if (1)` debug leftovers | Log spam |
| **Low** | Hardcoded pi values | Readability |
