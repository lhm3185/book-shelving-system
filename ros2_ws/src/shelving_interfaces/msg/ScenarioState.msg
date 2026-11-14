uint8 UNKNOWN=0
uint8 STOPPED=1
uint8 RESETTING=2
uint8 READY=3
uint8 RUNNING=4
uint8 FAILED=5

std_msgs/Header header
string run_id
uint8 state
string message
