uint8 BOOTING=0
uint8 WAITING_FOR_SIM=1
uint8 WAITING_FOR_PC_B=2
uint8 READY=3
uint8 RUNNING=4
uint8 STOPPING=5
uint8 STOPPED=6
uint8 RESETTING=7
uint8 ERROR=8
uint8 EMERGENCY_STOPPED=9

std_msgs/Header header

string run_id
uint8 state
string phase
string active_job_id
float32 progress

bool simulation_ready
bool pc_a_ready
bool pc_b_ready
bool navigation_ready
bool manipulation_ready
bool perception_ready

int32 error_code
string message
builtin_interfaces/Time heartbeat_time