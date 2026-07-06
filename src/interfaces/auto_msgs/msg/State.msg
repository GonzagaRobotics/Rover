uint8 UNKNOWN            = 0 # Error state, should not happen
uint8 PLANNING           = 1 # Searching for a path to the target
uint8 TRAVELING          = 2 # Following the plan to get near the target
uint8 TERMINAL_SEARCHING = 3 # At the target area, searching for the target
uint8 TERMINAL_MOVING    = 4 # Found the target, moving to it
uint8 SUCCESS            = 5 # Stopped near the target
uint8 FAILURE            = 6 # Unable to reach the target

uint8 state