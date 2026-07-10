uint8 TYPE_GNSS   = 0 # High precision GNSS coordinates
uint8 TYPE_ARUCO  = 1 # Post marked with ARUCO tags
uint8 TYPE_OBJECT = 2 # Physical objects needing detection

uint8 ID_BOTTLE = 0 # Water bottle
uint8 ID_MALLET = 1 # Rubber mallet
uint8 ID_HAMMER = 2 # Rock hammer

Location location
uint8 type
# ArUco ID, or the id of the object to find. Does nothing for GNSS
uint8 id