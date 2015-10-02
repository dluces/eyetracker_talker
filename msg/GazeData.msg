# Declare of the type of event
uint8 EVENT_NONE = 0
uint8 EVENT_BLINK_CUSTOM = 1
uint8 EVENT_BLINK_CUSTOM_2X = 2
uint8 EVENT_BLINK_CUSTOM_3X = 3
uint8 EVENT_BLINK_1S = 4
uint8 EVENT_BLINK_2S = 5

float32 timestamp
geometry_msgs/Point32 loc
uint8 event