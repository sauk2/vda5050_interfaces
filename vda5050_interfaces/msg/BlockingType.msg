# Regulates if the action is allowed to be executed during movement and/or
# parallel to other actions.

# Enum values for blockingType
#
# NONE: action can happen in parallel with others, including movement
string BLOCKING_TYPE_NONE="NONE"
#
# SOFT: action can happen simultaneously with other actions, but not while moving
string BLOCKING_TYPE_SOFT="SOFT"
#
# HARD: no other action can be performed while this action is running
string BLOCKING_TYPE_HARD="HARD"
