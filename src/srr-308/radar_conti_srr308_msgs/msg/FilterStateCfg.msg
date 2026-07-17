std_msgs/Header header

# Bit i is set after a Cluster FilterState_Cfg reply for zero-based index i
# has been received. Entries whose bits are clear have not been observed.
uint16 valid_mask

FilterStateCfgEntryInt nofobj
FilterStateCfgEntryFloat distance
FilterStateCfgEntryFloat azimuth
FilterStateCfgEntryFloat vreloncome
FilterStateCfgEntryFloat vreldepart
FilterStateCfgEntryFloat rcs
FilterStateCfgEntryFloat lifetime
FilterStateCfgEntryFloat size
FilterStateCfgEntryInt probexists
FilterStateCfgEntryFloat y
FilterStateCfgEntryFloat x
FilterStateCfgEntryFloat vyrightleft
FilterStateCfgEntryFloat vxoncome
FilterStateCfgEntryFloat vyleftright
FilterStateCfgEntryFloat vxdepart
