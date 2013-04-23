# Single laser range reading from an active ranger.  NOTE: the sensor_msgs/Range.msg
# was not used because that message is for sensors with a field of view, as opposed
# to the laser range sensor which provides a point range.

Header header    	# timestamp in the header is the time the ranger
		 	# returned the distance reading

float32 range       # range value (position) [m]

