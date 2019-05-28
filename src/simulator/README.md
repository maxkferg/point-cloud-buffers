# Point Simulator
Accumulates points from a stream

## PointBuffer()

### get_frame()
Return the points that were just collected from the sensor

### get_buffer_points()
Return points that are held in the point buffer.
Some of these points may have expired.

### get_valid_points()
Return the buffer indices of points which are still valid (represent truth)

### get_expired_points
Return the buffer indices of points which are invalid () have been removed