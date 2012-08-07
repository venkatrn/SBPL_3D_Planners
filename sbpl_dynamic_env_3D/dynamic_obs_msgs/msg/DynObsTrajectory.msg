#a dynamic obstacle trajectory

#the probability of the obstacle following this trajectory
float64 probability

#whether this obstacle should be treated like a static obstacle at the end of its trajectory (true) 
#or the obstacle is ignored after the end of its trajectory (false)
bool exists_after

#the time parameterized path
geometry_msgs/PoseWithCovarianceStamped[] points
