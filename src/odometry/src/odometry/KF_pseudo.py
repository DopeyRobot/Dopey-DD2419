#This will be have its own package

#KF
#Predict
#   Use odometry 
#   Keep publishing dynamic tf between odom and map (that actually is static here)

#Update
#   Given an aruco marker pose and covariance
#   Update the position, the difference/jump should be published as the new dynamic tf between map and odom
