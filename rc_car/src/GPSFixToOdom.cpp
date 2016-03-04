/* Message de type nav_msgs::Odometry à publier dans /odom
 msg.header.stamp = gps_time                   // time of gps measurement
 msg.header.frame_id = base_footprint          // the tracked robot frame
 msg.pose.pose.position.x = gps_x              // x measurement GPS.
 msg.pose.pose.position.y = gps_y              // y measurement GPS.
 msg.pose.pose.position.z = gps_z              // z measurement GPS.
 msg.pose.pose.orientation.x = 1               // identity quaternion
 msg.pose.pose.orientation.y = 0               // identity quaternion
 msg.pose.pose.orientation.z = 0               // identity quaternion
 msg.pose.pose.orientation.w = 0               // identity quaternion
 msg.pose.covariance = {cox_x, 0, 0, 0, 0, 0,  // covariance on gps_x
                        0, cov_y, 0, 0, 0, 0,  // covariance on gps_y
                        0, 0, 99999, 0, 0, 0,  // covariance on gps_z
                        0, 0, 0, 99999, 0, 0,  // large covariance on rot x
                        0, 0, 0, 0, 99999, 0,  // large covariance on rot y
                        0, 0, 0, 0, 0, 99999}  // large covariance on rot z
*/
/* à ajouter au .launch pour lancer l'EKF
<node pkg=”robot_pose_ekf” type=”robot_pose_ekf” name=”robot_pose_ekf”>
<param name=”output_frame” value=”odom”/>
<param name=”freq” value=”30.0″/>
<param name=”sensor_timeout” value=”2.5″/>
<param name=”odom_used” value=”true”/>
<param name=”imu_used” value=”true”/>
<param name=”vo_used” value=”false”/>
<param name=”debug” value=”false”/>
<param name=”self_diagnose” value=”false”/>
</node>
*/