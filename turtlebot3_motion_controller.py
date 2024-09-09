#!/usr/bin/env python

import rospy              # Εισαγωγή του πακέτου rospy για το ROS σε Python
from geometry_msgs.msg import Twist                 # Εισαγωγή του μηνύματος Twist από το πακέτο geometry_msgs, χρησιμοποιείται για να δώσει εντολές κίνησης σε ένα ρομπότ
from nav_msgs.msg import Odometry                   # Εισαγωγή του μηνύματος Odometry από το πακέτο nav_msgs, πληροφορίες όπως η θέση (x, y, z)
from tf.transformations import euler_from_quaternion               # Εισαγωγή της συνάρτησης euler_from_quaternion για τη μετατροπή προσανατολισμού από quaternion σε γωνία
from math import atan2, sqrt                         # Εισαγωγή των συναρτήσεων atan2 και sqrt για υπολογισμούς γωνιών και αποστάσεων

class TurtlebotMotionController:
    def __init__(self):
        rospy.init_node('turtlebot_motion_controller')  # Αρχικοποίηση κόμβου ROS με όνομα "turtlebot_motion_controller"
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)  # Εγγραφή στο topic '/odom' για λήψη δεδομένων θέσης
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Δημιουργία publisher για το topic '/cmd_vel' για αποστολή εντολών κίνησης
        self.waypoints = [[2.0, 2.0, 0.0], [4.0, 4.0, 0.0], [6.0, 6.0, 0.0], [8.0, 8.0, 0.0]]  # Οι στόχοι κίνησης σε μορφή [x, y, y]
        self.waypoint_index = 0  # Εκκίνηση από τον πρώτο στόχο
        self.rate = rospy.Rate(10)  # Συχνότητα ενημέρωσης 10 Hz

    def odom_callback(self, odom_msg):
        # Εκτελείται κάθε φορά που λαμβάνεται μήνυμα Odometry
        current_pose = odom_msg.pose.pose # Ανάκτηση της τρέχουσας θέσης του ρομπότ από το μήνυμα Odometry
        current_x = current_pose.position.x  # Τρέχουσα θέση στον άξονα x
        current_y = current_pose.position.y  # Τρέχουσα θέση στον άξονα y

        goal_x, goal_y, goal_yaw = self.waypoints[self.waypoint_index]  # Προσπελάστε τους στόχους κίνησης

        distance_to_goal = sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2) # Υπολογισμός απόστασης από τον τρέχοντα στόχο

        if distance_to_goal < 0.1: # Έλεγχος αν έχουμε φτάσει στον τρέχοντα στόχο
            self.waypoint_index += 1  # Προχωρήστε στον επόμενο στόχο
            if self.waypoint_index >= len(self.waypoints): # Έλεγχος αν έχουμε φτάσει στον τελευταίο στόχο
                self.waypoint_index = 0  # Επιστροφή στον πρώτο στόχο αν έχουμε φτάσει στον τελευταίο
            return

        # Υπολογισμός γωνίας προς τον τρέχοντα στόχο
        delta_x = goal_x - current_x # Υπολογισμός διαφοράς θέσης στον άξονα x
        delta_y = goal_y - current_y # Υπολογισμός διαφοράς θέσης στον άξονα y
        desired_yaw = atan2(delta_y, delta_x) # Υπολογισμός επιθυμητής γωνίας προσανατολισμού προς τον στόχο

        # Εξαγωγή της τρέχουσας γωνίας από τα δεδομένα Odometry
        orientation_q = current_pose.orientation  # Ανάκτηση του προσανατολισμού του ρομπότ από το μήνυμα Odometry
        _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]) # Μετατροπή του προσανατολισμού από quaternion σε γωνία

        # Υπολογισμός σφάλματος στη γωνία yaw
        yaw_error = desired_yaw - current_yaw

        # Έλεγχος αν η γωνία σφάλματος υπερβαίνει το πεδίο τιμών [-π, π]
        if yaw_error > 3.14159:
            yaw_error -= 2*3.14159 # Προσαρμογή στο εύρος [-π, π]
        elif yaw_error < -3.14159:
            yaw_error += 2*3.14159 # Προσαρμογή στο εύρος [-π, π]

        # Υπολογισμός γραμμικής και γωνιακής ταχύτητας
        linear_velocity = 0.2  # Σταθερή γραμμική ταχύτητα
        angular_velocity = 0.5 * yaw_error  # Προσαρμοσμένη γωνιακή ταχύτητα

        cmd_vel_msg = Twist() # Δημιουργία μηνύματος κίνησης Twist
        cmd_vel_msg.linear.x = linear_velocity # Ορισμός γραμμικής ταχύτητας x
        cmd_vel_msg.angular.z = angular_velocity # Ορισμός γωνιακής ταχύτητας y

        # Δημοσίευση μηνύματος κίνησης
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()  # Εκτέλεση κόμβου ROS για πάντα

if __name__ == '__main__':
    try:
        controller = TurtlebotMotionController()  # Δημιουργία αντικειμένου κλάσης TurtlebotMotionController
        controller.run()  # Έναρξη λειτουργίας του κόμβου
    except rospy.ROSInterruptException:
        pass  # Σιγουρευτείτε ότι το πρόγραμμα θα κλείσει με ασφάλεια σε περίπτωση διακοπής του κόμβου



