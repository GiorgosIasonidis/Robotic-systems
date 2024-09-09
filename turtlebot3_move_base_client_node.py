#!/usr/bin/env python

import rospy #δημιουργία κόμβων ROS
import actionlib #υλοποίηση action clients και servers
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #Εισαγωγή των μηνυμάτων MoveBaseAction και MoveBaseGoal που χρησιμοποιούνται από το move_base

def movebase_client(x_goal, y_goal, qz_goal, qw_goal):
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #Δημιουργία action client με το όνομα "move_base" και το αρχείο action definition "MoveBaseAction"
    client.wait_for_server() #Αναμονή μέχρι να ξεκινήσει ο action server και να αρχίσει να ακούει για στόχους

   
    goal = MoveBaseGoal() #Δημιουργία αντικειμένου MoveBaseGoal για τον ορισμό του στόχου κίνησης
    goal.target_pose.header.frame_id = "map" #Ορισμός του πλαισίου αναφοράς του στόχου ως "map" (χάρτης)
    goal.target_pose.header.stamp = rospy.Time.now() #καθορισμός της χρονικής στιγμής που δημιουργείται ο στόχος
   
    goal.target_pose.pose.position.x = x_goal #καθορισμός της οριζόντιας θέσης του στόχου (x)
    goal.target_pose.pose.position.y = y_goal #καθορισμός της κατακόρυφης θέσης του στόχου (y)
    
    goal.target_pose.pose.orientation.z = qz_goal #καθορισμός της περιστροφής γύρω από τον άξονα z για τον στόχο (η κατεύθυνση της περιστροφής)
    goal.target_pose.pose.orientation.w = qw_goal #καθορισμός της περιστροφής γύρω από τον άξονα w για τον στόχο (πόσο γρήγορα περιστρέφεται)

    
    client.send_goal(goal) #Αποστολή στόχου στον action server
    wait = client.wait_for_result() #Αναμονή για την ολοκλήρωση της εκτέλεσης της ενέργειας από τον server
    
    if not wait: #Έλεγχος αν το αποτέλεσμα δεν έρθει, υποθέτοντας ότι ο server δεν είναι διαθέσιμος
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        
        return client.get_result() #Επιστροφή αποτελέσματος εκτέλεσης της ενέργειας


if __name__ == '__main__':
    try:
        
        rospy.init_node('movebase_client_py') #Αρχικοποίηση κόμβου rospy για να επιτραπεί στον SimpleActionClient να δημοσιεύει και να εγγράφει
	
	#Ορισμός στόχων xyzw
        goals = [
            [0.7, 1.6, 0.33, 0.94],
            [5.7, -3.9, -0.44, 0.89],
            [-6.0, -3.1, -0.29, 0.95]
        ]

        for goal in goals:
            result = movebase_client(goal[0], goal[1], goal[2], goal[3]) #Κλήση της συνάρτησης movebase_client με τις συντεταγμένες του τρέχοντος στόχου
            if result:
                rospy.loginfo("Goal execution done!")
            rospy.sleep(2)  #Αναμονή λίγο πριν προχωρήσει στον επόμενο στόχο

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.") #Καταγραφή μηνύματος όταν η εκτέλεση της πλοήγησης ολοκληρώνεται ή διακόπτεται


