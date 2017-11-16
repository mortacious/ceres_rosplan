#!/usr/bin/python2
import rospy
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

if __name__ == "__main__":
    rospy.init_node("init_ceres_rosplan")
    
    rospy.loginfo("(Init_Ceres_Rosplan) Waiting for knowledge update service")
    
    rospy.wait_for_service("kcl_rosplan/update_knowledge_base")
    rospy.loginfo("(Init_Ceres_Rosplan) Initializing robot 'ceres' to waypoint 'wp0'")
    update_service = rospy.ServiceProxy("kcl_rosplan/update_knowledge_base", KnowledgeUpdateService)


    update_service(update_type=KnowledgeUpdateService._request_class.ADD_KNOWLEDGE,
                   knowledge=KnowledgeItem(knowledge_type=KnowledgeItem.INSTANCE,
                                           instance_type='robot',
                                           instance_name='ceres'))
    update_service(update_type=KnowledgeUpdateService._request_class.ADD_KNOWLEDGE,
                   knowledge=KnowledgeItem(knowledge_type=KnowledgeItem.FACT,
                                           attribute_name='robot_at',
                                           values=[KeyValue(key='v', value='ceres'),
                                                   KeyValue(key='wp', value='wp0')]))


