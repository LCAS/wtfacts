#! /usr/bin/env python

import rospy

import actionlib

import WTF.msg

import wtflib 

class WTFAction(object):


    def __init__(self, name):
        self._action_name = name
        self._current_models = dict()
        self._as = actionlib.SimpleActionServer(self._action_name, WTF.msg.WTFAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.spin()
      
    def execute_cb(self, goal):
        receivedOperation = str.lower(goal.operation)
        rospy.logdebug("Received action request.")
        rospy.logdebug("Action: %s", receivedOperation)

        # helper variables
        result = WTF.msg.WTFResult()
        result.success = True       

        # TODO: allow preemption        
        # check that preempt has not been requested by the client
        # if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         result.success = False
        #         break

        # start executing the action            
        # possible operations are 'create', 'append', 'predict', 'delete', 'list'
        if   receivedOperation== 'create':
                result=self.perform_create_op(goal)
        elif receivedOperation== 'append':
                result=self.perform_append_op(goal)
        elif receivedOperation== 'predict':
                result=self.perform_predict_op(goal)
        elif receivedOperation== 'delete':
                result=self.perform_delete_op(goal)
        elif receivedOperation== 'list':
                result=self.perform_list_op(goal)
        else:
            result=self.create_error_result('Unknown operation: '+ receivedOperation)

        
        if (result.success):
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._as.set_aborted(result)

        
    def perform_create_op(self, goal):
        # TODO: explore change in default parameters: resampling = '30S', mother_wavelet_type = 'rbio2.2', wave_mode = 'periodization', lenThVector=10
        result = wave_acts.msg.WaveResult()

        # give some feedback
        sendFeedback("Creating")
        
        #   -  'create' creates a new wavelet [model] using the corresponding [values] and [timestamps]
        if not (goal.model in _current_models):
            X = goal.values
            T = goal.timestamps
            
            # give some feedback
            sendFeedback("Computing model")

            (threshold_vector,mse_list) = chooseThreshold(X, T)
            model = createModel(X, T, c_threshold)
            # (cAhat, cDhat, N, times[0], resampling, wave_mode, mother_wavelet_type) = model
            
            _current_models[goal.model] = model
            result.success = True

        else:
            result.success = False
            result.message = "Model already in server"
            sendFeedback("ERROR: " + result.message)
        return result

    def perform_append_op(self, goal):
        result = wave_acts.msg.WaveResult()
        
        # give some feedback
        sendFeedback("Appending")

        #TODO awesome code here!         
        return result

    def perform_predict_op(self, goal):
        result = wave_acts.msg.WaveResult()

        # give some feedback
        sendFeedback("Predicting")

        #TODO awesome code here!         
        return result

    def perform_delete_op(self, goal):
        result = wave_acts.msg.WaveResult()
        
        # give some feedback
        sendFeedback("Deleting")

        #TODO awesome code here!         
        return result

    def perform_list_op(self, goal):
        result = wave_acts.msg.WaveResult()
        
        # give some feedback
        sendFeedback("Listing")

        #TODO awesome code here!         
        return result

    def create_error_result(self, reason_str):
        result = wave_acts.msg.WaveResult()
        
        # give some feedback
        sendFeedback("Error!: "+reason_str)

        #TODO awesome code here!         
        return result

    # helper functions here:

    def sendFeedback(feedback_message):
        feedback = WTF.msg.WTFFeedback()
        feedback.status = feedback_message
        self._as.publish_feedback(feedback)




# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('wave_acts_node')#, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        server = WTFAction(rospy.get_name())
    except rospy.ROSInterruptException: 
        pass