#! /usr/bin/env python

import rospy

import actionlib

import wtfacts.msg

from wtflib.wtfmodule import *

class WTFACTionServer(object):

    def __init__(self, name):
        self._action_name = name

        # TODO: find a better way to store data ...
        self._current_models = dict()
        self._stored_data = dict()
        self._model_names_separator = ', '

        rospy.logdebug("Node [" + rospy.get_name() + "]: " + " Creating Action Server")
        self._as = actionlib.SimpleActionServer(self._action_name,  wtfacts.msg.WTFAction, execute_cb=self.execute_cb, auto_start = False)
        rospy.logdebug("Node [" + rospy.get_name() + "]: " + " Starting Action Server")
        self._as.start()
        rospy.logdebug("Node [" + rospy.get_name() + "]: " + " Spin")
        rospy.spin()
      
    def execute_cb(self, goal):
        receivedOperation = str.lower(goal.operation)
        rospy.logdebug("Received action request.")
        rospy.logdebug("Action: %s", receivedOperation)

        # helper variables
        result = wtfacts.msg.WTFResult()
        result.success = True       

        # TODO: allow preemption        
        # check that preempt has not been requested by the client
        # if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         result.success = False
        #         break

        # TODO: check input data (len, values...)

        # start executing the action            
        # possible operations are 'bcreate', 'append', 'predict', 'delete', 'list'
        if   receivedOperation== 'bcreate':
                result=self.perform_bcreate_op(goal)
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
        
    def perform_bcreate_op(self, goal):
        # TODO: explore change in default parameters: resampling = '30S', mother_wavelet_type = 'rbio2.2', wave_mode = 'per', lenThVector=10
        result = wtfacts.msg.WTFResult()

        # give some feedback
        self.sendFeedback("Creating")
        
        #   -  'bcreate' creates a new wavelet [model] using the corresponding [values] and [timestamps]
        if not self.isModelled(goal.model):
            X = goal.values
            T = goal.timestamps

            # give some feedback
            self.sendFeedback("Computing model (" + goal.model+ ")")

            (threshold_vector,mse_list) = chooseThreshold(X, T)

            # TODO: choose threshold in a smarter way ...
            c_threshold = threshold_vector[-1]

            model = createModel(X, T, c_threshold)
            # (cAhat, cDhat, N, times[0], resampling, wave_mode, mother_wavelet_type) = model
            
            self.storeModel(goal.model, model, X, T)
            result.success = True
            result.message = "Model (" + goal.model+ ") computed"
            self.sendFeedback("Success: " + result.message)            

        else:
            result.success = False
            result.message = "Model (" + goal.model+ ") already in server"
            self.sendFeedback("ERROR: " + result.message)
        return result

    def perform_append_op(self, goal):
        # TODO: Im just recomputing the model by appending new data to the old raw data, 
        #        not caring if there is a gap between the new data and the old one.
        #        A better way to do this could be [raw data] + [predictions] + [new data]
        
        result = wtfacts.msg.WTFResult()
        
        # give some feedback
        self.sendFeedback("Appending to (" + goal.model+ ")")

        #   -  'append' appends [values] and [timestamps] to the provided [model], recalculating the model
        if self.isModelled(goal.model):
            X_add = goal.values
            T_add = goal.timestamps

            (X_old,T_old) = self.getSourceData(goal.model)
            
            # warning: if you use lists, this is ok. if you use numpy arrays, this could be awful
            X = X_old + X_add
            T = T_old + T_add

            # give some feedback
            self.sendFeedback("Re-computing model (" + goal.model+ ")")

            (threshold_vector,mse_list) = chooseThreshold(X, T)

            # TODO: choose threshold in a smarter way ...
            c_threshold = threshold_vector[-1]

            model = createModel(X, T, c_threshold)
            # (cAhat, cDhat, N, times[0], resampling, wave_mode, mother_wavelet_type) = model
            
            self.replaceModel(goal.model, model,X, T)
            result.success = True
            result.message = "Model (" + goal.model+ ") recomputed"
            self.sendFeedback("Success: " + result.message)            

        else:
            result.success = False
            result.message = "Model (" + goal.model+ ") not in server. Unable to append"
            self.sendFeedback("ERROR: " + result.message)
        return result

    def perform_predict_op(self, goal):
        result = wtfacts.msg.WTFResult()

        # give some feedback
        self.sendFeedback("Predicting in model (" + goal.model+ ")")

        #   -  'predict' returns expected [values] at provided [model] and  [timestamps] 
        if self.isModelled(goal.model):
            T = goal.timestamps
            
            model = self.getModel(goal.model)

            # give some feedback
            self.sendFeedback("Predicting data")            
            X = predictWaveletValues(T, model)
            
            result.values = X
            result.success = True
            result.message = "Prediction computed"
            self.sendFeedback("Success: " + result.message)            

        else:
            result.success = False
            result.message = "Model (" + goal.model+ ") not in server. Unable to predict"
            self.sendFeedback("ERROR: " + result.message)
        return result

    def perform_delete_op(self, goal):
        result = wtfacts.msg.WTFResult()
        
        # give some feedback
        self.sendFeedback("Deleting (" + goal.model+ ") ")

        if self.isModelled(goal.model):
            self.deleteModel(goal.model)

            result.success = True
            result.message = "Model (" + goal.model+ ") deleted"
            self.sendFeedback("Success: " + result.message)    
        else:
            result.success = False
            result.message = "Model (" + goal.model+ ") not in server. Unable to delete"
            self.sendFeedback("ERROR: " + result.message)
        return result

    def perform_list_op(self, goal):
        result = wtfacts.msg.WTFResult()
        
        # give some feedback
        self.sendFeedback("Listing")

        result.message = self.listModels()
        result.success = True
        self.sendFeedback("Success: Models listed")    
 
        return result

    def create_error_result(self, reason_str):
        result = wtfacts.msg.WTFResult()
        
        # give some feedback
        self.sendFeedback("Error!: "+reason_str)
        result.success = False
        result.message = reason_str
        return result

    # .................................................................................
    #                               Helper functions here:
    # .................................................................................

    def sendFeedback(self, feedback_message):
        feedback = wtfacts.msg.WTFFeedback()
        feedback.status = feedback_message
        self._as.publish_feedback(feedback)

    def printModel(self, modelData):
        (cAhat, cDhat, N, times, resampling, wave_mode, mother_wavelet_type) = modelData
        print('\n\n\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n')
        print(type(cAhat))
        print('\n...........................................................................\n')
        print(type(cDhat))
        print('\n...........................................................................\n')
        print(type(N))
        print('\n...........................................................................\n')
        print(type(times))
        print('\n...........................................................................\n')
        print(type(resampling))
        print('\n...........................................................................\n')
        print(type(wave_mode))
        print('\n...........................................................................\n')
        print(type(mother_wavelet_type))
        print('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n\n\n')
                    

    # .................................................................................
    # TODO: I'm using dicts to store data, but something better can be used ...
    # .................................................................................

    def isModelled(self, model_name):
        ans = (model_name in self._current_models)
        return ans
    
    def listModels(self):
        ans = ''
        model_names_list = self._current_models.keys()        
        
        if len(model_names_list)>0:
            ans = self._model_names_separator.join(model_names_list)

        return ans

    def storeModel(self, model_name, modelData, values, timestamps):
        # (cAhat, cDhat, N, times[0], resampling, wave_mode, mother_wavelet_type) = modelData
        if not self.isModelled(model_name):
            self._current_models[model_name] = modelData
            self._stored_data[model_name] = (values, timestamps)
        else:
            rospy.logerr("Node [" + rospy.get_name() + "]: " + " Model (" + model_name + ") already exists. Not storing model ")

    def deleteModel(self, model_name):
        if self.isModelled(model_name):
            del self._current_models[model_name]
            del self._stored_data[model_name]
        else:
            rospy.logerr("Node [" + rospy.get_name() + "]: " + " Model (" + model_name + ") does not exist. Not deleting model")

    def replaceModel(self, model_name, modelData, values, timestamps):
        if self.isModelled(model_name):
            self.deleteModel(model_name)
            self.storeModel(model_name, modelData, values, timestamps)
        else:
            rospy.logerr("Node [" + rospy.get_name() + "]: " + " Model (" + model_name + ") does not exist. Can't replace it")

    def getModel(self, model_name):
        modelData = None

        if self.isModelled(model_name):
            modelData = self._current_models[model_name]
        else:
            rospy.logerr("Node [" + rospy.get_name() + "]: " + " Model (" + model_name + ") does not exist. Can't retrieve it")

        return modelData

    def getSourceData(self, model_name):
        (values, timestamps) = (None, None)

        if self.isModelled(model_name):
            (values, timestamps) = self._stored_data[model_name]
        else:
            rospy.logerr("Node [" + rospy.get_name() + "]: " + " Model (" + model_name + ") does not exist. Can't retrieve data")

        return (values,timestamps)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('wtfacts_node') #, log_level=rospy.DEBUG)

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        server = WTFACTionServer(rospy.get_name())
    except rospy.ROSInterruptException: 
        pass