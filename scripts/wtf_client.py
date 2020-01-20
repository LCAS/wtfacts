#! /usr/bin/env python

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the action, including the
# goal message and the result message.
import wtfacts.msg

from pymongo import MongoClient
import datetime
import pandas as pd


def wtf_send_op(client, op_code, model_name, vals, ts):

    # Creates a goal to send to the action server.
    goal = wtfacts.msg.WTFGoal()

    goal.operation = op_code
    goal.model = model_name
    goal.values = vals   
    goal.timestamps = ts

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    r = client.get_result()

    print("\nResponse to op (" + op_code+ ") is :\n")
    print(r)
    print('..............\n\n')

def wtf_create_model(client, model_name, vals, ts):    
    return  wtf_send_op(client, 'bcreate',  model_name, vals, ts)

def getDataFromMongoCollection(coll, sensorName, dbStartDate, dbEndDate):
    # This method is adapted to the structure of our entries in the database.
    # You may need to change it to fit your database structure.

    val = []
    tm = []

    # first element: last entry before start date
    andQuery = [{"item": sensorName}, {"timestamp": {"$lte": dbStartDate}}]
    cursor = coll.find({"$and": andQuery}).sort([("timestamp", -1)]).limit(1)

    # if database is not that old...
    if cursor.count() == 0:
        rospy.logerr('Sensor[%s]: Database started after requested (%s) start date...' % (sensorName,dbStartDate.ctime()))
        andQuery = [{"item": sensorName}]
        cursor = coll.find({"$and": andQuery}).sort([("timestamp", 1)]).limit(1)
        document = cursor.next()        
        currDate = (pd.Timestamp(document['timestamp']))
        rospy.logerr('Sensor[%s]: Earliest date is (%s) ...' % (sensorName,currDate.ctime()))
        return (val,tm)

    # our first entry will be value at starting time, either because is on
    # database or we asume it's same level till next change
    document = cursor.next()
    currVal = document['value'] # this should be 0 or 1
    currDate = toPOSIX(dbStartDate)
    val.append(currVal)
    tm.append(currDate)

    # rest of entries ...
    andQuery = [{"item": sensorName},
                {"timestamp": {"$gte": dbStartDate, "$lte": dbEndDate}}]
    cursor = coll.find({"$and": andQuery}).sort([("timestamp", 1)])

    rospy.logdebug('Sensor[%s]: Retrieved %d entries'% (sensorName,cursor.count()))
    for document in cursor:
        currVal = document['value'] # this should be 0 or 1
        currDate = toPOSIX(pd.Timestamp(document['timestamp']))
        val.append(currVal)
        tm.append(currDate)

    # if we don't have data on endDate, we duplicate last entry on end date
    # also, if no document was found on the interval, we add again the value
    # previous to interval
    if currDate != dbEndDate:
        currVal = val[-1]
        val.append(currVal)
        tm.append(toPOSIX(dbEndDate))    

    return (val,tm)

def toPOSIX(dateT):
    return (dateT - datetime.datetime(1970,1,1)).total_seconds()

def wtf_create_model_from_mongo(client):
    # parameters
    dbName = 'ENRICHME'
    collName = 'openhab'
    sensorName = 'Bathroom_Multi_Presence'
    mongo_port = 27017
    mongo_host = '127.0.0.1'
    startDate  = datetime.datetime(2017, 11, 8, 9, 25, 0, 0)  # 9/11/2017 09:25    
    endDate = datetime.datetime(2017, 11, 13, 23, 59, 30, 0)  # 13/11/2017 23:59
    model_name = 'bathroom'
    # connect to mongo collection
    mongo_client = MongoClient(host=mongo_host, port=mongo_port)
    db = mongo_client[dbName]
    coll = db[collName]

    # get the data
    (vals, ts) = getDataFromMongoCollection(coll, sensorName, startDate, endDate)

    # query the server
    return wtf_send_op(client, 'bcreate',  model_name, vals, ts)


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('wtfacts__client_py')
        # Creates the SimpleActionClient, passing the type of the action
        # (WTFAction) to the constructor.
        wtfacts_client = actionlib.SimpleActionClient('wtfacts_node', wtfacts.msg.WTFAction)

        # Waits until the action server has started up and started
        # listening for goals.
        wtfacts_client.wait_for_server()

        r1 = wtf_create_model(wtfacts_client, 'hola',  [0,0,0,1,1,1], [0,100,200,300,400,500] )
        r2 = wtf_create_model(wtfacts_client, 'adios', [1,1,0,0,0,0], [0,1000,2000,3000,4000,5000] )
        r3 = wtf_create_model_from_mongo(wtfacts_client)
        r4 = wtf_send_op(wtfacts_client,'append', 'adios', [1,0], [1200,1800] )        
        r5 = wtf_send_op(wtfacts_client,'list', '', [], [] )
        r6 = wtf_send_op(wtfacts_client,'predict', 'adios', [], [1500, 1800, 2000, 2100] )
        r7 = wtf_send_op(wtfacts_client,'delete', 'adios', [], [] )
        r8 = wtf_send_op(wtfacts_client,'list', '', [], [] )


    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
