#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach_ros import IntrospectionServer
from std_msgs.msg import String
from dialogflow_ros.msg import DialogflowResult
# import smach_msgs # If I do need this, then would need to adjust CMakeLists and package.xml accordingly
# Imports for navigation:
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import json
import os
import math
# For ocr:
from medicine_retrieval.srv import OCRService, OCRServiceRequest
# For grabbing / dropping bottle
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelConfiguration, SetModelState
from geometry_msgs.msg import Twist, Pose

# TODO add all these nodes to a single LAUNCH FILE. They would all need to be turned on as well with this sm
# Perhaps keep the tiago one separate as it is a whole different concern - and print msgs won't be seen
# Also keep darknet seperate maybe (unless it only spews out msgs upon trigger)
# They would just not do anything until they are triggered to do so
# Alternatively (DON'T DO THIS), could figure out how to activate from this class (there could also be ros 2 node lifecycle)
# But above would be the cleanest way to do so... Like darknet could stay on (but only publishes one image, no queue)
                                                    # Then ocr would be triggered, us latest darknet output, and 'sleep'
                                                    # If I could turn darknet off too that would be ideal
                                                    # But best design is node stays alive, just spins until called
class RestState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate', 'rest'], input_keys=['input_data'], output_keys=['output_data'])

    def execute(self, ud):
        print('Idle State, waiting for any command (speak your command)!')
        input_data = ud.input_data # Rest state does not really need this data, since it is only doing as per command
        rospy.loginfo("User data received in the Rest state: %s", input_data)

        df_result = rospy.wait_for_message('/dialogflow_client/results', DialogflowResult)
        # If wait_for_message does not work could do the standard way where this is a subscriber, and has a callback to store msg data as intent... The execute has while self.intent is None and not rospy.is_shutdown(): rospy.sleep(1)
        # The df result is the whole DialogflowResult msg (which for my purposes use the intent and paramters)


        # Process the df_result here (doing different tasks for different results) - logic to determine what to do with the intent
        # if df_result.intent == 'Get_Meds' (and df_result.entity would have which meds)
                # A further tasks for this is to get the medicine back to the nurse who ordered (but add that later if time)
        if df_result.intent == 'get_medicine':
            # get the entity - loop through them and the one which is medicine name, take that and pass it on
            for parameters in df_result.parameters:
                if parameters.param_name == 'medicine_type':
                    med_name = parameters.value[0]
                    break # Currently only accepting on med - change this in future if needed

            if med_name == '': # If it got left blank
                ud.output_data = []
                return 'rest'

            ud.output_data = ['get_medicine', 'medicine_table_1', med_name]
            # Future possible addition - add where to drop it - currently will only drop at default dropping table
            return 'navigate'

        # if Intent.intent == 'Go_to_Nurse@ (and the entity would have which nurse)
        # The nurse one would be similar to idle where the speaker would have to give command
            # In the other case where it is coming back
        elif df_result.intent == 'go_to_nurse':
            #Get entity (try this at least, figure more out on dialogflow)
            #output_data = location is nurse
            has_medicine = False # In this spoken case, it is going just to get next command from nurse; not carrying medicine currently
            # Above rn always false: in case of going back after grabbing bottle, it would not come through here as a spoken command
            nurse_num = 'nurse_1' # This is the default nurse if none is specified
            for parameters in df_result.parameters:
                if parameters.param_name == 'nurse_num':
                    nurse_num = parameters.value[0]

            ud.output_data = ['go_to_nurse', nurse_num, has_medicine]
            return 'navigate'

        elif df_result.intent == 'return_home':
            # Further advancement here could be it goes to its nearest rest corner, but that is beyond for now
            # No data really to keep, but could add something later if we wanted (like a list to keep track)
            ud.output_data = ['return_home', 'rest_corner']
            return 'navigate'


        else: #== '':
            print('Please specify an intent')
            return 'rest' # Goes back to waiting for a command




class MoveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognise_meds', 'rest', 'drop_bottle'], input_keys=['input_data'], output_keys=['output_data'])

    # Note - ud should always have the second value as location; first value is intent, which it then decides what to do next
    def execute(self, ud): # The user data is data passed from the last state
        # Get the context from the previous state using userdata
        input_data = ud.input_data # input data[0] is intent, [1] is location, [2] intent-specific info
        rospy.loginfo("User data received in the Navigation state: %s", input_data)
        print('Moving State, Will now perform moving!')

        # The actual navigation happens after if statements (since the second value of input_data would always be the location)

        # Case for going to nurse (either upon spoken command, or through other state)
        if input_data[0] == 'go_to_nurse':
            # Different cases within this going with bottle or without
            if input_data[2] == False:
                ud.output_data = [] # Nothing to do so no ud needed rn
                return_val = 'rest' # It will navigate, but nothing more to do after this
            elif input_data[2] == True:
                ud.output_data = [input_data[1], input_data[3]] # [1] is which nurse, [3] is which model to drop
                return_val = 'drop_bottle'

        # Case for getting medicine - nav to medicine counter
        elif input_data[0] == 'get_medicine':
            med_name = input_data[2]
            ud.output_data = [input_data[1], med_name] # The input_data[1] passes on which table it is at, so it can use that to decide what to do next
            return_val = 'recognise_meds'


        elif input_data[0] == 'return_home':
            ud.output_data = [] # This does not need any data at the moment - return home is like reset rn
            return_val = 'rest'

        else:
            print('No valid Intent given (should not come to this)!! Going back to rest state and waiting for command.')
            # No navigation to do in this case, so can return to rest
            return 'rest'


        # Send navigation command
        location = input_data[1] # As said above, this is always the case, but open to future changes
        nav_result = self.handle_navigation_client(location)

        if nav_result == "succeeded":
            print('Navigation seems to have succeeded - going to next state')
            # Output data set for each case above
            return return_val
        elif nav_result == "failed" or nav_result == "invalid_location":
            print('navigation did not succeed: ', nav_result, '; Returning to rest state awaiting new command')
            ud.output_data = [] # Clear the output data as there has been some error navigating
            return 'rest'


    def handle_navigation_client(self, location):

        # First, get the location details for all locations
        locations_filepath = os.path.join(os.path.dirname(__file__), '..', 'config', 'locations.json')
        with open(locations_filepath, 'r') as file:
            all_locations_data = json.load(file)

        # Now get the data for the one which corresponds to my location string
        if location in all_locations_data:
            location_data = all_locations_data[location]
        else:
            print('Location given does not correspond, check given location has an entry in locations.json')
            return 'invalid_location'

        # Now, establish client and send a move base goal for the server to move to
        nav_client = actionlib.SimpleActionClient('move_base', ActionSpec=MoveBaseAction)
        nav_client.wait_for_server()

        # Set the goal parameters, according to needed location data gotten above
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = location_data["x"]
        goal.target_pose.pose.position.y = location_data["y"]
        goal.target_pose.pose.position.z = location_data["z"]

        yaw = location_data["yaw"]
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = math.sin(yaw/ 2)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2)

        nav_client.send_goal(goal)
        #The goal has been sent. Now will wait for resutl - could later also add a feedback thing which prints later
        nav_client.wait_for_result()

        if nav_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            return 'failed'




class MedicineRecognitionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate', 'grab_bottle'], input_keys=['input_data'], output_keys=['output_data'])
        # Navigate outcome is correct, and would go wherever next based on input data (so if at table 1, navigate to table 2, if at table 2, navigate to nurse (nurse 1 only for now))
        # Grab bottle is also correct - only triggers when there is a bottle to grab, then navigates straight back to nurse table

    def execute(self, ud):
        print('Meds Recognition State, running Darknet & OCR-ing the text from med bottles!')
        input_data = ud.input_data #input_data[1] is bottle to look for; input_data[0] is location - to be used for next step
        rospy.loginfo("User data received in the MedsRecognition state: %s", input_data)

        current_table = input_data[0]
        med_bottle_to_find = input_data[1]
        med_bottle_found, med_bottle_position = self.find_bottle(med_bottle_to_find)
        # med_bottle location has iniitially been defined as if it's the 1st, 2nd, 3rd and so on (n'th) bottle

        if med_bottle_found is True:
            # Grab the bottle; Send such necessary output data to grab (like med position on which table)
            ud.output_data = [current_table, med_bottle_position] # Only med bottle name needed for grabbing... Other info like nurse to drop off to, or where it was found etc could potentially be added later
            return 'grab_bottle'

        elif med_bottle_found is False:
            # Either navigate to 2nd table if at table 1
            # Or if at table 2, print that not found and navigate to home
            current_table = input_data[0]
            if current_table == 'medicine_table_1':
                ud.output_data = ['get_medicine', 'medicine_table_2', med_bottle_to_find]
                return 'navigate'
            elif current_table == 'medicine_table_2':
                print('The medicine bottle you are looking for has not been found. I go home and await any command.')
                ud.output_data = ['return_home', 'rest_corner']
                return 'navigate'



    # Do the Darknet image recognition combined with OCR to find the specific medicine bottle
    # Note that darknet should be assumed here to be running (it would always be running when this is called)
    def find_bottle(self, med_bottle_to_find):
        med_bottle_found = False
        med_bottle_position = None

        # Darknet will always be running when this function is called, so continue with ocr service
        rospy.wait_for_service('ocr_service')
        try:

            # Create the service proxy (like the client I think)
            ocr_service = rospy.ServiceProxy('ocr_service', OCRService)
            # Create request object to send
            ocr_request = OCRServiceRequest()
            ocr_request.bottle_to_find = med_bottle_to_find
            # Send request to service
            ocr_result = ocr_service(ocr_request)

            med_bottle_found = ocr_result.found
            if med_bottle_found:
                med_bottle_position = ocr_result.med_number # Currently just which bottle it is on the table, 1st or 2nd or so on
                print("Received values: med_bottle_found:{}, med_number:{}".format(med_bottle_found, med_bottle_position))
            else:
                print("Received values: med_bottle_found:{}".format(med_bottle_found))

        except rospy.ServiceException as error:
            print('OCR service had some error!')

        return med_bottle_found, med_bottle_position




class GrabBottleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigate', 'rest'], input_keys=['input_data'], output_keys=['output_data'])
        # Only needs outcome of navigate, and even back to nurse dropping station only (rest if failed);
        # Output data needs to be correctly set (look at move state case for going back to nurse with bottle)
        # Input data format: [current_table, med_bottle_position] # Only med bottle name needed for grabbing... Other info like nurse to drop off to, or where it was found etc could potentially be added later
        self.model_states = None

    def execute(self, ud):
        # For now it should just receive the bottle on Tiago body
        print('Meds Grab Bottle State, Grabbing bottle!')
        input_data = ud.input_data
        rospy.loginfo("User data received in the Grab Bottle state: %s", input_data)

        current_table = input_data[0] # The table the med was found at
        med_bottle_position = input_data[1] # The position of the med bottle on the table

        bottle_model_name = self.get_med_model_name(current_table, med_bottle_position)
        destination_model_pose = self.get_model_pose('tiago') # At this point, always want to move it onto center of Tiago

        # Send new pose to gazebo service for my mode
        sucessfully_moved = self.move_model_to_new_pose(bottle_model_name, destination_model_pose)

        if sucessfully_moved is True:
            print('Good job - did some moving with SetModelState')
        else: # In case some error with moving
            print('Could not move - some error with model names perhaps - aborting')
            ud.output_data = 'Failed'
            return 'rest'

        ud.output_data = ['go_to_nurse', 'nurse_1', True, bottle_model_name]
        return 'navigate'


    # Consider extrapolating this and putting into something drop/grab can both access - as pretty similar code with picking and dropping
    def move_model_to_new_pose(self, object_model_name, destination_model_pose):
        success = False
        if object_model_name == None:
            return False
        elif destination_model_pose == None:
            return False

        dest_pose_adjusted = destination_model_pose
        # dest_pose_adjusted.position.z = destination_model_pose.position.z + 2
        # dest_pose_adjusted.position.x = destination_model_pose.position.x - 0.2
        # dest_pose_adjusted.position.y = destination_model_pose.position.y - 0.2


        # Send a service request to move my model to pose of destination (z should be .5 above) - rest could be exact post
        #set_model_configuration_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        try:
            model_state = ModelState()
            model_state.model_name = object_model_name
            model_state.pose = dest_pose_adjusted
            model_state.twist = Twist()
            response = set_model_state_service(model_state)
            success = True
            # current_configuration = set_model_configuration_service.call(model_name=object_model_name).configuration
            # current_configuration.urdf_pose = dest_pose_adjusted
            # response = set_model_configuration_service(model_name=object_model_name, pose=dest_pose_adjusted)
        except rospy.ServiceException as error:
            print(error)

        return success


    def get_med_model_name(self, current_table, med_bottle_position):
        model_name = None

        # Load all the medicine bottle to model mapping json
        med_model_filepath = os.path.join(os.path.dirname(__file__), '..', 'config', 'medicine_bottle_to_model.json')
        with open(med_model_filepath, 'r') as file:
            all_tables_data = json.load(file)

        # Now get the data for the table which corresponds to my table string, and the model at position I have
        if current_table in all_tables_data:
            table_data = all_tables_data[current_table]
            model_name = table_data[str(med_bottle_position)]

        else:
            print('Table given does not correspond, check given table has an entry in medicine_bottle_to_model.json')
            return None


        return model_name


    def get_model_pose(self, model_name):
        rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.set_model_states)
        while self.model_states == None:
            rospy.sleep(0.1) # Sleep until we get model states loaded

        model_index = -1
        for i, name in enumerate(self.model_states.name):
            if name == model_name:
                model_index = i
                break

        if model_index is not -1:
            model_pose = self.model_states.pose[model_index]
            return model_pose
        else:
            print('Model by name of {} not found'.format(model_name))
            return None

    def set_model_states(self, models):
        self.model_states = models




class DropBottleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rest'], input_keys=['input_data'], output_keys=['output_data'])
        self.model_states = None
        # Currently only dropping at nurse table 1

    def execute(self, ud):
            print('Meds Drop Bottle State, Dropping bottle!')
            input_data = ud.input_data
            rospy.loginfo("User data received in the Dropo Bottle state: %s", input_data)

            model_to_drop = input_data[1]
            destination_model_pose = self.get_model_pose('table')  # At this point, always want to move it onto center of table

            # Send new pose to gazebo service for my mode
            sucessfully_moved = self.move_model_to_new_pose(model_to_drop, destination_model_pose)

            if sucessfully_moved is True:
                print('Good job - did some moving onto table')
            else:  # In case some error with moving
                print('Could not move - some error with model names perhaps - aborting')
                ud.output_data = 'Failed'
                return 'rest'

            ud.output_data = []
            return 'rest'

            # Consider extrapolating this and putting into something drop/grab can both access - as pretty similar code with picking and dropping

    def move_model_to_new_pose(self, object_model_name, destination_model_pose):
        success = False
        if object_model_name == None:
            return False
        elif destination_model_pose == None:
            return False

        dest_pose_adjusted = destination_model_pose
        dest_pose_adjusted.position.z = destination_model_pose.position.z + 2
        # dest_pose_adjusted.position.x = destination_model_pose.position.x - 0.5
        # dest_pose_adjusted.position.y = destination_model_pose.position.y - 0.2

        # Send a service request to move my model to pose of destination (z should be .5 above) - rest could be exact post
        # set_model_configuration_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        try:
            model_state = ModelState()
            model_state.model_name = object_model_name
            model_state.pose = dest_pose_adjusted
            model_state.twist = Twist()
            response = set_model_state_service(model_state)
            success = True
            # current_configuration = set_model_configuration_service.call(model_name=object_model_name).configuration
            # current_configuration.urdf_pose = dest_pose_adjusted
            # response = set_model_configuration_service(model_name=object_model_name, pose=dest_pose_adjusted)
        except rospy.ServiceException as error:
            print(error)

        return success

    # We have model name passed into this state from navigate already
    # def get_med_model_name(self, current_table, med_bottle_position):
    #     model_name = None
    #
    #     # Load all the medicine bottle to model mapping json
    #     med_model_filepath = os.path.join(os.path.dirname(__file__), '..', 'config', 'medicine_bottle_to_model.json')
    #     with open(med_model_filepath, 'r') as file:
    #         all_tables_data = json.load(file)
    #
    #     # Now get the data for the table which corresponds to my table string, and the model at position I have
    #     if current_table in all_tables_data:
    #         table_data = all_tables_data[current_table]
    #         model_name = table_data[str(med_bottle_position)]
    #
    #     else:
    #         print('Table given does not correspond, check given table has an entry in medicine_bottle_to_model.json')
    #         return None
    #
    #     return model_name

    def get_model_pose(self, model_name):
        rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.set_model_states)
        while self.model_states == None:
            rospy.sleep(0.1)  # Sleep until we get model states loaded

        model_index = -1
        for i, name in enumerate(self.model_states.name):
            if name == model_name:
                model_index = i
                break

        if model_index is not -1:
            model_pose = self.model_states.pose[model_index]
            return model_pose
        else:
            print('Model by name of {} not found'.format(model_name))
            return None

    def set_model_states(self, models):
        self.model_states = models






if __name__ == '__main__':
    rospy.init_node('smach_meds_retrieval_node')

    meds_sm = smach.StateMachine(outcomes=['finished', 'aborted'])
    # meds_sm.userdata.input_data = 'Starting' # Initiallising key
    meds_sm.userdata.output_data = 'Starting' # Initiallising key
    # General note about input & output data: using a list or any data structure is fine; but for a more complex, it is also possible to design a quick custom class (serializeable) and use this object as data type

    with meds_sm:
        smach.StateMachine.add('REST_STATE', RestState(), transitions={'navigate': 'MOVE_STATE', 'rest': 'REST_STATE'}, remapping={'input_data': 'output_data'})
        smach.StateMachine.add('MOVE_STATE', MoveState(), transitions={'recognise_meds': 'RECOGNISE_STATE', 'rest': 'REST_STATE', 'drop_bottle': 'DROP_BOTTLE_STATE'}, remapping={'input_data': 'output_data'})
        smach.StateMachine.add('RECOGNISE_STATE', MedicineRecognitionState(), transitions={'navigate': 'MOVE_STATE', 'grab_bottle': 'GRAB_BOTTLE_STATE'}, remapping={'input_data': 'output_data'})
        smach.StateMachine.add('GRAB_BOTTLE_STATE', GrabBottleState(), transitions={'navigate': 'MOVE_STATE', 'rest': 'REST_STATE'}, remapping={'input_data': 'output_data'})
        smach.StateMachine.add('DROP_BOTTLE_STATE', DropBottleState(), transitions={'rest': 'REST_STATE'}, remapping={'input_data': 'output_data'})
        # TODO - if any updates with states, then update them above
        # Do the same for ALL. Note the transitions are named as defined in the classes, same with remapping (names in the '' are matching basically)


    # Create and start introspection server (which allows to visualise smach) - I don't have the package files currently
    # intrspection_server = smach_ros.IntrospectionServer('introspection_server', meds_sm, '/SM_GUI')
    # intrspection_server.start()

    # Here the smach is executed - runs until it stops
    result = meds_sm.execute()

    # Close introspeciton
    # intrspection_server.stop()