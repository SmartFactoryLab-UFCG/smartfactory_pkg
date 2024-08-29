#!/usr/bin/env python3
# This script defines a function to generate fuzzy logic rules for mitigating risks based on
# object distance, direction, and the assessed risk level. The rules dictate the appropriate
# speed adjustments for the robot, using fuzzy logic to handle varying conditions.

from skfuzzy import control as ctrl

def rule_list_generator(object_distance,object_direction,object_risk_input,left_speed,right_speed):
    """
    Generates a list of fuzzy logic rules for speed mitigation based on object distance, direction, and risk level.
    Here are all mitigation rules.

    Parameters:
    - object_distance: Fuzzy variable representing the distance of the object
    - object_direction: Fuzzy variable representing the direction of the object
    - object_risk_input: Fuzzy variable representing the assessed risk level
    - left_speed: Fuzzy variable representing the speed of the left wheel
    - right_speed: Fuzzy variable representing the speed of the right wheel

    Returns:
    - A list of skfuzzy Rule objects for speed adjustment
    """
    # Mitigation Rule Format
    #rule00X= ctrl.Rule(object_distance['Near']&object_direction[]&object_risk_input , (left_speed['???'],right_speed['???']))
    # Define fuzzy logic rules for speed mitigation
    rule001= ctrl.Rule(object_distance['Near'] & object_direction['Front'], (left_speed['Stop'],right_speed['Stop']))
    
    rule002= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryLow'],  (left_speed['Medium'],right_speed['Medium']))
    rule003= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Low'],      (left_speed['Medium'],right_speed['Medium']))
    rule004= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Medium']))
    rule005= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Slow']))
    rule006= ctrl.Rule( object_distance['Medium'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule007= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule008= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule009= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['Medium'],   (left_speed['Fast'],right_speed['Medium']))
    rule010= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['High'],     (left_speed['Medium'],right_speed['Slow']))
    rule011= ctrl.Rule( object_distance['Medium'] & object_direction['FrontLeft'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Slow']))

    rule012= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule013= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule014= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Medium']))
    rule015= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Slow']))
    rule016= ctrl.Rule( object_distance['Medium'] & object_direction['Left'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule017= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule018= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule019= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Fast']))
    rule020= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Medium']))
    rule021= ctrl.Rule( object_distance['Medium'] & object_direction['FrontRight'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Medium']))

    rule022= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule023= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule024= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['Medium'],   (left_speed['Medium'],right_speed['Medium']))
    rule025= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['High'],     (left_speed['Slow'],right_speed['Slow']))
    rule026= ctrl.Rule( object_distance['Medium'] & object_direction['Right'] & object_risk_input['VeryHigh'], (left_speed['Slow'],right_speed['Slow']))

    rule027= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryLow'],  (left_speed['Fast'],right_speed['Fast']))
    rule028= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Low'],      (left_speed['Fast'],right_speed['Fast']))
    rule029= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['Medium'],   (left_speed['Fast'],right_speed['Fast']))
    rule030= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['High'],     (left_speed['Medium'],right_speed['Medium']))
    rule031= ctrl.Rule( object_distance['Far'] & object_direction['Front'] & object_risk_input['VeryHigh'], (left_speed['Medium'],right_speed['Medium']))

    rule032= ctrl.Rule( object_distance['Far'] & object_direction['FrontLeft'], (left_speed['Fast'],right_speed['Fast']))
    rule033= ctrl.Rule( object_distance['Far'] & object_direction['Left'] ,     (left_speed['Fast'],right_speed['Fast']))
    rule034= ctrl.Rule( object_distance['Far'] & object_direction['FrontRight'],(left_speed['Fast'],right_speed['Fast']))
    rule035= ctrl.Rule( object_distance['Far'] & object_direction['Right'],     (left_speed['Fast'],right_speed['Fast']))

    rule036= ctrl.Rule(object_distance['Near'] & object_direction['FrontLeft'],  (left_speed['Stop'],right_speed['Stop']))
    rule037= ctrl.Rule(object_distance['Near'] & object_direction['FrontRight'], (left_speed['Stop'],right_speed['Stop']))
    rule038= ctrl.Rule(object_distance['Near'] & object_direction['Left'],       (left_speed['Slow'],right_speed['Stop']))
    rule039= ctrl.Rule(object_distance['Near'] & object_direction['Right'],      (left_speed['Stop'],right_speed['Slow']))
    
    # New rules for reversing maneuvers
    rule040 = ctrl.Rule(object_distance['Near'] & object_direction['Front'], (left_speed['Reverse'], right_speed['Reverse']))
    rule041 = ctrl.Rule(object_distance['Near'] & object_direction['FrontLeft'], (left_speed['Reverse'], right_speed['Reverse']))
    rule042 = ctrl.Rule(object_distance['Near'] & object_direction['FrontRight'], (left_speed['Reverse'], right_speed['Reverse']))

    # New rules for continuous rotation
    rule043 = ctrl.Rule(object_distance['Near'] & object_direction['Left'], (left_speed['Stop'], right_speed['Slow']))
    rule044 = ctrl.Rule(object_distance['Near'] & object_direction['Right'], (left_speed['Slow'], right_speed['Stop']))
    
    #rule040= ctrl.Rule(object_direction['BigRear'],  (left_speed['Fast'], right_speed['Fast']))
    
    # Collect all rules into a list
    rule_list = [rule001, rule002, rule003,rule004, rule005, rule006, rule007, rule008, rule009, rule010,rule011, rule012, rule013,rule014, rule015, rule016,rule017, rule018, rule019, rule020,rule021, rule022, rule023,rule024, rule025, rule026, rule027, rule028, rule029, rule030,rule031, rule032, rule033,rule034, rule035, rule036, rule037, rule038, rule039, rule040, rule041, rule042, rule043, rule044]
    return rule_list
