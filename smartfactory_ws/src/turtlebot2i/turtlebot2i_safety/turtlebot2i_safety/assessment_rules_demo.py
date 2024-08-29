#!/usr/bin/env python3
# This script defines a function to generate fuzzy logic rules for assessing risk levels based on
# object distance and direction. The rules are created using the skfuzzy control system and are 
# formatted to specify conditions (antecedents) and their corresponding outcomes (consequences).
    
from skfuzzy import control as ctrl

def rule_list_generator(object_distance, object_direction, object_risk): 
    """
    Generates a list of fuzzy logic rules for risk assessment based on object distance and direction.

    Parameters:
    - object_distance: Fuzzy variable representing the distance of the object
    - object_direction: Fuzzy variable representing the direction of the object
    - object_risk: Fuzzy variable representing the risk level

    Returns:
    - A list of skfuzzy Rule objects
    """
    '''The format of rules is:
    rule_NUMBER = ctrl.Rule(Antecedent1['X'] & Antecedent2['Y'] & Antecedent3['Z'], Consequent['A'])'''
    # Define fuzzy logic rules
    rule001= ctrl.Rule(object_distance['Near']    & object_direction['Front'] ,     object_risk['VeryHigh'])
    rule002= ctrl.Rule(object_distance['Near']   & object_direction['FrontLeft'] ,  object_risk['High'])
    rule003= ctrl.Rule(object_distance['Near']   & object_direction['Left'] ,       object_risk['Low'])
    rule004= ctrl.Rule(object_distance['Near']   & object_direction['FrontRight'] , object_risk['High'])
    rule005= ctrl.Rule(object_distance['Near']   & object_direction['Right'] ,      object_risk['Low'])
    
    rule006= ctrl.Rule(object_distance['Near']   & object_direction['RearRight'] ,  object_risk['VeryLow'])
    rule007= ctrl.Rule(object_distance['Near']   & object_direction['RearLeft'] ,   object_risk['VeryLow'])

    rule008= ctrl.Rule(object_distance['Medium'] & object_direction['Front'] ,      object_risk['High'])
    rule009= ctrl.Rule(object_distance['Medium'] & object_direction['FrontLeft'] ,  object_risk['Medium'])
    rule010= ctrl.Rule(object_distance['Medium'] & object_direction['Left'] ,       object_risk['Low'])
    rule011= ctrl.Rule(object_distance['Medium'] & object_direction['FrontRight'] , object_risk['Medium'])
    rule012= ctrl.Rule(object_distance['Medium'] & object_direction['Right'] ,      object_risk['Low'])
    rule013= ctrl.Rule(object_distance['Medium'] & object_direction['RearRight'] ,  object_risk['VeryLow'])
    rule014= ctrl.Rule(object_distance['Medium'] & object_direction['RearLeft'] ,   object_risk['VeryLow'])

    rule015= ctrl.Rule(object_distance['Far']    & object_direction['Front'] ,      object_risk['Medium'])
    rule016= ctrl.Rule(object_distance['Far']    & object_direction['FrontLeft'] ,  object_risk['Low'])
    rule017= ctrl.Rule(object_distance['Far']    & object_direction['Left'] ,       object_risk['VeryLow'])
    rule018= ctrl.Rule(object_distance['Far']    & object_direction['FrontRight'] , object_risk['Low'])
    rule019= ctrl.Rule(object_distance['Far']    & object_direction['Right'] ,      object_risk['VeryLow'])
    rule020= ctrl.Rule(object_distance['Far']    & object_direction['RearRight'] ,  object_risk['VeryLow'])
    rule021= ctrl.Rule(object_distance['Far']    & object_direction['RearLeft'] ,   object_risk['VeryLow'])
    # Collect all rules into a list
    rule_list = [rule001, rule002, rule003, rule004, rule005, rule006, rule007, rule008, rule009, rule010, rule011, rule012, rule013, rule014, rule015, rule016, rule017, rule018, rule019, rule020, rule021]
    return rule_list


