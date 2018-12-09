#!/usr/bin/env python
import rospy
import numpy as np
import re
import Queue
from multiprocessing import process
from std_msgs.msg import String

class Agent:
  def __init__(self, numTools, numWork):
    self.numTools = numTools
    self.numWork = numWork
    self.mdp = {
        "states" : [],
        "actions" : np.zeros((numTools, numWork)),
        "rewards" : np.zeros((numTools, numWork)),
        "model" :{}
    }

class Environment:
    def __init__(self, ltlInput, agents, partitions, numTools, numWork):
        self.ltl = str(ltlInput)
        self.agents = agents
        self.partitions = partitions
        self.numTools = numTools
        self.numWork = numWork

        self.TranslateLTL(self.ltl)

    def ParseLTL(self, ltl):
        perenCount = 0
        startIndex = 0
        endIndex = 0
        toParse = Queue.PriorityQueue(maxsize=20)

        indexes = self.Indexing(self.ltl)

        operators = ["[]", "<>", "!", "U", "-->", "&&", "||"]

        operatorsParsed = {
            "always": False,
            "eventually": False,
            "negation": False,
            "until": False,
            "implication": False,
            "and": False,
            "or": False
        }
        operandsParsed = {
            "always": ([]),
            "eventually": ([]),
            "negation": ([]),
            "until": ([]),
            "implication": ([]),
            "and": ([]),
            "or": ([])
        }

        for operator in operatorsParsed.keys():
            if not indexes[operator]:
                operatorsParsed[operator] = True

        for operator in operatorsParsed:
            if not operatorsParsed[operator]:
                for index in indexes[operator]:
                    toParse.put((index, operator))

        alwaysIndex = []
        eventuallyIndex = []
        always = []
        eventually = []
        negateFlag = 0
        while not toParse.empty():
            index, operator = toParse.get()
            if operator == "negation":
                startIndex = index + 1
                while startIndex < len(ltl):
                    if "[" in ltl[startIndex]:
                        negateFlag = 1
                        break
                    elif "<" in ltl[startIndex]:
                        negateFlag = 1
                        break
                    elif "U" in ltl[startIndex]:
                        negateFlag = 1
                        break
                    elif "-" in ltl[startIndex]:
                        negateFlag = 1
                        break
                    elif "(" in ltl[startIndex]:
                        endIndex = startIndex + 1
                        while endIndex < len(ltl):
                            if "(" in ltl[endIndex]:
                                perenCount +=1
                                endIndex +=1
                            elif ")" in ltl[endIndex] and perenCount != 0:
                                perenCount -= 1
                                endIndex +=1
                            elif ")" in ltl[endIndex] and perenCount == 0:
                                operandsParsed["negation"].append((ltl[startIndex + 1:endIndex],True))
                                break
                            else:
                                endIndex +=1
                                break
                    elif ltl[startIndex] != " " and ltl[startIndex] != "":
                        operandsParsed["negation"].append((ltl[startIndex],True))
                        break
                    else:
                        startIndex += 1
            elif operator == "always":
                for startIndex in indexes["always"]:
                    while startIndex < len(ltl):
                        if "(" in ltl[startIndex]:
                            endIndex = startIndex + 1
                            while endIndex < len(ltl):
                                if "(" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount == 0:
                                    if negateFlag == 1:
                                        operandsParsed["always"].append((ltl[startIndex + 1:endIndex],True))
                                        negateFlag = 0
                                    else:
                                        operandsParsed["always"].append((ltl[startIndex + 1:endIndex],False))
                                    break
                                else:
                                    endIndex +=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            if negateFlag == 1:
                                operandsParsed["always"].append((ltl[startIndex],True))
                                negateFlag = 0
                            else:
                                operandsParsed["always"].append((ltl[startIndex],False))
                            break
                        else:
                            startIndex += 1
            elif operator == "eventually":
                for startIndex in indexes["eventually"]:
                    while startIndex < len(ltl):
                        if "(" in ltl[startIndex]:
                            endIndex = startIndex + 1
                            while endIndex < len(ltl):
                                if "(" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount == 0:
                                    if negateFlag == 1:
                                        operandsParsed["eventually"].append((ltl[startIndex + 1:endIndex],True))
                                        negateFlag = 0
                                    else:
                                        operandsParsed["eventually"].append((ltl[startIndex + 1:endIndex],False))
                                    break
                                else:
                                    endIndex +=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            if negateFlag == 1:
                                operandsParsed["eventually"].append((ltl[startIndex],True))
                                negateFlag = 0
                            else:
                                operandsParsed["eventually"].append((ltl[startIndex],False))
                            break
                        else:
                            startIndex += 1
            elif operator == "and":
                preOperand = ""
                postOperand = ""
                startIndex = index + 1
                for startIndex in indexes["and"]:
                    while startIndex < len(ltl):
                        if "(" in ltl[startIndex]:
                            endIndex = startIndex + 1
                            while endIndex < len(ltl):
                                if "(" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount == 0:
                                    postOperand = ltl[startIndex + 1:endIndex]
                                    break
                                else:
                                    endIndex +=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            postOperand = ltl[startIndex]
                            break
                        else:
                            startIndex += 1
                for startIndex in indexes["and"]:
                    startIndex = startIndex - 3
                    while startIndex > 0:
                        rospy.loginfo(ltl[startIndex])
                        if ")" in ltl[startIndex]:
                            endIndex = startIndex - 1
                            while endIndex > 0:
                                if ")" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex -=1
                                elif "(" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex -=1
                                elif "(" in ltl[endIndex] and perenCount == 0:
                                    preOperand = ltl[endIndex + 1:startIndex]
                                    break
                                else:
                                    endIndex -=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            preOperand = ltl[startIndex]
                            break
                        else:
                            startIndex -= 1
                if negateFlag == 1:
                    operandsParsed["and"].append(("{},{}".format(preOperand, postOperand),True))
                    negateFlag = 0
                else:
                    operandsParsed["and"].append(("{},{}".format(preOperand, postOperand),False))
            elif operator == "until":
                preOperand = ""
                postOperand = ""
                startIndex = index + 1
                for startIndex in indexes["until"]:
                    while startIndex < len(ltl):
                        if "(" in ltl[startIndex]:
                            endIndex = startIndex + 1
                            while endIndex < len(ltl):
                                if "(" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount == 0:
                                    postOperand = ltl[startIndex + 1:endIndex]
                                    break
                                else:
                                    endIndex +=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            postOperand = ltl[startIndex]
                            break
                        else:
                            startIndex += 1
                for startIndex in indexes["until"]:
                    startIndex = startIndex - 3
                    while startIndex > 0:
                        rospy.loginfo(ltl[startIndex])
                        if ")" in ltl[startIndex]:
                            endIndex = startIndex - 1
                            while endIndex > 0:
                                if ")" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex -=1
                                elif "(" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex -=1
                                elif "(" in ltl[endIndex] and perenCount == 0:
                                    preOperand = ltl[endIndex + 1:startIndex]
                                    break
                                else:
                                    endIndex -=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            preOperand = ltl[startIndex]
                            break
                        else:
                            startIndex -= 1
                if negateFlag == 1:
                    operandsParsed["until"].append(("{},{}".format(preOperand, postOperand),True))
                    negateFlag = 0
                else:
                    operandsParsed["until"].append(("{},{}".format(preOperand, postOperand),False))
            elif operator == "implication":
                preOperand = ""
                postOperand = ""
                startIndex = index + 1
                for startIndex in indexes["implication"]:
                    while startIndex < len(ltl):
                        if "(" in ltl[startIndex]:
                            endIndex = startIndex + 1
                            while endIndex < len(ltl):
                                if "(" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex +=1
                                elif ")" in ltl[endIndex] and perenCount == 0:
                                    postOperand = ltl[startIndex + 1:endIndex]
                                    break
                                else:
                                    endIndex +=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            postOperand = ltl[startIndex]
                            break
                        else:
                            startIndex += 1
                for startIndex in indexes["implication"]:
                    startIndex = startIndex - 4
                    while startIndex > 0:
                        rospy.loginfo(ltl[startIndex])
                        if ")" in ltl[startIndex]:
                            endIndex = startIndex - 1
                            while endIndex > 0:
                                if ")" in ltl[endIndex]:
                                    perenCount +=1
                                    endIndex -=1
                                elif "(" in ltl[endIndex] and perenCount != 0:
                                    perenCount -= 1
                                    endIndex -=1
                                elif "(" in ltl[endIndex] and perenCount == 0:
                                    preOperand = ltl[endIndex + 1:startIndex]
                                    break
                                else:
                                    endIndex -=1
                            break
                        elif ltl[startIndex] != " " and ltl[startIndex] != "":
                            preOperand = ltl[startIndex]
                            break
                        else:
                            startIndex -= 1
                if negateFlag == 1:
                    operandsParsed["implication"].append(("{},{}".format(preOperand, postOperand),True))
                    negateFlag = 0
                else:
                    operandsParsed["implication"].append(("{},{}".format(preOperand, postOperand),False))

            rospy.loginfo(operandsParsed)
            return operandsParsed

    def Indexing(self, ltl):
        indexes = {
            "always": [],
            "eventually": [],
            "negation": [],
            "until": [],
            "implication": [],
            "and": [],
            "or": []
        }

        if "[]" in ltl:
            for match in re.finditer("\[\]", ltl):
                indexes["always"].append(match.end())
        if "<>" in ltl:
            for match in re.finditer("\<\>", ltl):
                indexes["eventually"].append(match.end())
        if "!" in ltl:
            for match in re.finditer("\!", ltl):
                indexes["negation"].append(match.end())
        if "U" in ltl:
            for match in re.finditer("U", ltl):
                indexes["until"].append(match.end())
        if "-->" in ltl:
            for match in re.finditer("\-\-\>", ltl):
                indexes["implication"].append(match.end())
        if "&&" in ltl:
            for match in re.finditer("\&\&", ltl):
                indexes["and"].append(match.end())
        if "||" in ltl:
            for match in re.finditer("\|\|", ltl):
                indexes["or"].append(match.end())

        return indexes

    def TranslateLTL(self, ltl):
        '''
        Grammar:
	       ltl ::= opd | ( ltl ) | ltl binop ltl | unop ltl

        Operands (opd):
	       true, false, user-defined names, embedded expressions inside curly braces.

        Unary Operators (unop):
	       []	(the temporal operator always)
	          <>	(the temporal operator eventually)
	             ! 	(the boolean operator for negation)

        Binary Operators (binop):
	       U 	(the temporal operator strong until)
	          &&	(the boolean operator for logical and)
	             ||	(the boolean operator for logical or)
	                ->(the boolean operator for logical implication)

        finite set of operands: W = {w0,w1,...wn}
                                T = {t0,t1,...tm}
                                R = {r0,r1,...rq}
                                P = {p0,p1,...pl}

        '''
        parsedLTL = self.ParseLTL(self.ltl)

    '''
    for robot in agents:
        robot.mdp["states"] =
        robot.mdp["actions"] =
        robot.mdp["rewards"] =
        robot.mdp["model"] =
        '''
def GetDefaults():
    numAgents = 3
    numTools = 3
    numWork = 3
    partitions = 1
    return numAgents, numTools, numWork, partitions

def InitEnvironment(ltlInput, agents, partitions, numTools):
    env = Environment(ltlInput, agents, partitions, numTools)
    return env

def main():
    agents = {}
    i = 0

    pub = rospy.Publisher('robot_commands', String, queue_size=20)
    rospy.init_node('ltl_translator', anonymous=True)
    rate = rospy.Rate(1)

    numAgents, numTools, numWork, partitions = GetDefaults()

    ltlInput = rospy.get_param('/ltl_translation/ltl_input')

    while i < numAgents:
        agents["r{0}".format(i)] = Agent(numTools, numWork)
        i+=1

    env = Environment(ltlInput, agents, partitions, numTools, numWork)

    while not rospy.is_shutdown():
        rospy.get_time()
        pub.publish(ltlInput)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
