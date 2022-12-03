import os
import sys
import optparse

from xml.dom import minidom
from enum import Enum

#import some python modules from the SUMO_HOME/tools dir
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary   #checks for binary environment vars
import traci

import PIDController

##-----------------------

# Constants for the platoon
MAX_PARTICIPANTS = 10
MAX_TIME_LEADING = 50   # TODO: try different values (value used for simulations was 50)
PLATOONING_SPEED = 30   # TODO: try different values (value used for simulations was 30)
SLOW_FACTOR = 0.91       # Optimal value based on simulations is 0.91
PREF_LANE_IDX = 1

## Constantly for the simulation
LOOK_AHEAD_DIST = 1000
PREF_SPACE_DIST = PLATOONING_SPEED/500  # TODO: Try reseting to 0.1
PREF_TIME_DIST = 0.0001
SAVINGS_PERCENTAGE = 0.1                # TODO: Set to desired amount (should be around 0.05-0.15)
CATCH_UP_MULTIPLIER = 1.2
LANE_DURATION = 10
MULTIPLIER_WITH_FULL_AIR_RESISTANCE = 1
MULTIPLIER_WITH_REDUCED_AIR_RESISTANCE = MULTIPLIER_WITH_FULL_AIR_RESISTANCE-SAVINGS_PERCENTAGE
OPEN_GAP_DURATION = 0.1
CHANGE_RATE = 5
MAX_DECEL = -1

class CONFIG_TO_RUN(Enum):
    LONG_STRAIGHT_HIGHWAY = "SUMO\LongStraightHighway\LongStraightHighway.sumocfg"
    LONG_CIRCLE_ROAD = "SUMO\LongCircleRoad\LongCircleRoad.sumocfg"

class EMISSION_FILES(Enum):
    UNCONTROLLED = "simulationResults\emissionBaseValues_Uncontrolled.xml"
    CONTROLLED = "simulationResults\emissionBaseValues_Controlled.xml"
    CONTROLLED_SLOW_TESTING = "simulationResults\emissionBaseValues_Controlled_SlowFactorAt_" + str('%g'%(SLOW_FACTOR*100)) + "_Percentage.xml"

class SCALING_EMISSION_FILES(Enum):
    UNCONTROLLED = "simulationResults\emissionScalingValues_Uncontrolled.xml"
    CONTROLLED = "simulationResults\emissionScalingValues_Controlled.xml"
    CONTROLLED_SLOW_TESTING = "simulationResults\emissionScalingValues_Controlled_SlowFactorAt_" + str('%g'%(SLOW_FACTOR*100)) + "_Percentage.xml"

class WITH_DELAY(Enum):
    NORMAL = "150"
    SHORT = "10"
    NONE = "0"

class ZOOM_LEVEL(Enum):
    LONG_STRAIGHT_HIGHWAY = 9500
    LONG_CIRCLE_ROAD = 4500

class NUM_OF_SUB_LANES(Enum):
    NORMAL = "10"
    MINIMAL = "2"
    NONE = "0"

class RUN_WITH_GUI(Enum):
    YES = False
    NO = True

class IS_CONTROLLED(Enum):
    YES = True
    NO = False

class Preset:
    def __init__(self, configName, zoomLevel, runWithGUI, isControlled, emissionBaseFileName, emissionScalingFileName, withDelay, numOfSubLanes, runUntilTime, simulationStepLength):
        self.configName = configName
        self.zoomLevel = zoomLevel
        self.runWithGUI = runWithGUI
        self.isControlled = isControlled
        self.emissionBaseFileName = emissionBaseFileName
        self.emissionScalingFileName = emissionScalingFileName
        self.withDelay = withDelay
        self.numOfSubLanes = numOfSubLanes
        self.runUntilTime = runUntilTime
        self.simulationStepLength = simulationStepLength

PRESET = Preset(
    configName = CONFIG_TO_RUN.LONG_CIRCLE_ROAD.value,
    zoomLevel = ZOOM_LEVEL.LONG_CIRCLE_ROAD.value,
    runWithGUI = RUN_WITH_GUI.NO,
    isControlled = IS_CONTROLLED.YES,
    emissionBaseFileName = EMISSION_FILES.CONTROLLED.value,
    emissionScalingFileName = SCALING_EMISSION_FILES.CONTROLLED.value,
    withDelay = WITH_DELAY.NONE.value,
    numOfSubLanes = NUM_OF_SUB_LANES.MINIMAL.value,
    runUntilTime = float(1000),
    simulationStepLength = 0.2
)

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=PRESET.runWithGUI.value,
                         help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

class Car:
    def __init__(self, id):
        self.id = id
        #self.PID = PIDController.PIDController(3.0, 2.5, 2.5, 1.0, -10000, 10000, 1.0, 20, 20)
        self.isFinished = False
        self.isLeading = False
        self.isFollowing = False
        self.consumptionValue = []
        
    def setLaneAndSpeed(self, laneID, duration, speed):
        traci.vehicle.changeLane(self.id, laneID, duration)
        traci.vehicle.setSpeed(self.id, speed)
        
    def setLaneAndOpenGap(self, laneID, laneDuration, prefTimeDist, prefSpaceDist, openGapDuration, changeRate, maxDecel):
        traci.vehicle.changeLane(self.id, laneID, laneDuration)
        traci.vehicle.setTau(self.id, prefSpaceDist)
        if traci.vehicle.getLeader(self.id, LOOK_AHEAD_DIST) != None:
            [currLeaderID, currLeaderDist] = traci.vehicle.getLeader(self.id, LOOK_AHEAD_DIST)
            traci.vehicle.openGap(self.id, prefTimeDist, prefSpaceDist, openGapDuration, changeRate, maxDecel, currLeaderID)
            if traci.vehicle.getSpeed(self.id) < traci.vehicle.getSpeed(currLeaderID):  #Remove this for openGap error testing
                traci.vehicle.setSpeed(self.id, traci.vehicle.getSpeed(currLeaderID) * CATCH_UP_MULTIPLIER) #If distance in time with car in front is bigget than preferred, set speed to catchUpMultiplier
            
    def checkLeftForCarInFront(self, carInFront):   # Checks if the car in front of us in the platooning list is ahead of us in the left. Used for moving to the back of the platoon.
        leftLeaders = traci.vehicle.getLeftLeaders(self.id)
        for leftLeader in leftLeaders:
            if (leftLeader[0] == carInFront.id): # Car in fron of us in the list is ahead of us
                return True
        return False

    def checkIfIsChangingLanes(self):  # Check if the car is currently changing lanes
        lateralSpeed = traci.vehicle.getLateralSpeed(self.id)
        if (lateralSpeed == 0):
            return False
        return True

        
class Platoon:
    def __init__(self, maxParticipants, maxTimeLeading, platoonSpeed, slowFactor, prefLaneIdx):
        self.leader = None
        self.cars = []
        self.maxParticipants = maxParticipants
        self.maxTimeLeading = maxTimeLeading
        self.platoonSpeed = platoonSpeed
        self.prefLaneIdx = prefLaneIdx
        self.slowFactor = slowFactor
        self.prefTimeDist = PREF_TIME_DIST
        self.savingPercentage = SAVINGS_PERCENTAGE
        
    def addCar(self, car):
        if(len(self.cars)+1 <= self.maxParticipants):   # Checks if platoon capacity is at its maximum, only adds new cars if they can fit
            if(self.leader == None):    # If there is no leader, the newly added car becomes the leader
                self.leader = car
                self.leader.isLeading = True
                self.leader.isFollowing = False
            else:                       # Else it is just added to the end of the list of the following cars
                self.cars.append(car)
                car.isLeading = False
                car.isFollowing = True
                
    def removeCarByID(self, carID): # Uses the ID of cars, as the given Car has a different ID in memory
        if self.leader.id == carID: # Wanting to delete Leader car
            self.leader.isLeader = False    # Leading car is no longer the Leader
            if len(self.cars) > 0:          # The Platoon still has other cars
                self.leader = self.cars.pop(0)   # First car from the following list becomes the Leader
                self.leader.isLeader = True      # Leader values is flipped for new Leader
                self.leader.isFollowing = False  # New Leader doesn't follow anyone
            else:                           # The Platoon has no cars left
                self.leader.isFinished = True
                self.leader = None
                print("Platoon is empty")
        else:                       # Wanting to delete non-leading car. Search for the car and delete it. No need to set new leader car for the one following it, it works based on order in a list
            for tempCar in self.cars:
                if tempCar.id == carID:
                    self.cars.pop(self.cars.index(tempCar))
            
    def controlCars(self, isControlled):
        if (isControlled): 
            if traci.simulation.getTime() % self.maxTimeLeading == 0 and len(self.cars) > 0:
                self.changeLeader()
            
            if self.leader != None: # If there is a Leader car, set it's values
                self.leader.setLaneAndSpeed(self.prefLaneIdx, LANE_DURATION, self.platoonSpeed)
                self.leader.consumptionValue.append(MULTIPLIER_WITH_FULL_AIR_RESISTANCE)
            
            for car in self.cars:
                if car.isFollowing:     # Car is following someone
                    car.setLaneAndOpenGap(self.prefLaneIdx, LANE_DURATION, self.prefTimeDist, PREF_SPACE_DIST, OPEN_GAP_DURATION, CHANGE_RATE, MAX_DECEL)
                    car.consumptionValue.append(MULTIPLIER_WITH_REDUCED_AIR_RESISTANCE)
                else:                   # Car is not following another
                    carInFront = self.cars[self.cars.index(car)-1] # The Car before ours in the list
                    gotBehindCIF = False
                    if car.checkLeftForCarInFront(carInFront):      # Checks if the car in front of us in the lists is really in front of us
                        car.setLaneAndSpeed(self.prefLaneIdx, LANE_DURATION, self.platoonSpeed*self.slowFactor)    # Car is in front of us, we can change to it's lane and follow it normally
                        gotBehindCIF = True
                    else:                                           # Car that should be in front of us is behind us
                        if traci.vehicle.getLaneIndex(car.id) == self.prefLaneIdx:  # Car is in the preferred lane, but should not be
                            car.setLaneAndSpeed(self.prefLaneIdx-1, LANE_DURATION, self.platoonSpeed)   # Car has to change to an outer layer to slow down and go to the back
                        else:                                                       # Car is in the outer lane, where it is moving slowly to the back
                            if car.checkIfIsChangingLanes() == False:
                                car.setLaneAndSpeed(self.prefLaneIdx-1, LANE_DURATION, self.platoonSpeed*self.slowFactor)   # Car can slow without making others also slow
                            else:
                                car.setLaneAndSpeed(self.prefLaneIdx-1, LANE_DURATION, self.platoonSpeed)   # Car cant yet slow without affecting others

                    if gotBehindCIF:    # Car got back to the end of the line, and is in the right lane
                        car.isFollowing = True
                    car.consumptionValue.append(MULTIPLIER_WITH_FULL_AIR_RESISTANCE)
        else:   # Simulates traffic happening without suggested platoon methods. We assume that the cars are far enough from each other that no reduction in air resistance is present
            self.leader.setLaneAndSpeed(self.prefLaneIdx, LANE_DURATION, self.platoonSpeed)
            self.leader.consumptionValue.append(MULTIPLIER_WITH_FULL_AIR_RESISTANCE)
            for car in self.cars:
                car.setLaneAndSpeed(self.prefLaneIdx, LANE_DURATION, self.platoonSpeed)
                car.consumptionValue.append(MULTIPLIER_WITH_FULL_AIR_RESISTANCE)
    
    def changeLeader(self): # Changes the leader of the platoon, adding the former leader to the end of the following cars, and the first from the following list becomes the new Leader
        self.cars.append(self.leader)
        self.leader.isLeader = False
        self.leader = self.cars.pop(0)
        self.leader.isLeader = True
        self.leader.isFollowing = False


#control loop for TraCI
def run(): 
    step = 0
        
    listOfCars = []

    bigPlatoon = Platoon(MAX_PARTICIPANTS, MAX_TIME_LEADING, PLATOONING_SPEED, SLOW_FACTOR, PREF_LANE_IDX)
    
    while traci.simulation.getMinExpectedNumber() > 0:    # runs while there are cars in the network

        if ( PRESET.configName == CONFIG_TO_RUN.LONG_CIRCLE_ROAD.value and traci.simulation.getTime() > PRESET.runUntilTime ):  # Stops the simulation when infinite loop is used
            break
        
        traci.simulationStep()
                
        departedList = traci.simulation.getDepartedIDList() # list of vehicles that departed in this step
        for currVeh in departedList:    # Goes through the list of departed cars, if they are not in the platoon, it adds them
            isTracked = False
            for i in listOfCars:
                if currVeh == i.id:
                    isTracked = True
            if isTracked == False:
                newCar = Car(currVeh)
                listOfCars.append(newCar)
                bigPlatoon.addCar(newCar)
                
        arrivedList = traci.simulation.getArrivedIDList() # vehicles which have arrived to their goal
        for currVeh in arrivedList:
            for i in listOfCars:
                if currVeh == i.id:
                    bigPlatoon.removeCarByID(i.id)
        
        if (PRESET.runWithGUI.value == RUN_WITH_GUI.YES.value):             # Only call GUI method if running with GUI
            vehList = traci.vehicle.getIDList()
            if('veh0' in vehList):
                    traci.gui.trackVehicle('View #0', 'veh0')               # Sets view focused to "veh0"
                    traci.gui.setZoom('View #0', PRESET.zoomLevel)          # PARAM: Sets the zoom level to given value, 9500 for straight, 4500 for circle
        
        bigPlatoon.controlCars(PRESET.isControlled.value)    # Controls the cars in the platoon
                
        # Simulation event examples
        # if traci.simulation.getTime() == 10:  # Simulation event for removing one car
        #     bigPlatoon.removeCarByID(listOfCars[4].id)
        # if traci.simulation.getTime() == 20:    # First leader change in the program
        #     bigPlatoon.changeLeader()
                
        step += 1
        
    traci.close()

    createConsumptionXML(listOfCars)


def createConsumptionXML(listOfCars):
    ''' pref xml form:
    <scaling-export>
        <timestep time = "0.00">
            <vehicle id="veh0" scaling=1.0 />
            <vehicle id="veh1" scaling=0.85 />
            ...
        <timestep/>
    <scaling-export>
    '''
    base = minidom.Document()
    root = base.createElement("scaling-export")
    base.appendChild(root)

    longestScaleList = 0    # length of longest consumption value table
    for car in listOfCars:
        if len(car.consumptionValue) > longestScaleList:
            longestScaleList = len(car.consumptionValue)

    for i in range(longestScaleList):
        timestepChild = base.createElement('timestep')
        timestepChild.setAttribute("time", str("{:.2f}".format(i*PRESET.simulationStepLength))) #i/10
        root.appendChild(timestepChild)
        for car in listOfCars:
            carChild = base.createElement('vehicle')
            carChild.setAttribute("id", car.id)
            if(i < len(car.consumptionValue)):
                carChild.setAttribute("scaling", str("{:.2f}".format(car.consumptionValue[i])))
            else:
                carChild.setAttribute("scaling", str("{:.2f}".format(float(0))))
            timestepChild.appendChild(carChild)

    xmlString = base.toprettyxml(indent = "\t")
    savePathFile = PRESET.emissionScalingFileName
    with open(savePathFile, "w") as f:
        f.write(xmlString)
        

# main entry point
if __name__ == "__main__":
    options = get_options()
    
    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # traci tarts sumo as a subprocess and then this scipt connects and runs
    traci.start([sumoBinary,
                "-c", PRESET.configName,                                    # sets config file for sumo to use
                # "--tripinfo-output", "simulationResults/tripinfo.xml",    # writes out relevant data about car's trip. Eg.: distance traveled
                # "--tripinfo-output.write-unfinished",                     # changes behavior to write out unfinished trips too
                #"--lanechange-output", "lanechanges.xml",                  # writes out all lanechanges to the given file 
                "--emission-output", PRESET.emissionBaseFileName,           # writes out emissions by each car in each step
                "--step-length", str(PRESET.simulationStepLength),          # length of each simulation step
                "--lateral-resolution", PRESET.numOfSubLanes,               # PARAM: simulates sublanes
                "--collision.mingap-factor", "0",   # only physical collisions are registered
                "--collision.action", "warn",       # cars wont teleport because of collision, only print a warning
                "-d", PRESET.withDelay,             # PARAM: sets delay in gui, optimal for viewing is ~150
                "-b", "0",          # sets beginning simulation time
                "-e", "1000",       # sets ending simulation time
                "--no-warnings"
                ])
    
    run()