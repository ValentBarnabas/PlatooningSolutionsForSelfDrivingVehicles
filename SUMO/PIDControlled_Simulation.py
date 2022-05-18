import os
import sys
import optparse

import pandas as pd
from xml.dom import minidom

#import some python modules from the SUMO_HOME/tools dir
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary   #checks for binary environment vars
import traci

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False,
                         help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

##-----------------------
import PIDController

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
        if traci.vehicle.getLeader(self.id, 1000) != None:
            [currLeaderID, currLeaderDist] = traci.vehicle.getLeader(self.id, 1000)
            traci.vehicle.openGap(self.id, prefTimeDist, prefSpaceDist, openGapDuration, changeRate, maxDecel, currLeaderID)
            if traci.vehicle.getSpeed(self.id) < traci.vehicle.getSpeed(currLeaderID):  #Remove this for openGap error testing
                traci.vehicle.setSpeed(self.id, traci.vehicle.getSpeed(currLeaderID)*1.2)
            #If distance in time with car in front is bigget than preferred, set speed to *1.2
            
    def checkLeftForLeader(self, carInFront):
        leftLeaders = traci.vehicle.getLeftLeaders(self.id)
        for leaders in leftLeaders:
            if (leaders[0] == carInFront.id): ##car in fron of us in the list is ahead of us
                return True
        return False
    
    def checkForLeaderInPrefLine(self, carInFront):
        if traci.vehicle.getLeader(self.id, 1000) != None:
            [currLeaderID, currLeaderDist] = traci.vehicle.getLeader(self.id, 1000)
            if currLeaderID == carInFront:
                return True
        return False
        
class Platoon:
    def __init__(self, maxParticipants, maxTimeLeading, platoonSpeed, slowFactor, prefLaneIdx):
        self.leader = None
        self.cars = []
        self.maxParticipants = maxParticipants
        self.maxTimeLeading = maxTimeLeading
        self.platoonSpeed = platoonSpeed
        self.prefLaneIdx = prefLaneIdx
        self.slowFactor = slowFactor
        self.prefTimeDist = 0.0001
        self.savingPercentage = 0.15
        
    def addCar(self, car):
        if(len(self.cars)+1 <= self.maxParticipants):
            if(self.leader == None):
                self.leader = car
                self.leader.isLeading = True
                self.leader.isFollowing = False
            else:
                self.cars.append(car)
                car.isFollowing = True
                car.isLeading = False
                
    def removeCarByID(self, carID): #a jarmuvek ID-jat hasznalja azonositashoz, mivel a megkapott jarmu mas memoria beli cimmel rendelkezik
        if self.leader.id == carID:
            self.leader.isLeader = False     #vezeto mar nem vezeto
            if len(self.cars) > 0:
                #Platoon still has other cars
                self.leader = self.cars.pop(0)   #vezeto az utana kovetkezo jarmu lesz
                self.leader.isLeader = True      #uj vezeto lesz a vezeto
                self.leader.isFollowing = False  #uj vezeto nem kovet senkit
            else:
                #Platoon has no cars left
                self.leader.isFinished = True
                self.leader = None
                print("Platoon is empty")
        else:  #search for the car and delete it
            for tempCar in self.cars:
                if tempCar.id == carID:
                    self.cars.pop(self.cars.index(tempCar))
            
    def controlCars(self):
        #control
        if self.leader != None:
            self.leader.setLaneAndSpeed(self.prefLaneIdx, 10, self.platoonSpeed)
            self.leader.consumptionValue.append(1)
        
        for car in self.cars:
            #Car is following another
            if car.isFollowing:
                #Reset: prefSpaceDist = 0.1
                car.setLaneAndOpenGap(laneID=self.prefLaneIdx, laneDuration=10, prefTimeDist=self.prefTimeDist, prefSpaceDist=self.platoonSpeed/500, openGapDuration=0.1, changeRate=5, maxDecel=-1)
                car.consumptionValue.append(1-self.savingPercentage)
            #Car is not following another
            else:
                carInFront = self.cars[self.cars.index(car)-1] #car before ours in the list
                gotBehindCIF = False
                if car.checkLeftForLeader(carInFront):
                    car.setLaneAndSpeed(self.prefLaneIdx, 10, self.platoonSpeed*self.slowFactor)
                    gotBehindCIF = True
                elif car.checkForLeaderInPrefLine(carInFront):
                    car.setLaneAndSpeed(self.prefLaneIdx, 10, self.platoonSpeed*self.slowFactor) ##remove self.slowfactor
                else:
                    if traci.vehicle.getLaneIndex(car.id) == self.prefLaneIdx:
                        car.setLaneAndSpeed(self.prefLaneIdx-1, 10, self.platoonSpeed)
                    else:
                        car.setLaneAndSpeed(self.prefLaneIdx-1, 10, self.platoonSpeed*self.slowFactor)
                    
                #ha vegigment es a helyen van, es mar a megfelelo savban van, isFollowing = true
                if gotBehindCIF:
                    car.isFollowing = True
                car.consumptionValue.append(1)
    
    def changeLeader(self):
        self.cars.append(self.leader)
        self.leader.isLeader = False
        self.leader = self.cars.pop(0)
        self.leader.isLeader = True
        self.leader.isFollowing = False

#control loop for TraCI
def run(): 
    step = 0
    
    prefDistance = 0.1
    
    listOfCars = []
    
    maxParticipants = 10
    maxTimeLeading = 10
    platoonSpeed = 30
    slowFactor = 0.8
    prefLaneIdx = 2

    bigPlatoon = Platoon(maxParticipants, maxTimeLeading, platoonSpeed, slowFactor, prefLaneIdx)
    
    while traci.simulation.getMinExpectedNumber() > 0:    #runs while there are cars in the network
        traci.simulationStep()
        
        departedList = traci.simulation.getDepartedIDList() #list of vehicles that departed in this step
        for currVeh in departedList:
            isTracked = False
            for i in listOfCars:
                if currVeh == i.id:
                    isTracked = True
            if isTracked == False:
                newCar = Car(currVeh)
                listOfCars.append(newCar)
                bigPlatoon.addCar(newCar)
                
        arrivedList = traci.simulation.getArrivedIDList() #vehicles which have to be set to finished
        for currVeh in arrivedList:
            for i in listOfCars:
                if currVeh == i.id:
                    bigPlatoon.removeCarByID(i.id)
        
        #getTime - can be used for tracking the lead of each car
        vehList = traci.vehicle.getIDList()
        if('veh0' in vehList):
                traci.gui.trackVehicle('View #0', 'veh0')
                traci.gui.setZoom('View #0', 9500)
        
        bigPlatoon.controlCars()
        
        
        #Simulation events
        # if traci.simulation.getTime() == 10:
        #     bigPlatoon.removeCarByID(listOfCars[4].id)
        
        if traci.simulation.getTime() == 20:
            bigPlatoon.changeLeader()
        if traci.simulation.getTime() % 50 == 0:
            bigPlatoon.changeLeader()
                
        step += 1
        
    traci.close()

    createConsumptionXML(listOfCars)

    #sys.stdout.flush()

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

    longestScaleList = 0    #length of longest consumption value table
    for car in listOfCars:
        if len(car.consumptionValue) > longestScaleList:
            longestScaleList = len(car.consumptionValue)

    for i in range(longestScaleList):
        timestepChild = base.createElement('timestep')
        timestepChild.setAttribute("time", str("{:.2f}".format(i/10)))
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
    savePathFile = "emissionScaleValues.xml"
    with open(savePathFile, "w") as f:
        f.write(xmlString)

#main entry point
if __name__ == "__main__":
    options = get_options()
    
    #check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    
    #traci tarts sumo as a subprocess and then this scipt connects and runs
    sumocfgURL = "PlatooningSolutionsForSelfDrivingVehicles\SUMO\LongStraightHighway\LongStraightHighway.sumocfg"
    traci.start([sumoBinary, "-c", sumocfgURL,
                #"--tripinfo-output", "tripinfo.xml",    #writes out relevant data about car's trip
                #"--lanechange-output", "lanechanges.xml",   #writes out all lanechanges to the given file 
                "--emission-output", "emissionBaseValues.xml",    #writes out emissions by each car in each step
                "--step-length", "0.1",    #length of each simulation step
                "--lateral-resolution", "10.0",    #simulates sublanes
                "--collision.mingap-factor", "0", #only physical collisions are registered
                "--collision.action", "warn",     #cars wont teleport because of collision, only print a warning
                "-d", "150",   #sets delay in gui
                "-b", "0",     #sets beginning simulation time
                "-e", "1000"   #sets ending simulation time
                ])
    #sublane and timestamp comes here
    
    run()