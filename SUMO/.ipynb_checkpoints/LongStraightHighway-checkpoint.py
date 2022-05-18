import os
import sys
import optparse
import PIDController

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


##Temporary class for measuring CO2 emissions and fuel consumption 
class Car:
    def __init__(self, id):
        self.id = id
        self.CO2emission = 0
        self.FuelConsumption = 0
        
    def increaseCO2(self, emission, time):
        self.CO2emission += emission*time
        
    def increaseFuelConsumption(self, consumption, time):
        self.FuelConsumption += consumption*time
        
listOfCars = []
for x in range(10):
    listOfCars.append(Car("veh"+str(x)))
    
#-----------------------------------    


#control loop for TraCI
def run():    
    step = 0
    timeNew = traci.simulation.getTime()
    timeOld = 0
    
    controller = PIDController(300.0, 250.0, 250.0, 1.0, 0, 10000, 1.0, 20, 20)
    
    while traci.simulation.getMinExpectedNumber() > 0:    #runs while there are cars in the network
        traci.simulationStep()
        
        vehList = traci.vehicle.getIDList()
        maxSpeed = 10
        if(len(vehList) != 0):
            for currVeh in vehList:
                #Control
                traci.vehicle.changeLane(currVeh, 2, 10)    #move all cars to one lane, (carID, lane, duration)
                traci.vehicle.setSpeed(currVeh, maxSpeed)   #make following cars faster so they close the distance
                traci.vehicle.setTau(currVeh, 0.1)         #sets minimum following distance in seconds
                maxSpeed += 5

                #Emission 
                stepLength = timeNew-timeOld
                emissionCO2InStep = traci.vehicle.getCO2Emission(currVeh)
                currIdx = [car.id for car in listOfCars].index(currVeh)
                listOfCars[currIdx].increaseCO2(emissionCO2InStep, stepLength)
                consumptionInStep = traci.vehicle.getFuelConsumption(currVeh)
                listOfCars[currIdx].increaseFuelConsumption(consumptionInStep, stepLength)
        
        step += 1
        timeOld = timeNew
        timeNew = traci.simulation.getTime()    #get time after simulation step
        
        
    traci.close()
    #sys.stdout.flush()

#main entry point
if __name__ == "__main__":
    options = get_options()
    
    #check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    
    #traci tarts sumo as a subprocess and then this scipt connects and runs
    traci.start([sumoBinary, "-c", "LongStraightHighway\LongStraightHighway.sumocfg",
                #"--tripinfo-output", "tripinfo.xml",    #writes out relevant data about car's trip
                #"--lanechange-output", "lanechanges.xml",   #writes out all lanechanges to the given file 
                "--emission-output", "baseEmissions.xml",    #writes out emissions by each car in each step
                "--lateral-resolution", "10.0",    #simulates sublanes
                "-d", "150",   #sets delay in gui
                "-b", "0",     #sets beginning simulation time
                "-e", "1000"   #sets ending simulation time
                ])
    #sublane and timestamp comes here
    
    run()