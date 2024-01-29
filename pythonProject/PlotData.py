import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
#dataGroundTruth = pd.read_csv(r"C:\Users\72419360\Downloads\GitHub\SensorFusionCpp\dataGroundTruth.csv")
#dataMeasured = pd.read_csv(r"C:\Users\72419360\Downloads\GitHub\SensorFusionCpp\dataMeasured.csv")
#dataEstimated = pd.read_csv(r"C:\Users\72419360\Downloads\GitHub\SensorFusionCpp\dataEstimated.csv")
dataGroundTruth = pd.read_csv(r"C:\Users\Jonas\Documents\GitHub\SensorFusionCpp\dataGroundTruth.csv")
dataMeasured = pd.read_csv(r"C:\Users\Jonas\Documents\GitHub\SensorFusionCpp\dataMeasured.csv")
dataEstimated = pd.read_csv(r"C:\Users\Jonas\Documents\GitHub\SensorFusionCpp\dataEstimated.csv")



# Plotting
plt.figure(figsize=(10, 6))

"""
numberOfFigs = [1,2]
variable = 'ax'
plt.subplot(numberOfFigs[0], numberOfFigs[1], 1)
plt.plot(dataGroundTruth['time'], dataGroundTruth[variable], label='GroundTruth ' + variable)
plt.legend()
plt.subplot(numberOfFigs[0], numberOfFigs[1], 2)
plt.plot(dataEstimated['time'], dataEstimated[variable], label='Estimated ' + variable)
plt.legend()
"""


numberOfFigs = [3,3]
plotstuff1 = ['px', 'py', 'vx', 'vy', 'ax', 'ay']
for i, variable in enumerate(plotstuff1):
    plt.subplot(numberOfFigs[0], numberOfFigs[1], i+1)
    #plt.plot(dataGroundTruth['time'], dataGroundTruth[variable], label='GroundTruth ' + variable)
    #plt.plot(dataEstimated['time'], dataEstimated[variable], label='Estimated ' + variable)
    plt.plot(dataGroundTruth['time'], abs(dataGroundTruth[variable]-dataEstimated[variable]), label='error ' + variable)
    plt.title(variable + ' over Time')
    plt.legend()

#plotstuff2 = ['px', 'py', 'vx', 'vy', 'ax', 'ay']
#for i, variable in enumerate(plotstuff2):
#    plt.subplot(numberOfFigs[0], numberOfFigs[1], i+7)
#    plt.plot(dataMeasured['time'], dataMeasured[variable], label='GroundTruth ' + variable)
#    plt.plot(dataEstimated['time'], dataEstimated[variable], label='Estimated ' + variable)
#    plt.plot(dataGroundTruth['time'], abs(dataGroundTruth[variable]-dataEstimated[variable]), label='error ' + variable)
#    plt.title(variable + ' over Time')
#    plt.legend()



# Adjust layout and show the plot
plt.tight_layout()
plt.show()