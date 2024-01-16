import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
trueVelocity = pd.read_csv(r"C:\Users\72419360\OneDrive - Sigma AB\Skrivbordet\GitHub\SensorFusionCpp\dataGroundTruth.csv")
dataMeasuredVelocity = pd.read_csv(r"C:\Users\72419360\OneDrive - Sigma AB\Skrivbordet\GitHub\SensorFusionCpp\dataMeasured.csv")
dataEstimatedVelcity = pd.read_csv(r"C:\Users\72419360\OneDrive - Sigma AB\Skrivbordet\GitHub\SensorFusionCpp\dataEstimated.csv")

# Plotting
plt.figure(figsize=(10, 6))
numberOfFigs = [2,3]


# Plot position_x and position_y
i = 1
plt.subplot(numberOfFigs[0], numberOfFigs[1], i)
plt.plot(trueVelocity['position_x'], trueVelocity['position_y'], label='True position')
plt.plot(dataEstimatedVelcity['position_x'], dataEstimatedVelcity['position_y'], label='Estimated position')

plt.title('Position over Time')
plt.xlabel('x position')
plt.ylabel('y position')
plt.legend()

# Plot velocity_x and velocity_y
i = i + 1
plt.subplot(numberOfFigs[0], numberOfFigs[1], i)
plt.plot(trueVelocity['time'], trueVelocity['velocity_x'], label='True X velocity')
plt.plot(dataEstimatedVelcity['time'], dataEstimatedVelcity['velocity_y'], label='Estimated X velocity')
plt.title('X velocity over Time')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

i = i + 1
plt.subplot(numberOfFigs[0], numberOfFigs[1], i)
plt.plot(trueVelocity['time'], abs(trueVelocity['velocity_x'] - dataEstimatedVelcity['velocity_x']), label='Error X velocity')
plt.title('Error X velocity over Time')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

i = i + 1
plt.subplot(numberOfFigs[0], numberOfFigs[1], i)
plt.plot(trueVelocity['time'], abs(trueVelocity['velocity_y'] - dataEstimatedVelcity['velocity_y']), label='Error Y velocity')
plt.title('Error Y velocity over Time')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

# Adjust layout and show the plot
plt.tight_layout()
plt.show()