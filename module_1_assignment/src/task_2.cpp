/*### Task 2: Simulating Sensor Readings

- **Create a C++ program** that represents a robot equipped with temperature and distance sensors.

- **Simulate sensor readings** with hardcoded values:
  - Example for temperature: `Temperature: 20°C`
  - Example for distance: `Distance: 100cm`

- **Print these values** to the console with appropriate descriptions.*/






#include <iostream>
#include <string>

namespace Sensors
{
    namespace T
    {
        // Method to get temperature reading
        void readTemperature(int temperatureValue)
        {
            std::cout << "Temperature Sensor Reading: " << temperatureValue << "°C" << std::endl;
        }
    }

    namespace D
    {
        // Method to get distance reading
        void readDistance(int distanceValue)
        {
            std::cout << "Distance Sensor Reading: " << distanceValue << "cm" << std::endl;
        }
    }
}

int main()
{
    // Hardcoded values for sensor readings
    int temperatureValue = 25; // Hardcoded temperature value
    int distanceValue = 150;   // Hardcoded distance value

    // Using Temperature Sensor
    std::cout << "Temperature Sensor Operations:" << std::endl;
    Sensors::T::readTemperature(temperatureValue);

    // Using Distance Sensor
    std::cout << "\nDistance Sensor Operations:" << std::endl;
    Sensors::D::readDistance(distanceValue);

    return 0;
}
