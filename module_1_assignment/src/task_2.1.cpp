#include <iostream>
#include <string>

class TemperatureSensor
{
public:
    // Constructor
    TemperatureSensor(const std::string &name, int temperature)
        : sensorName(name), temperatureValue(temperature) {}

    // Method to get temperature reading
    void readTemperature()
    {
        std::cout << sensorName << " Reading: " << temperatureValue << "°C" << std::endl;
    }

private:
    std::string sensorName; // Name of the sensor
    int temperatureValue;   // Hardcoded temperature value
};

class DistanceSensor
{
public:
    // Constructor
    DistanceSensor(const std::string &name, int distance)
        : sensorName(name), distanceValue(distance) {}

    // Method to get distance reading
    void readDistance()
    {
        std::cout << sensorName << " Reading: " << distanceValue << "cm" << std::endl;
    }

private:
    std::string sensorName; // Name of the sensor
    int distanceValue;      // Hardcoded distance value
};

class Robot
{
public:
    // Constructor
    Robot(const std::string &name)
        : robotName(name),
          tempSensor("Temperature Sensor", 20),
          distSensor("Distance Sensor", 100) {}

    // Simulate sensor readings
    void simulateSensors()
    {
        std::cout << "Simulating Sensor Readings for " << robotName << ":\n";
        tempSensor.readTemperature();
        distSensor.readDistance();
    }

private:
    std::string robotName;        // Name of the robot
    TemperatureSensor tempSensor; // Temperature sensor
    DistanceSensor distSensor;    // Distance sensor
};

int main()
{
    // Create a robot object
    Robot myRobot("Robo-Sense");

    // Simulate sensor readings
    myRobot.simulateSensors();

    return 0;
}
