#include <iostream>
#include <string>

// Template class for generic sensors
template <typename T>
class Sensor {
private:
    std::string sensorType;
    T sensorValue;

public:
    // Constructor to initialize sensor type
    Sensor(std::string type) : sensorType(type) {}

    // Method to set a sensor reading
    void setReading(T value) {
        sensorValue = value;
    }

    // Method to get a sensor reading
    void getReading() const {
        std::cout << sensorType << " Sensor Reading: " << sensorValue << std::endl;
    }
};

// Derived class for temperature sensor
class TemperatureSensor : public Sensor<double> {
public:
    TemperatureSensor() : Sensor<double>("Temperature") {}
};

// Derived class for distance sensor
class DistanceSensor : public Sensor<double> {
public:
    DistanceSensor() : Sensor<double>("Distance") {}
};

int main() {
    // Create a TemperatureSensor object
    TemperatureSensor tempSensor;
    tempSensor.setReading(22.5); // Setting temperature value
    tempSensor.getReading();    // Getting temperature value

    // Create a DistanceSensor object
    DistanceSensor distSensor;
    distSensor.setReading(125.8); // Setting distance value
    distSensor.getReading();      // Getting distance value

    // Using the generic Sensor template for string data
    Sensor<std::string> stringSensor("Name");
    stringSensor.setReading("Robot Sensor Module");
    stringSensor.getReading();

    // Using the generic Sensor template for char data
    Sensor<char> charSensor("Status");
    charSensor.setReading('A'); // Example: 'A' for Active
    charSensor.getReading();

    return 0;
}
