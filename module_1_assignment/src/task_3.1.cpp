/*### Task 3: Sensor Library Design

- **Design a simple sensor library** that includes classes for different types of sensors:
  - `TemperatureSensor`: Class for handling temperature readings.
  - `DistanceSensor`: Class for handling distance measurements.

- **Use these classes in a main program** to simulate getting readings from sensors.

- **Create a single-class template** that can be used for multiple sensor types:
  - For `double` data
  - For `string` data
  - For `char` data*/




#include <iostream>
#include <string>

// Template class for a generic sensor
template <typename T>
class Sensor
{
public:
    // Constructor to initialize sensor type and value
    Sensor(std::string type, T value)
        : sensorType(type), sensorValue(value) {}

    // Method to get the sensor reading
    void getReading()
    {
        std::cout << sensorType << " Sensor Reading: " << sensorValue << std::endl;
    }

private:
    std::string sensorType; // Type of sensor (e.g., Temperature, Distance)
    T sensorValue;          // Value of the sensor reading
};

int main()
{
    // Create a Temperature Sensor (double data)
    Sensor<double> temperatureSensor("Temperature", 22.5);
    temperatureSensor.getReading();

    // Create a Distance Sensor (double data)
    Sensor<double> distanceSensor("Distance", 125.8);
    distanceSensor.getReading();

    // Create a Sensor for Name (string data)
    Sensor<std::string> nameSensor("Name", "Robot Module");
    nameSensor.getReading();

    // Create a Sensor for Status (char data)
    Sensor<char> statusSensor("Status", 'A'); // Example: 'A' for Active
    statusSensor.getReading();

    return 0;
}
