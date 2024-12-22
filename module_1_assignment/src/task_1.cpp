/*### Task 1: Robot Class Implementation

- **Define a `Robot` class** with the following attributes:
  - `name`: The name of the robot.
  - `speed`: The speed of the robot.
  - **Physical Attributes:**
    - `weight`: The weight of the robot.
    - `size`: The size of the robot.
    - `number_of_sensors`: The number of sensors the robot has.

- **Methods for moving the robot:**
  - `moveForward()`: Simulate the robot moving forward.
  - `moveBackward()`: Simulate the robot moving backward.
  - `stop()`: Simulate stopping the robot.

- **Instantiate a `Robot` object** and simulate actions by invoking its methods.

- **Use namespaces** to define different robots. Ensure that each action is outputted to the console to demonstrate the robot's behavior.*/




#include <iostream>
#include <string>

namespace Robots
{

    class Robot
    {
    private:
        std::string name;
        double speed;
        double weight;
        std::string size;
        int number_of_sensors;

    public:
        // Constructor to initialize the Robot's attributes
        Robot(std::string robot_name, double robot_speed, double robot_weight, std::string robot_size, int sensors)
            : name(robot_name), speed(robot_speed), weight(robot_weight), size(robot_size), number_of_sensors(sensors)
        {
            std::cout << "Attributes of Robot :" << std::endl;
            displayAttributes();
            moveForward();
            moveBackward();
            stop();
        }

        // Methods to simulate robot actions
        void moveForward()
        {
            std::cout << name << " is moving forward at speed " << speed << " m/s." << std::endl;
        }

        void moveBackward()
        {
            std::cout << name << " is moving backward at speed " << speed << " m/s." << std::endl;
        }

        void stop()
        {
            std::cout << name << " has stopped." << std::endl;
        }

        // Method to display robot's details
        void displayAttributes()
        {
            std::cout << "Robot Name: " << name << std::endl;
            std::cout << "Speed: " << speed << " m/s" << std::endl;
            std::cout << "Weight: " << weight << " kg" << std::endl;
            std::cout << "Size: " << size << std::endl;
            std::cout << "Number of Sensors: " << number_of_sensors << std::endl;
        }
    };

} // namespace Robots

int main()
{
    // Using the Robots namespace
    using namespace Robots;

    // Instantiate the first robot
    Robot robot1("AlphaBot", 2.5, 15.0, "Medium", 5);

    // robot1.displayAttributes();
    // std::cout << std::endl;

    // Simulate robot actions
    //robot1.moveForward();
    //robot1.moveBackward();
    //robot1.stop();
    // std::cout << std::endl;

    // Instantiate the second robot
    Robot robot2("BetaBot", 1.8, 20.0, "Large", 7);
    // std::cout << "Attributes of Robot 2:" << std::endl;
    // robot2.displayAttributes();
    // std::cout << std::endl;

    // Simulate robot actions
    // robot2.moveForward();
    // robot2.moveBackward();
    // robot2.stop();

    return 0;
}
