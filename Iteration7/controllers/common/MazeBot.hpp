#ifndef MAZE_BOT
#define MAZE_BOT

// File:          MazeBot.hpp
// Date:          10-Oct-20
// Description:   Created to control epuck while navigating a maze
// Author:        Jeremy Dsilva

#include <iostream>
#include <math.h>
#include <limits>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>

#define TIME_STEP 64 // time in [ms] of a simulation step
#define MAX_SPEED 6.28
#define UNIT_SPEED (MAX_SPEED / 100.0)
#define PI 3.141592653589793116
#define HALF_PI 1.570796326794896558
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define CELL_LENGTH 0.1

using namespace webots;

class MazeBot
{
public:
    /**
     * Constructor
     * param robot  
     * 
     * Takes robot and create instance of motors, position sensor, distance sensors and light sensors
     * 
     **/
    MazeBot(Robot *robot)
    {
        this->robot = robot;
        leftMotor = robot->getMotor("left wheel motor");
        rightMotor = robot->getMotor("right wheel motor");
        leftSensor = leftMotor->getPositionSensor();
        rightSensor = rightMotor->getPositionSensor();
        leftSensor->enable(TIME_STEP);
        rightSensor->enable(TIME_STEP);

        setPosition(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

        char psNames[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

        for (int i = 0; i < 8; i++)
        {
            ps[i] = robot->getDistanceSensor(psNames[i]);
            ps[i]->enable(TIME_STEP);
        }
    }

    // step one time unit
    inline bool step()
    {
        return robot->step(TIME_STEP) != -1;
    }

    // step count time units
    inline bool step(int count)
    {
        for (int i = 0; i < count; ++i)
            if (!step())
                return false;
        return true;
    }

    // move robot forward by number of cells
    // calulation based on cell lenght and wheel radius
    inline void forward(double cell)
    {
        double dp = (CELL_LENGTH * cell) / WHEEL_RADIUS;
        std::cout << "Moving forward: " << dp << std::endl;

        setRelativePosition(dp, dp);
    }

    // move right in place
    // turn value calculated from wheel radius and axis length
    inline void left()
    {
        const double turnValue = 2.230916371;
        setRelativePosition(-1.0 * turnValue, turnValue);
    }

    // move left in place
    // turn value calculated from wheel radius and axis length
    inline void right()
    {
        const double turnValue = 2.230916371;
        setRelativePosition(turnValue, -1.0 * turnValue);
    }

    // set relative poistion
    void setRelativePosition(double left, double right)
    {
        // get target values based on currect position sensor reading
        double targetValueLeft = leftSensor->getValue() + left;
        double targetValueRight = rightSensor->getValue() + right;

        // stop set postion and step
        stop();
        setPosition(targetValueLeft, targetValueRight);
        step();

        // decrease speed and step
        setSpeed(1, 1);
        step();

        // loop while targer is not reached
        while (step() &&
               (std::abs(targetValueLeft - leftSensor->getValue()) > 0.01 &&
                std::abs(targetValueRight - rightSensor->getValue()) > 0.01))
        {
        } //std::cerr << "Position Error: " << std::abs(targetValueLeft - leftSensor->getValue()) << " " << std::abs(targetValueRight - rightSensor->getValue()) << std::endl;

        // set position to infinity stop
        setPosition(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        setSpeed(0, 0);
    }

    // sets speed to 0 and steps
    void stop()
    {
        setSpeed(0, 0);
        step();
    }

    // function to set speed of motors
    inline void setSpeed(double leftSpeed, double rightSpeed)
    {
        leftMotor->setVelocity(leftSpeed * UNIT_SPEED), rightMotor->setVelocity(rightSpeed * UNIT_SPEED);
    }

    // function to set position of motors
    inline void setPosition(double leftPostion, double rightPositon)
    {
        leftMotor->setPosition(leftPostion), rightMotor->setPosition(rightPositon);
    }

    // position robot between left and right wall
    void betweenWall()
    {

        // sensor weight
        double w_d1 = 0.6;
        double w_d2 = 0.4;

        // left and right sensor numbers
        int leftSensor[] = {6, 5}, rightSensor[] = {1, 2};

        // speed
        double sLeft = this->speed;
        double sRight = this->speed;

        // define pid gains
        double Kp = 0.2; // 0.2
        double Kd = 0.6;
        double Ki = 0.05;

        // error between walls
        double ecurr, eprev = w_d1 * (readDistanceSensor(leftSensor[0]) - readDistanceSensor(rightSensor[0])) + w_d2 * (readDistanceSensor(leftSensor[1]) - readDistanceSensor(rightSensor[1]));

        // error multiplier
        double err_mult = 0.005;

        // loop while there is no wall ahead
        while (step() && !wallAhead())
        {

            // break loop if left or right wall is not detected
            if (readDistanceSensor(leftSensor[1]) < PS_NOISE || readDistanceSensor(rightSensor[1]) < PS_NOISE)
                break;

            // current error between walls
            ecurr = w_d1 * (readDistanceSensor(leftSensor[0]) - readDistanceSensor(rightSensor[0])) + w_d2 * (readDistanceSensor(leftSensor[1]) - readDistanceSensor(rightSensor[1]));

            // compute errors
            double ep = ecurr;
            double ed = ecurr - eprev;
            double ei = ecurr + eprev;

            std::cout << "Error " << ep << ' ' << ed << ' ' << ei << std::endl;

            // calculated error correction
            double err_correction = (Kp * ep + Kd * ed + Ki * (ei));

            // compute speed
            sRight = this->speed;
            sLeft = this->speed + err_mult * err_correction;
            std::cout << "Speed " << sLeft << ' ' << sRight << std::endl;

            setSpeed(sLeft, sRight);

            // update error
            eprev = ecurr;
        }
    }

    void followWall(bool isRightWall)
    {
        // sensor weight
        double w_d1 = 0.6;
        double w_d2 = 0.4;

        // left sensors
        int sensor[] = {6, 5};

        // if following right wall switch to right sensors
        if (isRightWall)
            sensor[0] = 1, sensor[1] = 2;

        // speed
        double sLeft = this->speed;
        double sRight = this->speed;

        // define pid gains
        double Kp = 0.2; // 0.2
        double Kd = 0.6;
        double Ki = 0.05;

        const double WALL_DISTANCE = 120;

        // error between distance to keep and wall
        double ecurr, eprev = ((w_d1 * readDistanceSensor(sensor[0]) + w_d2 * readDistanceSensor(sensor[1]))) - WALL_DISTANCE;

        // error multiplier
        double err_mult = 0.01;

        // loop while there is no wall ahead
        while (step() && !wallAhead())
        {

            // break loop wall is not detected
            if (readDistanceSensor(sensor[1]) < PS_NOISE)
                break;

            // current error between distance to keep and wall
            ecurr = (w_d1 * readDistanceSensor(sensor[0]) + w_d2 * readDistanceSensor(sensor[1])) - WALL_DISTANCE;

            // compute errors
            double ep = ecurr;
            double ed = ecurr - eprev;
            double ei = ecurr + eprev;

            std::cout << "Error " << ep << ' ' << ed << ' ' << ei << std::endl;

            // calculated error correction
            double err_correction = (Kp * ep + Kd * ed + Ki * (ei));

            // compute speed
            // speed is kept between range inorder not to swing to far away from wall if bot is too close
            if (isRightWall)
            {
                sRight = this->speed + err_mult * err_correction;
                sRight = err_correction > 0 ? (sRight > 20.1 ? 20.1 : sRight) : sRight;
            }
            else
            {
                sLeft = this->speed + err_mult * err_correction;
                sLeft = err_correction > 0 ? (sLeft > 20.1 ? 20.1 : sLeft) : sLeft;
            }
            std::cout << "Speed " << sLeft << ' ' << sRight << std::endl;

            setSpeed(sLeft, sRight);

            // update error
            eprev = ecurr;
        }
    }

    // checks if wall exist on the right of bot
    inline bool wallRight()
    {
        return readDistanceSensor(2) > PS_THRESHOLD;
    }

    // checks if wall exist on the left of bot
    inline bool wallLeft()
    {
        return readDistanceSensor(5) > PS_THRESHOLD;
    }

    // checks if wall exist on the ahead of bot
    inline bool wallAhead()
    {
        return readDistanceSensor(0) > PS_THRESHOLD && readDistanceSensor(7) > PS_THRESHOLD;
    }

    // if bot as open space in front left or right
    inline bool openSpace()
    {
        return !(wallAhead() || wallRight() || wallLeft());
    }

    // read distance sensor
    inline double readDistanceSensor(int i)
    {
        return ps[i]->getValue();
    }

    // read light sensor
    inline double readLightSensor(int i)
    {
        return ls[i]->getValue();
    }

    // has bot reached goal
    // goal defined as reaching light source
    bool hasGoal()
    {
        int count = 0;
        for (int i = 0; i < 8; ++i)
            if (4000 < readLightSensor(i))
                ++count;
        return count == 8;
    }

    // destructor
    ~MazeBot()
    {
        if (robot != nullptr)
            delete robot;
    }

private:
    Robot *robot;
    Motor *leftMotor, *rightMotor;
    PositionSensor *leftSensor, *rightSensor;
    LightSensor *ls[8];
    DistanceSensor *ps[8];

    const double PS_THRESHOLD = 80;
    const double PS_NOISE = 65;
    const double speed = 20;
};

#endif