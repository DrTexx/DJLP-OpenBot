# DJLP-OpenBot
An affordable, accessible, open-source project for creating your own miniture - or larger - industrial robot arm/s.

## Python Terminology
For those confused by any terminology used in this readme, here are some definitions

_(**note**: click words to expand definitions)_

<details>
  <summary><b>CLASS</b></summary>
  <p>A template for creating user-defined objects. Class definitions normally contain method definitions which operate on instances of the class.</p>
</details>
<details>
  <summary><b>OBJECT</b></summary>
  <p>Any data with state (attributes or value) and defined behavior (methods). Also the ultimate base class of any new-style class.</p>
</details>
<br>

_All definitions are provided verbatim the official [python glossary](https://docs.python.org/3/glossary.html)._

## Purpose
Powered by MicroPython, DJLP-OpenBot aims to provide a simple way to control DIY industrial robot arms via the functionality of `Robot` classes.

## Robot Class
All information regarding your robot/s is stored in a `Robot` class. This is essentially the virtual version of your physical robot and it's hardware.

By calling objects (e.g. `ROBOT.joints['A'].axes['rot'].motor.get_pos`) or methods (e.g. `ROBOT.joints.axes['rot'].motor.set_pos(90)`) from your `ROBOT`, you can either recieve or set values accordingly.

### Physical Robots
Values are equal to the physical robot itself and if set are set immediantly

### Idol Robots
Values are equal to an imaginary idol robot and will be mimiced by the physical robot after x time has passed.

### Defining your robots in MicroPython
In order to control your robot, you need to create a representation of it in code.

In short, you need the following:
```python
ROBOT_NAME = Robot("ROBOT_NAME") # your robot
TARGET_NAME = Robot("TARGET_NAME>") # an imaginary 'idol' robot for your robot to copy
SERVO_OBJECT = Servo("SERVO_OBJECT",PIN,FREQENCY,STEPTIME,HOMEPOSITION,ISPHYSICAL) # a servo motor
for ITEM in [ROBOT,TARGET]: # for both your physical and target/idol robot...
    ITEM.joints['NEW_JOINT'] = Joint("NEW_JOINT") # add a new robot joint called "NEW_JOINT"
    ITEM.joints['NEW_JOINT'].axes['NEW_AXIS'] = Axis('NEW_AXIS',INITAL_VALUE,MIN_VALUE,MAX_VALUE,SERVO_OBJECT) # add a new axis to NEW_JOINT and link the servo object
```

Once you've done this, you can either enter the main loop or define additional robots/joints/servos/axes/LEDs/sensors/etc.

Once in the main loop routine, you can change the robots target position like this:

```python TARGET.joints['NEW_JOINT'].axes['NEW_AXIS'].motor.set_pos(NEW_POS)```

This changes position of the imaginary idol/target robot, which does not inheritly move the robot.

Instead, during the main loop the physical robot and the idol/target robot are compared; if the physical robot doesn't match all the axes on all the joints of the idol/target robot, the physical robots values will be modified incrementally (along with servos themselves) in order to move towards the idol/target robots position after x amount of time.

_(**NOTE:** specifying the duration to copy the idol robot's position is a work in progress, however it's not far from completion)_

**TL;DR** a routine will be executed to sync the physical motors to their target after x time has passed)

## Position Class
Position classes are currently a work in progress, however they are incredibly close to a working state.

These classes allow for multiple elements of a physical robot to all by mimiced at once, for instance, the robot's joints, servos contained within those joints, any LEDs, any sensors, etc. numerous servos.

I'm underdecided as of yet if positions should be an element of a attribute called 'state' under the robot object.

## Pull-requests?
Always welcome! I do request, however, that you adhere to PEP-8 standards in order to maintain readability.
Only one exception applies to this, in being that any variables derived from the `Robot` class must be fully captialised in order to provide the benefit of fast identification.

## Plans for Future Development and Improvement
- reverse kinemoatics
- paths
- sensor support
- visualisation of robot in a GUI? (lil' ambitious)
