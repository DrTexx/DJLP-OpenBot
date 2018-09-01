# DJLP-OpenBot
An affordable, accessible, open-source project for creating your own miniture - or larger - industrial robot arm/s.

## Python Terminology
For those confused by any terminology used in this readme, here are some definitions

_(**note**: click words to expand definitions)_

<details>
  <summary><b>ATTRIBUTE</b></summary>
  <p>A value associated with an object which is referenced by name using dotted expressions. For example, if an object o has an attribute a it would be referenced as o.a.</p>
</details>
<details>
  <summary><b>CLASS</b></summary>
  <p>A template for creating user-defined objects. Class definitions normally contain method definitions which operate on instances of the class.</p>
</details>
<details>
  <summary><b>METHOD</b></summary>
  <p>A function which is defined inside a class body. If called as an attribute of an instance of that class, the method will get the instance object as its first argument (which is usually called self). See function and nested scope.</p>
</details>
<details>
  <summary><b>OBJECT</b></summary>
  <p>Any data with state (attributes or value) and defined behavior (methods). Also the ultimate base class of any new-style class.</p>
</details>
<details>
  <summary><b>TICK</b></summary>
  <p>(personal definition): A single execution of everything in a loop.</p>
</details>
<br>

_All definitions are provided verbatim the official [python glossary](https://docs.python.org/3/glossary.html)._

## Purpose
To provide a simple way to control D.I.Y industrial robots via MicroPython `Robot` objects.

Robot objects are created from the `Robot()` class defined in DJLP-OpenBot.

## Usage
### Robot Objects
All information regarding your robot is stored in a `Robot` object.
This object is essentially the microproccesors interpretation of a physical robot and it's hardware.

You can gain information about the current state of your robot by calling object attributes

**EXAMPLE:**
```python
ROBOT.joints['A'].axes['rot'].motor.get_pos # get current position of motor
```

and you can set new values by executing the object's methods

**EXAMPLE:**
```python
ROBOT.joints.axes['rot'].motor.set_pos(90) # change motors current position to 90 degrees
```

#### Types of Robots
##### Physical Robots
Physical robot objects are intrinsicly linked to physical hardware itself.
When values are set for physical robots, updates are immediant (such as moving a servo)

##### Idol Robots
Idol robots are imaginary robots that your robot will attempt to mimic.
When values are set for idol robots, your physical robot will mimic it's position after _x_ time has passed.

#### Creating Robot Objects in MicroPython
The following is required to create and control robot objects in MicroPython:
```python
ROBOT_NAME = Robot("ROBOT_NAME") # your robot
TARGET_NAME = Robot("TARGET_NAME>") # an imaginary 'idol' robot for your robot to copy
SERVO_OBJECT = Servo("SERVO_OBJECT",PIN,FREQENCY,STEPTIME,HOMEPOSITION,ISPHYSICAL) # a servo motor
for ITEM in [ROBOT,TARGET]: # for both your physical and target/idol robot...
    ITEM.joints['NEW_JOINT'] = Joint("NEW_JOINT") # add a new robot joint called "NEW_JOINT"
    ITEM.joints['NEW_JOINT'].axes['NEW_AXIS'] = Axis('NEW_AXIS',INITAL_VALUE,MIN_VALUE,MAX_VALUE,SERVO_OBJECT) # add a new axis to NEW_JOINT and link the servo object
```

Once you've done this, you can either enter the main loop or you can define additional robots/joints/servos/axes/LEDs/sensors/etc.

Once the `Main()` loop has been entered, you can change the robots target position like so:

```python
IDOL.joints['NEW_JOINT'].axes['NEW_AXIS'].motor.set_pos(NEW_POS)
```

This will change the position of the `IDOL` robot. Once the current tick has completed (very, very quick) the `Main()` loop will identify that there's a difference between the physical robot and the idol robot. After identifying the difference between the two robots joints and axes, the `update_position()` command is ran every tick until the physical robot matches it's idol.

_(**NOTE:** specifying the amount of time taken for the physical robot to go from it's original position to the idol's positon is still currently a work in progress, however this isn't far from completion)_

### Position Objects
`Position` objects contain a set of data to overwrite the IDOL robot's current state (so that all changes can be executed in a single line)

Position objects are created with the `Position()` class

_This class is currently a work in progress, however it's very close to a satisfactory workingg level of completion._

#### Benefit
Position objects allow for multiple attributes of the idol robot to be changed at once.

This means that, for example, the brightness of an LED, the degree of multiple axes of a joint and a buzzer can be sounded all at once on the IDOL robot (which the physical robot will copy in time).

## Pull requests
Any and all pull requests welcome! I do however ask, that you adhere to PEP-8 standards in order to maintain readability.
Only one exception applies to this, in being that any variables derived from the `Robot` class must be fully captialised in order to provide the benefit of fast identification.

<details>
  <summary><i>Plans for Future Development and Improvement</i></summary>
  <ul>
    <li>paths</li>
    <li>sensor support</li>
    <li>visualisation of robot in a GUI? (lil' ambitious)</li>
    <li>reverse kinematics (very ambitious, will need help or pre-made packages)</li>
  </ul>
</details>
