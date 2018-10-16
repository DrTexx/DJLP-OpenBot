#### [DISCLAIMER]: *This project is undergoing some serious code changes, the following examples and/or explanations are likely not representative of future commits!*
# DJLP-OpenBot
An affordable, accessible, open-source project for creating your own miniature - or larger - industrial robot arm/s.

_(**note**: this readme is currently a WIP)_

## Python Terminology
If you're confused by any terminology in this readme, please feel free to consult these definitions

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

_All definitions are provided verbatim the official [python glossary](https://docs.python.org/3/glossary.html) unless otherwise specified._

## Purpose
To provide a simple way to control D.I.Y industrial robots via MicroPython `Robot` objects.

Robot objects are created from the `Robot()` class defined in DJLP-OpenBot.

## Usage
### Robot Objects
All information regarding your robot is stored in a `Robot` object.
This object is essentially the microcontroller's interpretation of a physical robot and it's hardware.

You can gain information about the current state of your robot by calling object attributes

**EXAMPLE:**
```python
ROBOT.joints['A'].get_hardware.get_pos # get current position of motor
```

and you can set new values by executing the object's methods

**EXAMPLE:**
```python
ROBOT.joints['A'].get_hardware.set_pos(90) # change motors current position to 90 degrees
```

#### Movement Objects
Movement objects are attached to joint hardware. These objects contain all the data required to complete movements within the duration specified.
If it weren't for movement objects updates would be immidient and servos would move to their new position as fast as they physically can.

#### Creating Robot Objects in MicroPython
The following is required to create and control robot objects in MicroPython:
```python
ROBOT_NAME = Robot("ROBOT_NAME") # your robot
ROBOT_NAME.joints['NEW_JOINT'] = Robot.Joint("NEW_JOINT",SERVO_OBJECT) # add a new robot joint called "NEW_JOINT"
```

Once you've done this, you can either enter the main loop or you can define additional robots/joints/servos/axes/LEDs/sensors/etc.

### Position Objects
`Position` objects contain a set of data to overwrite the IDOL robot's current state (so that all changes can be executed in a single line)

Position objects are created with the `Position()` class

_This class is currently a work in progress, however it's very close to a satisfactory working level of completion._

#### Benefit
Position objects allow for multiple attributes of the idol robot to be changed at once.

This means that, for example, the brightness of an LED, the degree of multiple axes of a joint and a buzzer can be sounded all at once on the IDOL robot (which the physical robot will copy in time).

## Pull requests
All pull requests welcome! I do however ask, that you adhere to PEP-8 standards to maintain readability.
Only one exception applies to this, in being that any variables derived from the `Robot` class must be fully capitalised to provide the benefit of fast identification.

<details>
  <summary><i>Plans for Future Development and Improvement</i></summary>
  <ul>
    <li>paths</li>
    <li>sensor support</li>
    <li>visualisation of robot in a GUI? (lil' ambitious)</li>
    <li>reverse kinematics (very ambitious, will need help or pre-made packages)</li>
  </ul>
</details>
