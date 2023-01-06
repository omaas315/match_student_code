# example_pkg
## Overview
Extended explanation what functionality is included in this package.

**Author:** Henrik Lurz
**E-Mail:** lurz@match.uni-hannover.de 

## Usage
To run the example_node launch the following command:

`roslaunch example_pkg example.launch`

## Config files
- `example.yaml`: Example config file that contains example parameter

## Launch files
- `example.launch`: Example launch-file

	Argument list:
	- `example_parameter` (type: `[string]`, default: `"example"`): Explanation of example parameter

## Nodes
### example_node
Description of example_ndoe

#### Subscribed Topics
- `example_sub` (std_msgs/Bool)

	Describe the data on this topic.

#### Published Topics
See `Subscribed Topics` for the structure of how to describe it.

#### Service Client/Server
See `Subscribed Topics` for the structure of how to describe it.

#### Action Client/Server
See `Subscribed Topics` for the structure of how to describe it.

#### Parameters

- `example_parameter1` (type: `int`, default: 100): Description of example_parameter1
- `example_parameter2` (type: `float`, default: None): Description of example_parameter2
