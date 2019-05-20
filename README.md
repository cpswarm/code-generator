# CPSwarm Code Generator
The CPSwarm Code Generator is one of the component of the CPSwarm Workbenech and is responsible to translate design-level modelled behaviours into concrete, executable code.
The first version of the Code Generator supports the generation of code starting from the formal description of a Finite State Machine using the SCXML standard.
This project is currently under active development and not ready for production.

## Documentation

## Dependencies
The project is a Maven project, based on Java 8. To compile it you need to have [Java 8 JDK](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) and [Maven](https://maven.apache.org/).

## Installation
The project can be installed with the following command

``` bash
mvn install
```
## Run
These are the parameters to be passed to the software to run

``` bash
usage: java -jar code-generator.jar
 --env <arg>     target Runtime Environment (default ROS)
 --src <arg>     input file path
 --target <arg>  output folder path
 ```
And this is an example of running command

``` bash
java -jar /home/gprato/cpswarm-ws/code-generator-0.0.1-SNAPSHOT-full.jar --src /home/cpswarm/launcher_project/Models/UAV_sar_FSM.xml --target /home/cpswarm/launcher_project/GeneratedCode
```
