# CPSwarm Code Generator
The CPSwarm Code Generator is one of the component of the CPSwarm Workbenech and is responsible to translate design-level modelled behaviours into concrete, executable code.
The first version of the Code Generator supports the generation of code starting from the formal description of a Finite State Machine using the SCXML standard.
This project is currently under active development and not ready for production.

## Getting Started
* Documentation: [wiki]

## Deployment
Packages are built continuously with [Bamboo](https://pipelines.linksmart.eu/browse/CPSW-CGB/latest).

### Prerequisites
The project is a Maven project, based on Java 8. To compile it you need to have: 

* [Java 8 JDK](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)
* [Maven](https://maven.apache.org/)

### Build from source
Within the root of the repository, the project can be easily built using the following Maven commands.

``` bash
mvn validate
mvn package -DskipTests

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
java -jar /home/cpswarm-ws/code-generator-0.0.1-full.jar --env ROS --src /home/cpswarm-ws/models/UAV_sar_FSM.xml --target /home/cpswarm-ws/GeneratedCode
```

## Development
### Run tests
```bash
 mvn test
```

## Contributing
Contributions are welcome. 

Please fork, make your changes, and submit a pull request. For major changes, please open an issue first and discuss it with the other authors.

## Affiliation
![CPSwarm](https://github.com/cpswarm/template/raw/master/cpswarm.png)  
This work is supported by the European Commission through the [CPSwarm H2020 project](https://cpswarm.eu) under grant no. 731946.