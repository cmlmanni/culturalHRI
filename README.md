# Culturally Aware HRI Framework: A framework to examine the importance of cultures in Human-Robot Interactions

## Project Overview

This framework is a research-based software engineering project aimed at exploring culturally adaptive human-robot interaction (HRI) within diverse office environments. The project focuses on simulating and studying greeting behaviors of robotic agents in multiethnic workplaces, utilizing the ROS4HRI framework and the OfficeBots simulation environment.

## Project Structure

The project repository is structured as follows:

```bash
├── .gitignore
├── rasa
│ ├── hall
│ │ ├── actions
│ │ ├── config.yml
│ │ ├── credentials.yml
│ │ ├── data
│ │ ├── domain.yml
│ │ ├── endpoints.yml
│ │ ├── models
│ │ └── tests
├── README.md
└── scripts
├── database
│ ├── database_connection.py
│ ├── database_factory.py
│ └── pg_config.py
├── html
│ ├── public
│ ├── server.js
│ ├── survey1.html
│ ├── survey2.html
│ └── thankYou.html
├── rasa
│ └── run_rasa.py
└── robot
├── engagement.py
├── officebots_ros.py
├── robot_database_interface.py
├── robot_factory.py
├── robot.py
└── simple_ros_bridge.py
```

## Components

### The Framework

This Framework serves as the core of the project, providing tools and methodologies to examine the influence of cultural factors on HRI. It includes modules for data collection, analysis, and simulation of cultural interactions between humans and robots.

### Scripts

The `scripts` directory contains various Python and JavaScript scripts utilized in the project, including:

- Database scripts for managing database connections and configurations.
- HTML and JavaScript files for building user interfaces and survey forms.
- Rasa scripts for running the Rasa conversational AI platform.
- Robot scripts for managing robotic engagement, interfacing with databases, and controlling robot behavior.

## Usage

To utilize the CulturalBot Framework or replicate the research findings:

1. Clone the repository: `git clone <repository_url>`
2. Install dependencies: `pip install -r requirements.txt`
3. Explore and modify the codebase as needed.
4. Run simulations, experiments, or tests using the provided scripts.
5. Document any changes or additions made to the project.

## Contributing

Contributions to the Framework are welcome! If you'd like to contribute, please follow the contribution guidelines outlined in the repository.

### Reporting Issues

If you encounter any issues or bugs while using CulturalBot Framework, please report them by opening an issue on GitHub. Include as much detail as possible, including steps to reproduce the issue and any relevant error messages or logs.

### Contact

If you have any questions or need further assistance, feel free to contact the project maintainers at [mlc212@student.bham.ac.uk](mailto:mlc212@student.bham.ac.uk).

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments and Related Projects

This project utilizes the following frameworks and tools:

- [ROS4HRI](https://blog.pal-robotics.com/ros4hri-standardising-an-interface-for-human-robot-interaction/): ROS4HRI is a framework developed for standardizing interfaces for Human-Robot Interaction (HRI) within the Robot Operating System (ROS). It provides uniform interfaces and conventions, fostering code reusability and experiment replicability in HRI research.

- [OfficeBots](https://blog.pal-robotics.com/ros4hri-standardising-an-interface-for-human-robot-interaction/): OfficeBots is a simulation environment developed for HRI research and education. It allows researchers to instantiate, customize, and control robotic avatars within a digital workspace, facilitating empirical studies on human-robot interactions.

This project acknowledges the contributions of the developers and researchers behind ROS4HRI and OfficeBots for their valuable tools and resources that have enhanced the capabilities of this project.
