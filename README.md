Note that Pixhawk boards have two processors. One for the main processing task and comms (stfm32f4) and one for servo outputs (stm32f1), thus there are two projects.
Both processors talk through an uart connection.
For the output projects, there are two different configurations for two versions of the pixhawk board too. Select the one according to your output processor model.
