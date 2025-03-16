# Intelligent Driver Model
This project was created as part of a 3rd year modelling and simulation module at Queen's University Belfast. 

The standard model simulates a given number of cars driving around a ring road where each driver reacts to their surroundings and makes adjustments to their speed.
The extended model adds the option to add traffic lights and roundabouts.

The model works on a time step basis. At each time step each cars current situation (velocity, accelleration, position, headway, velocity of proceeding car) is taken into account along with a set of parameters to calculate each cars velocity, acceleration and position for the next timestep. A detailed description of the intelligent driver model is given in chapter 11 of the 
book "Traffic Flow Dynamics: Data, Models and Simulation" by M. Treiber and A. Kesting.

Details of extensive testing of both models are included, these tests allowed for extensive data collection for analysis of the model. Programs used to perform analysis of the model are also included.

A file is also included that shows exerts from the report that was produced by me and a team of my peers based on this model.

#### Future Improvements
An interesting test case would be to reduce the size of the ring road and implement 4 junctions in order to model a roundabout. Traffic lights could also be included.\
I would like to improve the readability of this code, this was by far the biggest project I had attempted at the time and I have since learned to write much cleaner code.\
The next logical step for the model seems to be to introduce a second lane on the main road. It would be interesting to see if two main roads could be linked by a smaller road that had junctions 
leading onto the two main roads.
