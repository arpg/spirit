///////////////////////////////////////////////////////////////////////////////
/// This file includes the problems which has been realized during tests, to keep
/// track of problems and how the have been solved in the code (related code fix
/// might be followed from github history)
/*
 - if car falls on sides on the mesh it goes through mesh (collision with buddy
is not working)

 - when dropping the car on the mesh with zero motion commands and velocities,
car drifts on the mesh very slowly.

 - when using CarPlanner tic-toc instead of constant time for command.m_dT then
each time running the car It gives different answeres for final position of the
car.

*/
