/* Author: Ali Golestaneh and Constantinos Chamzas */
#include "DiskSampler.h"

bool isStateValid(const ob::State *state) {

    // cast the abstract state type to the type we expect
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0];
    double y = r2state->values[1];
    // A square obstacle with and edge of size 2*sqrt(2) is located in location [-3,-2,] and rotated pi/4 degrees around its center.
    // Fill out this function that returns False when the state is inside/or the obstacle and True otherwise// 

    // ******* START OF YOUR CODE HERE *******//

    // Obstacle parameters
    double obs_x = -3.0;
    double obs_y = -2.0;
    double obs_size = 2 * std::sqrt(2); // side length of the obstacle
    double angle = M_PI / 4; // rotation angle in radians

    // Translate the point to the obstacle's coordinate system
    double x_trans = x - obs_x;
    double y_trans = y - obs_y;

    // Rotate the point by -angle to align with axis
    double x_rot = x_trans * std::cos(-angle) - y_trans * std::sin(-angle);
    double y_rot = x_trans * std::sin(-angle) + y_trans * std::cos(-angle);

    // Check if the point is inside the square
    if (std::abs(x_rot) <= obs_size / 2 && std::abs(y_rot) <= obs_size / 2) {
        return false; // In collision
    }
    return true; // Not in collision

    // ******* END OF YOUR CODE HERE *******//
}


bool DiskSampler::sampleNaive(ob::State *state) 
{
    // ******* START OF YOUR CODE HERE *******//

    // Sample random polar coordinates
    double r = rng_.uniformReal(0.0, 10.0); // Radius
    double theta = rng_.uniformReal(0.0, 2 * M_PI); // Angle

    // Convert polar coordinates to Cartesian coordinates
    double x = r * std::cos(theta);
    double y = r * std::sin(theta);

    // Set state values
    auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    r2state->values[0] = x;
    r2state->values[1] = y;

    // ******* END OF YOUR CODE HERE *******//
    
    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}

bool DiskSampler::sampleCorrect(ob::State *state)
{
    // ******* START OF YOUR CODE HERE *******//

    // Sample a radius using the square root method to ensure uniform distribution
    double r = std::sqrt(rng_.uniformReal(0.0, 100.0)); // 10^2 = 100
    double theta = rng_.uniformReal(0.0, 2 * M_PI); // Angle

    // Convert polar coordinates to Cartesian coordinates
    double x = r * std::cos(theta);
    double y = r * std::sin(theta);

    // Set state values
    auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    r2state->values[0] = x;
    r2state->values[1] = y;

    // ******* END OF YOUR CODE HERE *******//


    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}
