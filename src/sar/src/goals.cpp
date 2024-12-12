/**
 * @file goals.cpp
 * @brief Implementation of the GoalGenerator class for generating random goal positions.
 * 
 * This file contains the implementation of the `GoalGenerator` class, which is responsible 
 * for generating random goal positions within a specified map area. These goals can be 
 * utilized for navigation and traversal in robotics applications.
 * 
 * @version 0.2
 * @date 2024-11-20
 * 
 * @authors
 * - Dhairya Shah
 * - Harsh Senjaliya
 * 
 * @copyright 
 * Copyright © 2023 <copyright holders>.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this 
 * software and associated documentation files (the “Software”), to deal in the Software 
 * without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
 * to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * - The above copyright notice and this permission notice shall be included in all copies 
 *   or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include "goals.hpp"
#include <random>

/**
 * @brief Constructs a new `GoalGenerator` object with specified map dimensions.
 * 
 * This constructor initializes the `GoalGenerator` class by setting the width and height 
 * of the map where the goals will be generated. These dimensions define the bounds for 
 * the random goal positions.
 * 
 * @param mapWidth The width of the map, representing the maximum x-coordinate for goals.
 * @param mapHeight The height of the map, representing the maximum y-coordinate for goals.
 */
GoalGenerator::GoalGenerator(int mapWidth, int mapHeight)
    : mapWidth_(mapWidth), mapHeight_(mapHeight) {}

/**
 * @brief Generates a random goal position within the defined map boundaries.
 * 
 * This method uses a random number generator to produce a uniformly distributed x and y 
 * coordinate pair within the dimensions of the map. The resulting goal position is 
 * encapsulated in a `GoalPosition` structure, which is returned to the caller.
 * 
 * The random number generation uses `std::random_device` for seeding and `std::mt19937` 
 * for generating pseudo-random numbers. The `std::uniform_real_distribution` ensures that 
 * the generated coordinates are continuous and fall within the specified map dimensions.
 * 
 * @return GoalPosition A structure containing the randomly generated x and y coordinates 
 *                      of the goal position.
 */
GoalPosition GoalGenerator::generateRandomGoal() {
  // Define a structure object to store the generated goal position.
  GoalPosition goal;

  // Create a random number generator and distributions for x and y coordinates.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dist(0.0, static_cast<double>(mapWidth_));
  std::uniform_real_distribution<> y_dist(0.0, static_cast<double>(mapHeight_));

  // Generate and assign the x and y coordinates.
  goal.x = x_dist(gen);
  goal.y = y_dist(gen);

  // Return the generated goal position.
  return goal;
}
