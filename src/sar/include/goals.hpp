/**
 * @file goals.hpp
 * @brief Class definition and supporting structures for the `GoalGenerator` class.
 * 
 * This file defines the `GoalGenerator` class, which provides functionality to 
 * generate random goal positions within specified map boundaries. The generated goals 
 * can be used for navigation and traversal in robotics and simulation environments.
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

#ifndef GOALS_HPP
#define GOALS_HPP

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief A structure representing a goal position.
 * 
 * This structure encapsulates the x and y coordinates of a goal. It is used as 
 * the return type for the goal generation functionality provided by the 
 * `GoalGenerator` class.
 */
struct GoalPosition {
    double x; /**< X-coordinate of the goal position. */
    double y; /**< Y-coordinate of the goal position. */
};

/**
 * @brief A class for generating random goal positions within specified map boundaries.
 * 
 * The `GoalGenerator` class generates random goal positions using a uniform distribution 
 * within the bounds of a map defined by its width and height. This is useful for applications 
 * such as autonomous navigation and robot path planning.
 */
class GoalGenerator {
public:
    /**
     * @brief Constructs a `GoalGenerator` object with specified map dimensions.
     * 
     * Initializes the generator with the width and height of the map, defining 
     * the boundaries for goal generation.
     * 
     * @param mapWidth The width of the map in units (e.g., meters, pixels).
     * @param mapHeight The height of the map in units (e.g., meters, pixels).
     */
    GoalGenerator(int mapWidth, int mapHeight);

    /**
     * @brief Generates a random goal position within the defined map boundaries.
     * 
     * This method uses a random number generator to produce x and y coordinates 
     * that fall within the specified map width and height. The generated goal is 
     * returned as a `GoalPosition` structure.
     * 
     * @return GoalPosition A structure containing the randomly generated goal's 
     *                      x and y coordinates.
     */
    GoalPosition generateRandomGoal();

private:
    /**
     * @brief The width of the map.
     * 
     * Specifies the maximum allowable x-coordinate for generated goal positions.
     */
    int mapWidth_;

    /**
     * @brief The height of the map.
     * 
     * Specifies the maximum allowable y-coordinate for generated goal positions.
     */
    int mapHeight_;
};

#endif  // GOALS_HPP
