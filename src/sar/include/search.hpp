/**
 * @file search.hpp
 * @brief Class definition for the `ObjectSearch` class.
 * 
 * This file defines the `ObjectSearch` class, which provides functionality for 
 * performing object detection using a pre-trained YOLO model. The class allows 
 * for initializing the model, analyzing video frames, and determining if specific 
 * objects are detected.
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

#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

/**
 * @brief Class for performing object detection using a pre-trained YOLO model.
 * 
 * The `ObjectSearch` class initializes a YOLO-based object detection model and provides 
 * methods to analyze video frames for object detection. It determines whether a specific 
 * object of interest is present in the analyzed frames and maintains the detection state.
 */
class ObjectSearch {
 public:
  /**
   * @brief Constructs an `ObjectSearch` object with a specified model and class names.
   * 
   * This constructor initializes an instance of the `ObjectSearch` class by loading the 
   * pre-trained object detection model and reading the class names from the specified file.
   * 
   * @param modelPath The file path to the pre-trained YOLO object detection model (e.g., weights).
   * @param yolo_names The file path to the class names file (e.g., coco.names).
   */
  ObjectSearch(const std::string& modelPath, const std::string& yolo_names);

  /**
   * @brief Destructor for the `ObjectSearch` class.
   * 
   * Cleans up resources and ensures proper shutdown of the `ObjectSearch` instance.
   */
  ~ObjectSearch();

  /**
   * @brief Analyzes a video frame for object detection.
   * 
   * This method processes a given video frame using the YOLO model to detect objects. 
   * The detection results are used to determine if the object of interest is present.
   * 
   * @param frame The input video frame for object detection.
   * @return True if the object of interest is detected in the frame, false otherwise.
   */
  bool runObjectDetection(const cv::Mat& frame);

  /**
   * @brief Checks whether the object of interest was detected.
   * 
   * This function returns the current detection state, indicating whether the object 
   * of interest was detected in the most recent frame analysis.
   * 
   * @return True if the object is found, false otherwise.
   */
  bool isObjectFound() const;

 private:
  /**
   * @brief Indicates whether the object of interest was detected.
   * 
   * This private member variable stores the detection status, updated after each 
   * frame is analyzed.
   */
  bool objectFound;

  /**
   * @brief The YOLO-based object detection model.
   * 
   * This private member variable holds the YOLO model used for object detection. 
   * The model is loaded using OpenCV's DNN module.
   */
  cv::dnn::Net humanDetectionModel;

  /**
   * @brief A vector containing class names for object detection.
   * 
   * The `classNames` vector stores the names of the object classes supported by the YOLO model. 
   * These names are loaded from a file (e.g., coco.names) during the initialization of the class.
   */
  std::vector<std::string> classNames;
};

#endif  // SEARCH_HPP
