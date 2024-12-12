/**
 * @file search.cpp
 * @brief Implementation of the ObjectSearch class for object detection.
 * 
 * This file contains the implementation of the `ObjectSearch` class, which is designed 
 * to detect specific objects in video frames using a pre-trained YOLO model. It provides 
 * functionality to analyze frames, determine the presence of objects of interest, and 
 * manage the state of object detection.
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

#include "search.hpp"

#include <opencv2/core.hpp>

/**
 * @brief Constructs a new `ObjectSearch` object.
 * 
 * This constructor initializes an instance of the `ObjectSearch` class by loading 
 * the YOLO object detection model and reading the class names from the specified file. 
 * The model is used to detect objects in video frames.
 * 
 * @param modelPath The path to the pre-trained object detection model (e.g., YOLO weights).
 * @param yolo_names The path to the file containing class names (e.g., coco.names).
 */
ObjectSearch::ObjectSearch(const std::string& modelPath,
                           const std::string& yolo_names)
    : objectFound(false), humanDetectionModel(cv::dnn::readNet(modelPath)) {
  // Load class names from the provided file
  std::ifstream classNamesFile(yolo_names.c_str());
  std::string text;
  while (classNamesFile >> text) {
    getline(classNamesFile, text);
    classNames.push_back(text);
  }
}

/**
 * @brief Destructor for the `ObjectSearch` class.
 * 
 * This destructor ensures the proper cleanup of resources used by the `ObjectSearch` 
 * instance. Additional resource management can be added if required.
 */
ObjectSearch::~ObjectSearch() {
  // Destructor implementation (if needed)
}

/**
 * @brief Runs object detection on a given video frame.
 * 
 * This function processes the input video frame using the pre-loaded YOLO model. It generates 
 * a blob from the frame, forwards it through the model, and analyzes the outputs to detect 
 * objects. The function updates the internal state (`objectFound`) based on the detection 
 * results and returns whether the object of interest was found.
 * 
 * The detection focuses on specific classes (e.g., "person" or "ball") based on their class IDs.
 * 
 * @param frame The input video frame for object detection.
 * @return True if the object of interest is detected, false otherwise.
 */
bool ObjectSearch::runObjectDetection(const cv::Mat& frame) {
  // Convert the input frame into a blob suitable for the model
  cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640.0, 640.0),
                                        cv::Scalar(), true, false);

  // Set the input blob for the detection model
  humanDetectionModel.setInput(blob);

  std::vector<cv::Mat> outputs;

  // Perform a forward pass through the model
  humanDetectionModel.forward(
      outputs, humanDetectionModel.getUnconnectedOutLayersNames());

  float* data = reinterpret_cast<float*>(outputs[0].data);

  // Analyze detection outputs for specific classes
  for (int i = 0; i < 25200; ++i) {
    float confidence = data[4];  // Confidence score of the detection
    if (confidence > 0.35) {     // Confidence threshold
      float class_score = data[5];
      cv::Mat scores(1, classNames.size(), CV_32FC1, class_score);
      cv::Point class_id;
      double max_score;
      cv::minMaxLoc(scores, 0, &max_score, 0, &class_id);

      // Check if the detected class matches the objects of interest
      if (class_id.x == 0 || class_id.x == 45) {
        objectFound = true;
        return true;
      }
    }
  }

  // If no object of interest is found, update the state and return false
  objectFound = false;
  return false;
}

/**
 * @brief Checks the detection status of the object of interest.
 * 
 * This function returns whether the object of interest was detected in the frames 
 * analyzed by the model. The detection state is updated internally after each frame analysis.
 * 
 * @return True if the object of interest is found, false otherwise.
 */
bool ObjectSearch::isObjectFound() const { return objectFound; }
