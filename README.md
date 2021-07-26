[//]: # (Image References)

[final_result]: ./writeup-images/final-result.jpg "Final Result"
[non_normalized]: ./writeup-images/non-normalized.jpg "Non Normalized"

# Extended Kalman Filter Project Writeup

In this project I use a kalman filter to estimate the state of a moving point object in (x,y) using measurements from a radar and lidar sensor.

Using the provided synethtic data, this filter acheives and RMSE of:
(0.0964, 0.0853, 0.4154, 0.4316) which is lower than the tolerance specified in the project rubric.

---

## Program Flow

The program flow follows the outline from the course and uses mostly code snippets from the lesson throughout.

* Initialize filter vectors and matrices in the ctor of ```FusionEKF```
* Initialize initial state in ```FusionEKF::ProcessMeasurement```
* Update the state in ```ProcessMeasurement``` using new measurements
  from measurement_pack.

## Results

The following image shows the results of the kalman filter in green triangles,
compared to the raw measurements which are shown in red and blue circles.

![alt text][final_result]

Note in my initial implementation, I did not normalize the phi component of the radar measurement error ```y(1)``` in ```VectorXd y = z - h```. This produces an error when the heading crosses a threshold. This error can be easily visualized in the below image.

![alt text][non_normalized]


## Discussion

It would be interesting to add acceleration into the state vector since it seems like this could be predicted using the same mechanism that velocity is prediced from the laser by using the velocity measurements from the radar sensor.

Additionally, based on the program flow, the init method of ```KalmanFilter``` does not seem useful (and was therefore not used). Each member vector/matrix of the KalmanFilter class was set explicitly in the ctor the ```FusionEKF``` class.
