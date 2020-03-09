// it is only a simple demo of serial pid and code style is not followed.
#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "PID.h"

using namespace std;
using namespace cv;

int main()
{
    // reference path given by planner
    vector<vector<double>> planner_ref_path; // x,y,v,a,

    // create a reference path (x,y,v,a)
    vector<double> planner_ref_point = {0, 0, 0, 0.5};
    const double dt = 0.2;
    const int points_num = 100;

    for (int num_points = 0; num_points < points_num; num_points++)
    {
        planner_ref_path.emplace_back(planner_ref_point);
        planner_ref_point[0] += planner_ref_point[2] * dt + 1 / 2 * planner_ref_point[3] * pow(dt, 2);
        planner_ref_point[2] += planner_ref_point[3] * dt;
    }

    int simulation_time = 0;
    // car start position
    double car_x = 0, car_y = 0, car_v = 0;
    vector<double> car_point = {car_x, car_y, car_v};
    vector<vector<double>> car_position;
    // image definition
    const int img_size_height = 600;
    const int img_size_width = 1200;
    Mat img(img_size_height, img_size_width, CV_8UC3, Scalar(255, 255, 255));
    imshow("Test", img);

    Scalar ref_color = Scalar(255, 255, 0);
    Scalar real_color = Scalar(255, 0, 255);
    const int pixel_scale_width = 12;
    const int pixel_scale_height = 5;

    putText(img, "reference position", Point(img_size_width/3, 10), 1, 1, ref_color);
    putText(img, "real position", Point(img_size_width/2, 10), 1, 1, real_color);
    PID position_controller, velocity_controller;
    position_controller.Init(0.08, 0.004, 0); // you can play with dif pid parameters and see the performance
    velocity_controller.Init(0.05, 0.001, 0);

    while (simulation_time < points_num - 1)
    {
        car_position.emplace_back(car_point);
        int index = simulation_time + 1;

        //2 calculate velocity compensation through position controller
        double position_error = planner_ref_path[index][0] - car_point[0];
        position_controller.UpdateError(position_error);
        double velocity_compensation = position_controller.TotalError();

        //3 calculate acceleration compensation through velocity controller
        velocity_controller.UpdateError(planner_ref_path[index][2] + velocity_compensation - car_point[2]);
        double acceleration = velocity_controller.TotalError() + planner_ref_point[3];
        car_point[0] += car_point[2] * dt + 0.5 * acceleration * pow(dt, 2);
        car_point[2] += acceleration * dt;

        // draw img x_pos verses time
        cv::Point circle_center_ref, circle_center_real;
        circle_center_ref.x = planner_ref_path[index][0] * pixel_scale_width;
        circle_center_ref.y = simulation_time * pixel_scale_height;

        circle_center_real.x = car_point[0] * pixel_scale_width;
        circle_center_real.y = simulation_time * pixel_scale_height;

        cv::circle(img, circle_center_ref, 3, ref_color, -1);
        cv::circle(img, circle_center_real, 3, real_color, -1);
        imshow("Test", img); // show img
        waitKey(200);
        simulation_time++;
    }
    waitKey();
    return 0;
}
