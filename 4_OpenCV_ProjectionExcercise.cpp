  
#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <stdlib.h>
#include <vector>
//#include <experimental/filesystem>
//namespace fs =  std::experimental::filesystem::v1;

using namespace cv;

// Read next video timestamp to make sure the video is synchronized
double read_next_video_timestamp(std::ifstream &file)
{
  std::string str;
  std::getline(file, str);

  std::string data = str.substr(str.find(':') + 2, str.find(',') - (str.find(':') + 2));
  double hr = std::stof(data.substr(0, data.find(':')));
  data.erase(0, data.find(':') + 1);
  double min = std::stof(data.substr(0, data.find(':')));
  data.erase(0, data.find(':') + 1);
  double sec = std::stof(data);

  sec = (hr * 60 + min) * 60 + sec;

  return sec;
};

//read into point_array points until the timestamp is passed (minimum 1 line)
int read_until(std::ifstream &file, std::vector<cv::Point3d> &point_array, double timestamp, std::vector<std::string> &lidar_str)
{
  std::string str;
  double new_timestamp = 0.0;
  double lo = timestamp - new_timestamp;
  int ctr = 0;
  while (lo > 0.0)
  {
    if (file.eof())
    {
      return -1;
    }
    std::getline(file, str);
    //const char* str = istr.c_str();
    new_timestamp = std::stod(str.substr(1, str.find(':')));

    std::string data = str.substr(str.find(':') + 1, str.find(']') - 1);
    while (data.length() > 1)
    {

      std::string entry = data.substr(0, data.find(';'));

      double d = std::stof(entry.substr(0, entry.find('|')));
      entry.erase(0, entry.find('|') + 1);
      double h = std::stof(entry.substr(0, entry.find('|')));
      entry.erase(0, entry.find('|') + 1);
      double v = std::stof(entry.substr(0, entry.find('|')));
      //entry.erase(0, entry.find('|'));

      data.erase(0, data.find(';') + 1);

      double diag = d * cos(v);

      point_array.push_back(Point3d(diag * cos(h), diag * sin(h), d * -sin(v)));

      lidar_str.push_back(str);
    }
    ctr++;
    lo = timestamp - new_timestamp;
  }
  return ctr;
};

//read objects into an array until the timestamp is passed (minimum 1 line)
int read_obj_until(std::ifstream &file, std::vector<cv::Vec4d> &object_array, double timestamp, std::vector<cv::Point3d> &extra_params)
{
  std::string str;
  double new_timestamp = 0.0;
  double lo = timestamp - new_timestamp;
  int ctr = 0;
  while (lo > 0.0)
  {
    if (file.eof())
    {
      return -1;
    }
    std::getline(file, str);
    new_timestamp = std::stod(str.substr(1, str.find(':')));

    std::string data = str.substr(str.find(':') + 1, str.find(']') - 1);
    while (data.length() > 1)
    {

      std::string entry = data.substr(0, data.find(';'));

      double center_x = std::stof(entry.substr(1, data.find(',')));
      entry.erase(0, entry.find(',') + 2);
      double center_y = std::stof(entry.substr(0, entry.find(')')));
      entry.erase(0, entry.find(')') + 2);

      //first 2 extra parameters: velocity x and velocity y
      double vel_x = std::stof(entry.substr(1, data.find(',')));
      entry.erase(0, entry.find(',') + 2);
      double vel_y = std::stof(entry.substr(0, entry.find(')')));
      entry.erase(0, entry.find(')') + 2);

      //remove second velocity parameter
      entry.erase(0, entry.find(')') + 2);

      //read width and height
      double width = std::stof(entry.substr(1, data.find(',')));
      entry.erase(0, entry.find(',') + 2);
      double height = std::stof(entry.substr(0, entry.find(')')));
      entry.erase(0, entry.find(')') + 2);

      int age = std::stoi(entry.substr(0, entry.find('|')));

      data.erase(0, data.find(';') + 1);

      object_array.push_back(cv::Vec4d(center_x - width / 2.0, center_y - height / 2.0, center_x + width / 2.0, center_y + height / 2.0));
      extra_params.push_back(Point3d(vel_x, vel_y, age));
    }
    ctr++;
    lo = timestamp - new_timestamp;
  }
  return ctr;
};

// Calculates rotation matrix given euler angles.
cv::Mat eulerAnglesToRotationMatrix(double *theta)
{
  // Calculate rotation about x axis
  cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                 0, cos(theta[0]), -sin(theta[0]),
                 0, sin(theta[0]), cos(theta[0]));

  // Calculate rotation about y axis
  cv::Mat R_y = (Mat_<double>(3, 3) << cos(theta[1]), 0, sin(theta[1]),
                 0, 1, 0,
                 -sin(theta[1]), 0, cos(theta[1]));

  // Calculate rotation about z axis
  cv::Mat R_z = (Mat_<double>(3, 3) << cos(theta[2]), -sin(theta[2]), 0,
                 sin(theta[2]), cos(theta[2]), 0,
                 0, 0, 1);

  // Combined rotation matrix
  cv::Mat R = R_z * R_y * R_x;

  return R;
}

// Return true if 2 given boxes overlap anywhere
bool overlap(cv::Vec4d obj1, cv::Vec4d obj2)
{
  return !(obj1[0] > obj2[2] || obj2[0] > obj1[2] || obj1[1] < obj2[3] || obj2[1] < obj1[3]);
}

//Return the distance between point and a given rectangle
double distance(cv::Vec4d rect, cv::Point3d p)
{
  double dx = std::max(rect[0] - p.x, p.x - rect[2]);
  dx = std::max(dx, 0.0);
  double dy = std::max(rect[1] - p.y, p.y - rect[3]);
  dy = std::max(dy, 0.0);
  return std::sqrt(dx * dx + dy * dy);
}

//Fuses all boxes in the given array that overlap
void fuse_similar_objects(std::vector<cv::Vec4d> objects)
{
  for (int i = 0; i < objects.size(); i++)
  {
    for (int j = i + 1; j < objects.size(); j++)
    {
      if (overlap(objects[i], objects[j]))
      {
        objects[i][0] = std::min(objects[i][0], objects[j][0]);
        objects[i][1] = std::min(objects[i][1], objects[j][1]);
        objects[i][2] = std::min(objects[i][0], objects[j][0]);
        objects[i][3] = std::min(objects[i][1], objects[j][1]);
        objects.erase(objects.begin() + j);
        i = 0;
        j = 0;
      }
    }
  }
}

int main(int, char **)
{
  double rv[3] = {0., 0., 0.};
  rv[0] = -0.004;
  rv[1] = -0.024;
  rv[2] = 0.036;


  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  tvec.at<double>(0) = 0.;
  tvec.at<double>(1) = -0.15;
  tvec.at<double>(2) = -0.3;

  cv::Mat K(3, 3, cv::DataType<double>::type);
 
  K.at<double>(0,0) = 1150; 
  K.at<double>(0,1) = 0.;
  K.at<double>(0,2) = 640;//960.;

  K.at<double>(1,0) = 0.;
  K.at<double>(1,1) = 1150;
  K.at<double>(1,2) = 360;//540;

  K.at<double>(2,0) = 0.;
  K.at<double>(2,1) = 0.;
  K.at<double>(2,2) = 1.;

  //Intrinsic distortion coefficeints
  cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 3.5543230203932972e-01;
  distCoeffs.at<double>(1) = -5.7444032014766666e-01;
  distCoeffs.at<double>(2) = 0.;
  distCoeffs.at<double>(3) = 0.;
  distCoeffs.at<double>(4) = 3.3972235806313755e+00;

  //Matrix for relative rotation of lidar and camera
  cv::Mat rvecR(3, 3, cv::DataType<double>::type);
  cv::Mat baseR(3, 3, cv::DataType<double>::type);
  baseR.at<double>(0, 0) = 0.;
  baseR.at<double>(1, 0) = 0.;
  baseR.at<double>(2, 0) = 1.;
  baseR.at<double>(0, 1) = -1.;
  baseR.at<double>(1, 1) = 0.;
  baseR.at<double>(2, 1) = 0.;
  baseR.at<double>(0, 2) = 0.;
  baseR.at<double>(1, 2) = -1.;
  baseR.at<double>(2, 2) = 0.;
  cv::Mat er = eulerAnglesToRotationMatrix(rv);
  rvecR = baseR * er;
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(rvecR, rvec);

  //Initialization of point arrays

  //Current frame for point projection
  Mat baseImg;
  //Points from lidar
  std::vector<cv::Point3d> points;
  //Projected points from lidar
  std::vector<cv::Point2d> projection_points;
  //Distances to points from lidar
  std::vector<double> distances;
  //Objects as detected by lidar
  std::vector<cv::Vec4d> objects;
  //Projected objects in opencv format
  std::vector<cv::Rect> projection_objects;

  //Some extra paremeters
  std::vector<cv::Point3d> extra_pars;
  std::vector<std::string> lpoints_str;

  //current timestamp
  double timestamp = 0.0;
  double prev_stamp = -100.0;
  //whether to play the video
  int play = 1;
  //in paused mode tells to read the next frame
  int nf = 0;

  //Files to use for point projection
  std::ifstream file("./lidarPoints.txt");
  std::ifstream ofile("./lidarObjects.txt");
  std::ifstream tfile("./lidarTimestamps.txt");
  VideoCapture cap("./lidarVideo.mp4");

  if (!cap.isOpened())
  {
    return -1; // check if we succeeded
  }

  //open lidar logs and read 2 lines from them before starting
  std::string str;
  std::string ostr;
  std::getline(file, str);
  std::getline(ofile, str);
  str = "";
  ostr = "";
  std::getline(ofile, ostr);
  std::getline(file, str);
  //std::cout << str << std::endl;

  //Configure offset, as zero for time for camera and lidar are different
  double orig_start = 0.0;
  double offset = -7.4;
  double next_timestamp = orig_start;

  //Initialize window for showing data
  namedWindow("fusion", 1);

  int fontFace = cv::FONT_HERSHEY_PLAIN;

  for (;;)
  {
    Mat frame;
    //If the video is playing, read the next frame
    if (play || nf)
    {
      nf = 0;
      cap >> frame;
      frame.copyTo(baseImg);
      timestamp = read_next_video_timestamp(tfile) + orig_start;
    }
    else
    {
      baseImg.copyTo(frame);
    }

    //Clear the arrays and read new data into them when the timestamps are passed
    if (timestamp >= next_timestamp)
    {
      if (timestamp > (orig_start - offset) && prev_stamp < (timestamp + offset))
      {
        points.clear();
        objects.clear();
        extra_pars.clear();
        lpoints_str.clear();
        int sz = read_until(file, points, timestamp + offset, lpoints_str);
        std::getline(file, str);
        if (sz < 0)
        {
          return 0;
        }
        read_obj_until(ofile, objects, timestamp + offset, extra_pars);
        prev_stamp = timestamp + offset;
      }
      next_timestamp = timestamp + 0.2;
    }

    //If there are Lidar points to parse
    if (points.size() > 0)
    {
      //Clear the arrays
      projection_points.clear();
      projection_objects.clear();
      distances.clear();
      //Project the 3D Lidar points onto a 2D pane, that can be shown on a picture
      projectPoints(points, rvec, tvec, K, distCoeffs, projection_points);
      for (int i = 0; i < points.size(); i++)
      {
        //std::cout << points[i].x << ":" << points[i].y << ":" << points[i].z << ";  ";
      }
      //std::cout << std::endl;
      for (int i = 0; i < projection_points.size(); i++)
      {
        //std::cout << projection_points[i].x << ":" << projection_points[i].y << ";  ";
      }
      //std::cout << std::endl;
      //For each point add it's distance to a separate array
      for (int i = 0; i < points.size(); i++)
      {
        distances.push_back(points[i].x);
      }

      //if lidar has detected objects
      if (objects.size() > 0)
      {
        //fuse similar objects together
        fuse_similar_objects(objects);
        for (int i = 0; i < objects.size(); i++)
        {
          projection_objects.push_back(cv::Rect(0, 0, 0, 0));
        }
        //PROBLEM: Lidar detects objects as 2d objects from top-down view, so it will not tell how high it is.
        //For visualizing objects you need to determine which point each object belongs to (use distance() function) and draw a box around those points.
        //This box needs to be saved in Rect object in projection_objects list
        //TODO: For every box in the objects list, you should have a box in projection_objects list that is over it.
        for (int i = 0; i < points.size(); i++)
        {
          double closest_distance = 10000;
          int closest_object_index = 0;
          for (int j = 0; j < objects.size(); j++) {
            if (distance(objects[j], points[i]) < closest_distance) {
              closest_distance = distance(objects[j], points[i]);
              closest_object_index = j;
            }
          }

          // Updating the rectangle
          cv::Rect update_rect = projection_objects[closest_object_index];
          cv::Point2d projectedPoint = projection_points[i];

          if (update_rect.x == 0 && update_rect.y == 0) {
            if (update_rect.width == 0 && update_rect.height == 0) {
            update_rect.x = projectedPoint.x;
            update_rect.y = projectedPoint.y;
            }
          }


          if (update_rect.x > projectedPoint.x) {
            int new_width = update_rect.width + update_rect.x - projectedPoint.x;
            update_rect.width = new_width;
            update_rect.x = projectedPoint.x;
          }
          if (update_rect.y > projectedPoint.y) {
            int new_height = update_rect.height + update_rect.y - projectedPoint.y;
            update_rect.height = new_height;
            update_rect.y = projectedPoint.y;
          }
          if (update_rect.x + update_rect.width < projectedPoint.x) {
            update_rect.width = projectedPoint.x - update_rect.x;
          }
          if (update_rect.y + update_rect.height < projectedPoint.y) {
            update_rect.height = projectedPoint.y - update_rect.y;
          }

          projection_objects[closest_object_index] = update_rect;
        }
      }

      points.clear();
      objects.clear();
    }
    //For each projected point, draw it and the distance on top of it.
    std::vector<cv::Rect> textBoxes;
    for (int i = 0; i < projection_points.size(); i++)
    {
      if (projection_points[i].x < 0 || projection_points[i].x > 1280 || projection_points[i].y < 0 || projection_points[i].y > 720)
      {
        continue;
      }
      int color = distances[i] * 255 / 150;
      if (color > 255)
      {
        color = 255;
      }
      //Do not draw points that are closer than 3 meters, since those are likely false positives
      if (distances[i] > 40)
      {
        circle(frame, projection_points[i], 2, Scalar(color, 0, 255 - color), 2);

        std::string dist = std::to_string(int(distances[i]));
        Point p = Point(projection_points[i].x + 3, projection_points[i].y - 5);

        //Check whether the point is too close to snother point with drawn distance. This way we don't draw distances on top of eachother
        bool printDist = true;
        for (int j = 0; j < textBoxes.size(); j++)
        {
          if (textBoxes[j].contains(p))
          {
            printDist = false;
            break;
          }
        }
        //If there is room, draw distance on top of the point
        if (printDist)
        {
          putText(frame, dist, p, fontFace, 1.3, Scalar(0, 255, 0), 2, 8);
          textBoxes.push_back(cv::Rect(p.x - 35, p.y - 17, 70, 35));
        }
      }
    }
    //For each projected object, draw a rectangle over it.
    for (int i = 0; i < projection_objects.size(); i++)
    {
      //TODO: Implement this function. Take example from how points are drawn and use rectangle() function to draw objects.
      //Look above how distance to points is calculated and display the distance to the object in question (average distance to all points of that object)
      //Boxes should be slightly bigger than the actual box containing points (approximately +10 pixels) as that is better for veiwing (looks cleaner)

      cv::Rect draw_rec = projection_objects[i];
      draw_rec.width += 20;
      draw_rec.height += 20;
      rectangle(frame, draw_rec, Scalar(255, 0, 200), 2);
    }

    imshow("fusion", frame);

    if (play)
    {
      int key = waitKey(40);
      switch (key)
      {
      case 'v':
        offset -= 0.2;
        //std::cout << "new offset: " << (offset) << std::endl;
        break;
      case 'b':
        offset += 0.2;
        //std::cout << "new offset: " << (offset) << std::endl;
        break;
      case 32:
        play = 0;
        break;
      case 27:
        return 0;
      }
    }
    else
    {
      int waitingForKey = 1;
      while (waitingForKey)
      {
        int key = waitKey(30);
        switch (key)
        {
        case 32:
          waitingForKey = 0;
          play = 1;
          break;
        case 'n':
          waitingForKey = 0;
          nf = 1;
          break;
        case 27:
          return 0;
        }
      }
    }
  }
  return 0;
}