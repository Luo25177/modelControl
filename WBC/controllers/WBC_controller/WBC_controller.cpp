#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "pid.hpp"
#include <Eigen>
#include <cmath>

using namespace webots;
using namespace std;

Eigen::MatrixXd pinv(Eigen::MatrixXd  A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	double  pinvtoler = 1.e-8; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = min(row,col);
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
	Eigen::MatrixXd singularValues_inv = svd.singularValues();//∆Ê“Ï÷µ
	Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i<k; ++i) {
			if (singularValues_inv(i) > pinvtoler)
					singularValues_inv(i) = 1.0 / singularValues_inv(i);
			else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i) 
	{
			singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());
	return X;
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  int timeStep = (int)robot->getBasicTimeStep();

  Motor* joint1_motor = new Motor("joint1_motor");
  Motor* joint2_motor = new Motor("joint2_motor");
  Motor* joint3_motor = new Motor("joint3_motor");

  PositionSensor *joint1_sensor = new PositionSensor("joint1_sensor");
  PositionSensor *joint2_sensor = new PositionSensor("joint2_sensor");
  PositionSensor *joint3_sensor = new PositionSensor("joint3_sensor");
  joint1_sensor->enable(timeStep);
  joint2_sensor->enable(timeStep);
  joint3_sensor->enable(timeStep);

  float joint1_angle;
  float joint2_angle;
  float joint3_angle;

  PID x_pid(1, 0, 0);
  PID y_pid(1, 0, 0);

  float x_target = 0.2;
  float y_target = 0.5;

  float l1 = 0.3;
  float l2 = 0.3;
  float l3 = 0.3;

  Eigen::MatrixXd j_x = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd j_y = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd j_x_pinv = Eigen::MatrixXd::Zero(3, 1);
  Eigen::MatrixXd j_y_pinv = Eigen::MatrixXd::Zero(3, 1);

  Eigen::MatrixXd n_x = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd n_y = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd I = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd q_dot = Eigen::MatrixXd::Zero(3, 1);

  I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  printf("%f", I(1, 1));
  while (robot->step(timeStep) != -1) {
    joint1_angle = joint1_sensor->getValue();
    joint2_angle = joint2_sensor->getValue();
    joint3_angle = joint3_sensor->getValue();
    j_x << l1 * cos(joint1_angle) + l2 * cos(joint1_angle + joint2_angle) + l3 * cos(joint1_angle + joint2_angle + joint3_angle),
      l2 * cos(joint1_angle + joint2_angle) + l3 * cos(joint1_angle + joint2_angle + joint3_angle),
      l3 * cos(joint1_angle + joint2_angle + joint3_angle);

    j_x << -l1 * sin(joint1_angle) - l2 * sin(joint1_angle + joint2_angle) - l3 * sin(joint1_angle + joint2_angle + joint3_angle),
      -l2 * sin(joint1_angle + joint2_angle) - l3 * sin(joint1_angle + joint2_angle + joint3_angle),
      -l3 * sin(joint1_angle + joint2_angle + joint3_angle);
           
    float x = l1 * sin(joint1_angle) + l2 * sin(joint1_angle + joint2_angle) + l3 * sin(joint1_angle + joint2_angle + joint3_angle);
    float y = l1 * cos(joint1_angle) + l2 * cos(joint1_angle + joint2_angle) + l3 * cos(joint1_angle + joint2_angle + joint3_angle);

    j_x_pinv = pinv(j_x);
    j_y_pinv = pinv(j_y);

    Eigen::MatrixXd temp1 = Eigen::MatrixXd::Zero(3, 3);
    temp1 = j_x_pinv * j_x;
    n_x = I - temp1;
    temp1 = j_y_pinv * j_y;
    n_y = I - temp1;

		Eigen::MatrixXd x_dot = Eigen::MatrixXd::Zero(1, 1);
    x_dot << x_pid.run(x, x_target);
		Eigen::MatrixXd y_dot = Eigen::MatrixXd::Zero(1, 1);
    y_dot << y_pid.run(y, y_target);

    temp << j_y * n_x;
    
    q_dot = j_x_pinv * x_dot - n_x * pinv(temp) * (y_dot - j_y * j_x_pinv * x_dot);

    float joint1_angle_dot_set = q_dot(0, 0);
    float joint2_angle_dot_set = q_dot(1, 0);
    float joint3_angle_dot_set = q_dot(2, 0);

    joint1_motor->setVelocity(joint1_angle_dot_set);
    joint2_motor->setVelocity(joint2_angle_dot_set);
    joint3_motor->setVelocity(joint3_angle_dot_set);
  };

  delete robot;
  return 0;
}
