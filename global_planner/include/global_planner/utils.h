/**
 * @file utils.h
 * @brief 全局规划器工具函数库
 *
 * 提供随机数生成、四元数转换、角度计算等通用工具函数
 */
#ifndef GLOBALPLANNER_UTILS_H
#define GLOBALPLANNER_UTILS_H
#include <random>                               // 随机数生成
#include <tf2/LinearMath/Quaternion.h>          // TF2四元数数学库
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // TF2与geometry_msgs的转换
#include <geometry_msgs/Quaternion.h>           // ROS几何消息类型

namespace globalPlanner{
	const double PI_const = 3.1415926;  // 圆周率常量

	/**
	 * @brief 生成指定范围内的均匀分布随机数
	 * @param min 最小值
	 * @param max 最大值
	 * @return [min, max] 范围内的随机浮点数
	 */
	inline double randomNumber(double min, double max){
		std::random_device rd;
		std::mt19937 mt(rd());
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(mt);
	}

	/**
	 * @brief 将欧拉角（Roll-Pitch-Yaw）转换为四元数
	 * @param roll  横滚角（弧度）
	 * @param pitch 俯仰角（弧度）
	 * @param yaw   偏航角（弧度）
	 * @return 对应的四元数消息
	 */
    inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
    	// 将偏航角限制在 [-PI, PI] 范围内
    	if (yaw > PI_const){
    		yaw = yaw - 2*PI_const;
    	}
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

	/**
	 * @brief 从四元数中提取偏航角（yaw）
	 * @param quat 输入的四元数消息
	 * @return 偏航角（弧度），范围 [0, 2π]
	 */
    inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	double roll, pitch, yaw;
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    	return yaw;
    }

	/**
	 * @brief 从四元数中提取完整的欧拉角（Roll, Pitch, Yaw）
	 * @param quat  输入的四元数消息
	 * @param roll  输出横滚角（弧度）
	 * @param pitch 输出俯仰角（弧度）
	 * @param yaw   输出偏航角（弧度）
	 */
    inline void rpy_from_quaternion(const geometry_msgs::Quaternion& quat, double &roll, double &pitch, double &yaw){
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

	/**
	 * @brief 计算两个向量之间的夹角
	 * @param a 向量a
	 * @param b 向量b
	 * @return 夹角（弧度），范围 [0, π]
	 *
	 * 使用 atan2(||a×b||, a·b) 公式计算，考虑了向量方向
	 */
	inline double angleBetweenVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b){
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }

	/**
	 * @brief 计算两个偏航角之间的角度差
	 * @param yaw1 偏航角1（弧度）
	 * @param yaw2 偏航角2（弧度）
	 * @return 角度差（弧度），范围 [0, π]
	 *
	 * 将偏航角转换为单位向量后计算夹角，考虑角度环绕
	 */
    inline double angleDiff(double yaw1, double yaw2){
    	Eigen::Vector3d p1 (cos(yaw1), sin(yaw1), 0);
    	Eigen::Vector3d p2 (cos(yaw2), sin(yaw2), 0);
    	return angleBetweenVectors(p1, p2);
    }
}

#endif
