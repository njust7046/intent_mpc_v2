/**
 * @file dep.h
 * @brief 动态探索规划器（Dynamic Exploration Planner, DEP）头文件
 *
 * 该模块实现了基于概率路线图（PRM）和信息增益的未知环境探索算法
 * 主要用于无人机在动态环境中的自主探索和路径规划
 *
 * 核心功能：
 * - 基于传感器视野的前沿区域检测
 * - 增量式概率路线图构建
 * - 信息增益计算与最优视点选择
 * - 候选路径生成与最佳路径选择
 *
 * 参考论文：
 * Zhefan Xu et al., "Autonomous UAV Exploration of Dynamic Environments via
 * Incremental Sampling and Probabilistic Roadmap", IEEE RA-L, 2021
 */

#ifndef DEP_H
#define DEP_H

#include <map_manager/dynamicMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <global_planner/PRMKDTree.h>
#include <global_planner/PRMAstar.h>
#include <global_planner/utils.h>
#include <opencv2/opencv.hpp>


namespace globalPlanner{
	/**
	 * @class DEP
	 * @brief 动态探索规划器类
	 *
	 * 实现基于增量采样和概率路线图的未知环境探索算法
	 * 通过信息增益评估和路径优化实现高效的自主探索
	 */
	class DEP{
	private:
		// ========== ROS相关成员 ==========
		std::string ns_;                          ///< ROS命名空间
		std::string hint_;                        ///< 日志提示前缀

		ros::NodeHandle nh_;                      ///< ROS节点句柄
		ros::Publisher roadmapPub_;               ///< 路线图可视化发布器
		ros::Publisher candidatePathPub_;         ///< 候选路径可视化发布器
		ros::Publisher bestPathPub_;              ///< 最佳路径发布器
		ros::Publisher frontierVisPub_;           ///< 前沿区域可视化发布器
		ros::Subscriber odomSub_;                 ///< 里程计订阅器
		ros::Timer visTimer_;                     ///< 可视化定时器

		nav_msgs::Odometry odom_;                 ///< 当前里程计数据
		std::shared_ptr<mapManager::occMap> map_; ///< 占据栅格地图指针
		std::shared_ptr<PRM::KDTree> roadmap_;    ///< 概率路线图（基于KD树）




		// ========== 算法参数 ==========
		double vel_ = 1.0;                        ///< 线速度 (m/s)
		double angularVel_ = 1.0;                 ///< 角速度 (rad/s)
		std::string odomTopic_;                   ///< 里程计话题名称
		Eigen::Vector3d globalRegionMin_, globalRegionMax_;  ///< 全局采样区域边界
		Eigen::Vector3d localRegionMin_, localRegionMax_;    ///< 局部采样区域边界（相对于当前位置）
		int localSampleThresh_;                   ///< 局部区域采样数量阈值
		int globalSampleThresh_;                  ///< 全局区域采样数量阈值
		int frontierSampleThresh_;                ///< 前沿区域采样数量阈值
		double distThresh_;                       ///< 节点距离阈值（用于剪枝）
		double safeDistXY_;                       ///< XY平面安全距离（碰撞检测）
		double safeDistZ_;                        ///< Z轴安全距离（碰撞检测）
		bool safeDistCheckUnknown_;               ///< 是否将未知区域视为障碍物
		double horizontalFOV_;                    ///< 传感器水平视场角 (rad)
		double verticalFOV_;                      ///< 传感器垂直视场角 (rad)
		double dmin_;                             ///< 传感器最小探测距离 (m)
		double dmax_;                             ///< 传感器最大探测距离 (m)
		int nnNum_;                               ///< 路线图构建时的近邻数量
		int nnNumFrontier_;                       ///< 前沿节点连接的近邻数量
		double maxConnectDist_;                   ///< 节点连接的最大距离
		std::vector<double> yaws_;                ///< 离散化的偏航角集合（用于信息增益计算）
		double minVoxelThresh_;                   ///< 最小体素阈值（用于候选点筛选）
		int minCandidateNum_;                     ///< 最少候选路径数量
		int maxCandidateNum_;                     ///< 最多候选路径数量
		double updateDist_;                       ///< 触发信息增益更新的移动距离阈值
		double yawPenaltyWeight_;                 ///< 偏航角变化惩罚权重

		// ========== 运行时数据 ==========
		bool odomReceived_ = false;               ///< 是否已接收到里程计数据
		Eigen::Vector3d position_;                ///< 当前位置
		double currYaw_;                          ///< 当前偏航角
		std::deque<Eigen::Vector3d> histTraj_;    ///< 历史轨迹（用于信息增益更新）
		// std::vector<std::shared_ptr<PRM::Node>> prmNodeVec_; // 所有节点（旧版本）
		std::unordered_set<std::shared_ptr<PRM::Node>> prmNodeVec_;  ///< 所有PRM节点集合
		std::vector<std::shared_ptr<PRM::Node>> goalCandidates_;     ///< 目标候选点集合
		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths_;  ///< 候选路径集合
		std::vector<std::shared_ptr<PRM::Node>> bestPath_;           ///< 当前最佳路径
		std::vector<std::pair<Eigen::Vector3d, double>> frontierPointPairs_;  ///< 前沿点对（位置，权重）


	public:
		// ========== 构造与初始化 ==========
		/**
		 * @brief 构造函数
		 * @param nh ROS节点句柄
		 */
		DEP(const ros::NodeHandle& nh);

		/**
		 * @brief 设置占据栅格地图
		 * @param map 地图指针
		 */
		void setMap(const std::shared_ptr<mapManager::occMap>& map);

		/**
		 * @brief 加载速度参数
		 * @param vel 线速度 (m/s)
		 * @param angularVel 角速度 (rad/s)
		 */
		void loadVelocity(double vel, double angularVel);

		/**
		 * @brief 从ROS参数服务器初始化参数
		 */
		void initParam();

		/**
		 * @brief 初始化内部模块（如路线图）
		 */
		void initModules();

		/**
		 * @brief 注册ROS发布器
		 */
		void registerPub();

		/**
		 * @brief 注册ROS回调函数
		 */
		void registerCallback();

		// ========== 核心规划函数 ==========
		/**
		 * @brief 执行完整的探索规划流程
		 * @return 是否成功生成规划路径
		 *
		 * 流程：前沿检测 -> 构建路线图 -> 剪枝 -> 更新信息增益 ->
		 *       选择候选点 -> 生成候选路径 -> 选择最佳路径
		 */
		bool makePlan();

		/**
		 * @brief 获取最佳路径（转换为ROS消息格式）
		 * @return 路径消息
		 */
		nav_msgs::Path getBestPath();

		/**
		 * @brief 检测前沿区域（已知与未知空间的边界）
		 * @param frontierPointPairs 输出的前沿点对（位置，权重）
		 *
		 * 使用形态学操作检测地图中的前沿区域
		 */
		void detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs);

		/**
		 * @brief 构建增量式概率路线图
		 *
		 * 在局部、全局和前沿区域进行采样，并连接近邻节点
		 */
		void buildRoadMap();

		/**
		 * @brief 剪枝路线图节点
		 *
		 * 移除距离当前位置过远或无效的节点
		 */
		void pruneNodes();

		/**
		 * @brief 更新所有节点的信息增益
		 *
		 * 基于传感器模型计算每个节点可观测到的未知体素数量
		 */
		void updateInformationGain();

		/**
		 * @brief 获取最佳视点候选集
		 * @param goalCandidates 输出的候选目标节点
		 *
		 * 根据信息增益选择最有价值的探索目标
		 */
		void getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);

		/**
		 * @brief 为候选目标生成路径
		 * @param goalCandidates 候选目标节点
		 * @param candidatePaths 输出的候选路径集合
		 * @return 是否成功生成路径
		 *
		 * 使用A*算法在路线图上搜索路径
		 */
		bool findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates,  std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths);

		/**
		 * @brief 从候选路径中选择最佳路径
		 * @param candidatePaths 候选路径集合
		 * @param bestPath 输出的最佳路径
		 *
		 * 综合考虑路径长度、信息增益和偏航角变化
		 */
		void findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath);


		// ========== 回调函数 ==========
		/**
		 * @brief 里程计回调函数
		 * @param odom 里程计消息
		 */
		void odomCB(const nav_msgs::OdometryConstPtr& odom);

		/**
		 * @brief 可视化定时器回调函数
		 * @param event 定时器事件
		 */
		void visCB(const ros::TimerEvent&);



		// ========== 辅助函数 ==========
		/**
		 * @brief 检查位置是否有效（无碰撞）
		 * @param p 待检查的位置
		 * @return 是否有效
		 */
		bool isPosValid(const Eigen::Vector3d& p);

		/**
		 * @brief 检查位置是否有效（自定义安全距离）
		 * @param p 待检查的位置
		 * @param safeDistXY XY平面安全距离
		 * @param safeDistZ Z轴安全距离
		 * @return 是否有效
		 */
		bool isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ);

		/**
		 * @brief 在指定边界框内随机采样配置
		 * @param minRegion 边界框最小值
		 * @param maxRegion 边界框最大值
		 * @return 采样得到的节点
		 */
		std::shared_ptr<PRM::Node> randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion);

		/**
		 * @brief 检查两节点是否满足传感器距离条件
		 * @param n1 节点1
		 * @param n2 节点2
		 * @return 是否在传感器探测范围内
		 */
		bool sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2);

		/**
		 * @brief 检查采样点是否在传感器视场内
		 * @param sample 采样点位置
		 * @param pos 传感器位置
		 * @return 是否在视场内
		 */
		bool sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos);

		/**
		 * @brief 计算节点的未知体素数量（信息增益）
		 * @param n 待计算的节点
		 * @param yawNumVoxels 输出各偏航角对应的未知体素数
		 * @return 最大未知体素数量
		 */
		int calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels);

		/**
		 * @brief 计算路径长度
		 * @param path 路径节点序列
		 * @return 路径总长度
		 */
		double calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path);

		/**
		 * @brief 路径快捷方式优化（移除冗余节点）
		 * @param path 原始路径
		 * @param pathSc 输出的优化路径
		 */
		void shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc);

		/**
		 * @brief 基于权重的加权采样
		 * @param weights 权重向量
		 * @return 采样得到的索引
		 */
		int weightedSample(const std::vector<double>& weights);

		/**
		 * @brief 从前沿区域采样点
		 * @param sampleWeights 采样权重
		 * @return 采样得到的节点
		 */
		std::shared_ptr<PRM::Node> sampleFrontierPoint(const std::vector<double>& sampleWeights);

		/**
		 * @brief 从节点向目标扩展（RRT风格）
		 * @param n 起始节点
		 * @param target 目标节点
		 * @return 扩展得到的新节点
		 */
		std::shared_ptr<PRM::Node> extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target);

		// ========== 可视化函数 ==========
		/**
		 * @brief 发布路线图可视化
		 */
		void publishRoadmap();

		/**
		 * @brief 发布候选路径可视化
		 */
		void publishCandidatePaths();

		/**
		 * @brief 发布最佳路径
		 */
		void publishBestPath();

		/**
		 * @brief 发布前沿区域可视化
		 */
		void publishFrontier();
	};
}


#endif


