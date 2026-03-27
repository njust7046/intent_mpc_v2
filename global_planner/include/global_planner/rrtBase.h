/**
 * @file rrtBase.h
 * @brief RRT（快速随机树）规划器基类
 *
 * 提供RRT和RRT*算法的通用框架和接口
 * 支持N维空间的路径规划，基于KD树实现高效的最近邻搜索
 *
 * 主要特性：
 * - 模板化设计，支持任意维度空间
 * - 基于KD树的高效最近邻查询
 * - 碰撞检测接口
 * - 目标到达判断
 * - 路径回溯
 */
#ifndef RRTBASE_H
#define RRTBASE_H
#include <ros/ros.h>
#include <global_planner/KDTree.h>
#include <Eigen/Eigen>
#include <random>
#include <geometry_msgs/Pose.h>
#include <global_planner/utils.h>

using std::cout; using std::endl;

namespace globalPlanner{
	/**
	 * @class rrtBase
	 * @brief RRT规划器基类（模板类）
	 * @tparam N 空间维度（通常为3，表示3D空间）
	 *
	 * 提供RRT算法的核心功能和接口
	 * 派生类需要实现地图相关的虚函数
	 */
	template <std::size_t N>
	class rrtBase{
	private:
		ros::NodeHandle nh_;  ///< ROS节点句柄

	protected:
		// ========== 算法参数 ==========
		double delQ_;                             ///< 增量距离（每次扩展的步长）
		double dR_;                               ///< 目标到达判定半径
		KDTree::Point<N> start_;                  ///< 起点
		KDTree::Point<N> goal_;                   ///< 终点
		KDTree::Point<N> emptyToken_;             ///< 空标记（用于表示根节点的父节点）
		KDTree::KDTree<N, int> ktree_;            ///< KD树（存储所有采样节点）
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_;  ///< 父节点映射（用于路径回溯）
		std::vector<double> collisionBox_;        ///< 碰撞检测盒尺寸 (lx, ly, lz)
		std::vector<double> envBox_;              ///< 环境边界 (xmin, xmax, ymin, ymax, zmin, zmax)
		double connectGoalRatio_;                 ///< 直接连接目标的概率
		double timeout_;                          ///< 规划超时时间 (秒)

	public:
		// ========== 构造与析构 ==========
		/**
		 * @brief 默认构造函数
		 */
		rrtBase();

		/**
		 * @brief 构造函数（使用Point类型）
		 * @param start 起点
		 * @param goal 终点
		 * @param collisionBox 碰撞检测盒尺寸
		 * @param envBox 环境边界
		 * @param delQ 增量距离
		 * @param dR 目标到达半径
		 * @param connectGoalRatio 连接目标概率
		 * @param timeout 超时时间
		 */
		rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		/**
		 * @brief 构造函数（使用vector类型）
		 */
		rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		/**
		 * @brief 构造函数（带ROS节点句柄，使用Point类型）
		 */
		rrtBase(const ros::NodeHandle &nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		/**
		 * @brief 构造函数（带ROS节点句柄，使用vector类型）
		 */
		rrtBase(const ros::NodeHandle &nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		/**
		 * @brief 构造函数（仅参数，需后续更新起点和终点）
		 */
		rrtBase(std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		/**
		 * @brief 虚析构函数
		 */
		virtual ~rrtBase();

		// ========== 纯虚函数（派生类必须实现） ==========
		/**
		 * @brief 更新地图（根据不同地图表示实现）
		 */
		virtual void updateMap() = 0;

		/**
		 * @brief 碰撞检测
		 * @param q 待检测的点
		 * @return 是否发生碰撞
		 */
		virtual bool checkCollision(const KDTree::Point<N>& q) = 0;

		/**
		 * @brief 在有效空间内随机采样
		 * @param qRand 输出的随机采样点
		 */
		virtual void randomConfig(KDTree::Point<N>& qRand) = 0;

		/**
		 * @brief 执行路径规划（核心函数）
		 * @param plan 输出的路径
		 */
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan) = 0;

		// ========== RRT核心算法函数 ==========
		/**
		 * @brief 在树中查找最近的顶点
		 * @param qKey 查询点
		 * @param qNear 输出的最近顶点
		 */
		void nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear);

		/**
		 * @brief 引导函数：从qNear向qRand扩展固定步长
		 * @param qNear 最近节点
		 * @param qRand 随机采样点
		 * @param qNew 输出的新节点
		 */
		void newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew);

		/**
		 * @brief 从目标回溯路径
		 * @param qGoal 目标点
		 * @param plan 输出的路径
		 */
		void backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan);

		/**
		 * @brief 判断是否到达目标
		 * @param q 当前点
		 * @return 是否到达
		 */
		bool isReach(const KDTree::Point<N>& q);

		/**
		 * @brief 添加新顶点到RRT树
		 * @param qNew 新顶点
		 */
		void addVertex(const KDTree::Point<N>& qNew);

		/**
		 * @brief 添加新边到RRT树
		 * @param qNear 父节点
		 * @param qNew 子节点
		 */
		void addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);

		/**
		 * @brief 检查两节点间是否无边连接
		 * @param qNear 节点1
		 * @param qNew 节点2
		 * @return 是否无边
		 */
		bool hasNoEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);

		// ========== 起点更新函数 ==========
		/**
		 * @brief 更新起点（vector类型）
		 */
		void updateStart(const std::vector<double>& newStart);

		/**
		 * @brief 更新起点（Eigen类型）
		 */
		void updateStart(const Eigen::Vector3d& newStart);

		/**
		 * @brief 更新起点（Point类型）
		 */
		void updateStart(const KDTree::Point<N>& newStart);

		/**
		 * @brief 更新起点（ROS Pose类型）
		 */
		void updateStart(const geometry_msgs::Pose& newStart);

		// ========== 终点更新函数 ==========
		/**
		 * @brief 更新终点（vector类型）
		 */
		void updateGoal(const std::vector<double>& newGoal);

		/**
		 * @brief 更新终点（Eigen类型）
		 */
		void updateGoal(const Eigen::Vector3d& newGoal);

		/**
		 * @brief 更新终点（Point类型）
		 */
		void updateGoal(const KDTree::Point<N>& newGoal);

		/**
		 * @brief 更新终点（ROS Pose类型）
		 */
		void updateGoal(const geometry_msgs::Pose& newGoal);

		/**
		 * @brief 清空RRT树
		 */
		void clearRRT();

		// ========== 访问器函数 ==========
		/**
		 * @brief 获取起点
		 */
		KDTree::Point<N> getStart();

		/**
		 * @brief 获取终点
		 */
		KDTree::Point<N> getGoal();

		/**
		 * @brief 获取碰撞检测盒
		 */
		std::vector<double> getCollisionBox();

		/**
		 * @brief 获取环境边界
		 */
		std::vector<double> getEnvBox();

		/**
		 * @brief 获取到达半径
		 */
		double getReachRadius();

		/**
		 * @brief 获取父节点字典（边信息）
		 */
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> getParentDict();

		/**
		 * @brief 获取连接目标概率
		 */
		double getConnectGoalRatio();

		/**
		 * @brief 获取超时时间
		 */
		double getTimeout();

	};

	// ========== 函数实现（模板类需在头文件中实现） ==========

	/**
	 * @brief 默认构造函数实现
	 * 初始化空标记为特殊值-11311
	 */
	template <std::size_t N>
	rrtBase<N>::rrtBase(){
		this->emptyToken_[0] = -11311;
	};

	/**
	 * @brief 构造函数实现（Point类型）
	 */
	template <std::size_t N>
	rrtBase<N>::rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311;
		this->parent_[start_] = this->emptyToken_; // 设置起点的父节点为空标记
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		KDTree::Point<N> startp = KDTree::vec2Point<N>(start);
		KDTree::Point<N> goalp = KDTree::vec2Point<N>(goal);
		this->start_ = startp;
		this->goal_ = goalp;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}


	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout) 
	: nh_(nh), collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: nh_(nh), collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		KDTree::Point<N> startp = KDTree::vec2Point<N>(start);
		KDTree::Point<N> goalp = KDTree::vec2Point<N>(goal);
		this->start_ = startp;
		this->goal_ = goalp;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		cout << "[RRTPlanner]: Please update start and goal!" << endl;
		this->emptyToken_[0] = -11311; 
	}

	template <std::size_t N>
	void rrtBase<N>::nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear){
		this->ktree_.nearestNeighbor(qKey, qNear);
	}

	template <std::size_t N>
	rrtBase<N>::~rrtBase(){

	}

	template <std::size_t N>
	void rrtBase<N>::newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew){
		double distance = Distance(qNear, qRand);
		KDTree::Point<N> direction = qRand - qNear;
		qNew = qNear + (this->delQ_/distance) * direction;
	}

	template <std::size_t N>
	void rrtBase<N>::backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan){
		KDTree::Point<N> ptr = qGoal;
		while (ptr[0] != this->emptyToken_[0]){
			plan.push_back(ptr);
			ptr = this->parent_[ptr];
		}
		std::reverse(plan.begin(), plan.end());
	}

	template <std::size_t N>
	bool rrtBase<N>::isReach(const KDTree::Point<N>& q){
		return KDTree::Distance(q, this->goal_) <= this->dR_;
	}

	template <std::size_t N>
	void rrtBase<N>::addVertex(const KDTree::Point<N>& qNew){
		this->ktree_.insert(qNew);
	}

	template <std::size_t N>
	void rrtBase<N>::addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		this->parent_[qNew] = qNear;
	}

	template <std::size_t N>
	bool rrtBase<N>::hasNoEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		if (qNear == qNew){
			return false;
		}
		return this->parent_[qNear] != qNew;
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const std::vector<double>& newStart){
		KDTree::Point<N> newStartp = KDTree::vec2Point<N>(newStart);
		this->updateStart(newStartp);
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const Eigen::Vector3d& newStart){
		KDTree::Point<N> newStartp = KDTree::eig2Point<N>(newStart);
		this->updateStart(newStartp);
	}
	
	template <std::size_t N>
	void rrtBase<N>::updateStart(const KDTree::Point<N>& newStart){
		this->start_ = newStart;
		this->parent_.clear();
		this->parent_[this->start_] = this->emptyToken_;
		if (not this->ktree_.empty()){
			this->clearRRT();
		}
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const geometry_msgs::Pose& newStart){
		KDTree::Point<N> newStartp;
		newStartp[0] = newStart.position.x; newStartp[1] = newStart.position.y; newStartp[2] = newStart.position.z;
		this->updateStart(newStartp);
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const std::vector<double>& newGoal){
		KDTree::Point<N> newGoalp = KDTree::vec2Point<N>(newGoal);
		this->updateGoal(newGoalp);
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const Eigen::Vector3d& newGoal){
		KDTree::Point<N> newGoalp = KDTree::eig2Point<N>(newGoal);
		this->updateGoal(newGoalp);
	}
	
	template <std::size_t N>
	void rrtBase<N>::updateGoal(const KDTree::Point<N>& newGoal){
		this->goal_ = newGoal;
		this->parent_.clear();
		this->parent_[this->start_] = this->emptyToken_;
		if (not this->ktree_.empty()){
			this->clearRRT();
		}
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const geometry_msgs::Pose& newGoal){
		KDTree::Point<N> newGoalp;
		newGoalp[0] = newGoal.position.x; newGoalp[1] = newGoal.position.y; newGoalp[2] = newGoal.position.z;
		this->updateGoal(newGoalp);
	}

	template <std::size_t N>
	void rrtBase<N>::clearRRT(){
		this->ktree_.~KDTree();
		new (&this->ktree_) KDTree::KDTree<N, int> ();
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getStart(){
		return this->start_;
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getGoal(){
		return this->goal_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getCollisionBox(){
		return this->collisionBox_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getEnvBox(){
		return this->envBox_;
	}

	template <std::size_t N>
	double rrtBase<N>::getReachRadius(){
		return this->dR_;
	}

	template <std::size_t N>
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> rrtBase<N>::getParentDict(){
		return this->parent_;
	}

	template <std::size_t N>
	double rrtBase<N>::getConnectGoalRatio(){
		return this->connectGoalRatio_;
	}

	template <std::size_t N>
	double rrtBase<N>::getTimeout(){
		return this->timeout_;
	}
}

#endif