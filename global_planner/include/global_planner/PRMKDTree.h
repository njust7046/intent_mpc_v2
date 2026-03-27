/**
 * @file PRMKDTree.h
 * @brief 基于KD树的概率路线图（PRM）数据结构
 *
 * 为DEP探索规划器提供高效的节点存储和查询功能
 * 结合了PRM的图结构和KD树的空间索引优势
 */

#ifndef PRMKDTREE_H
#define PRMKDTREE_H
#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <queue>


using std::cout; using std::endl;
namespace PRM{
	/**
	 * @struct Node
	 * @brief PRM路线图节点结构
	 *
	 * 包含位置、信息增益、树结构和图结构信息
	 * 用于DEP算法的增量式路线图构建和A*搜索
	 */
	struct Node{
		// ========== 位置与信息增益 ==========
		Eigen::Vector3d pos;                      ///< 节点3D位置
		int numVoxels = 0;                        ///< 可观测到的未知体素总数（信息增益）
		std::unordered_map<double, int> yawNumVoxels = {};  ///< 各偏航角对应的未知体素数

		// ========== KD树结构 ==========
		std::shared_ptr<Node> left = NULL;        ///< KD树左子节点
		std::shared_ptr<Node> right = NULL;       ///< KD树右子节点
		std::shared_ptr<Node> treeParent = NULL;  ///< KD树父节点

		// ========== A*搜索相关 ==========
		std::shared_ptr<Node> parent = NULL;      ///< A*搜索中的父节点
		double g = std::numeric_limits<double>::infinity();  ///< 从起点到当前节点的实际代价
		double f = std::numeric_limits<double>::infinity();  ///< 总代价估计 f = g + h

		// ========== 图结构 ==========
		bool newNode = false;                     ///< 是否为新添加的节点
		std::unordered_set<std::shared_ptr<Node>> adjNodes;  ///< 相邻节点集合（PRM图的边）

		/**
		 * @brief 构造函数
		 * @param p 节点位置
		 */
		Node (const Eigen::Vector3d& p){
			this->pos = p;
		}

		/**
		 * @brief 获取指定偏航角的未知体素数（插值计算）
		 * @param angle 偏航角（弧度）
		 * @return 该角度下可观测到的未知体素数
		 *
		 * 通过线性插值计算任意角度的信息增益
		 * 将角度归一化到[0, 2π)范围
		 */
		int getUnknownVoxels(double angle){
			if (int(yawNumVoxels.size()) == 0){
				return 0;
			}
			// 确保角度在 [0, 2PI) 范围内
			while (angle < 0){
				angle += 2 * M_PI;
			}

			while (angle >= 2 * M_PI){
				angle -= 2 * M_PI;
			}

			double startYaw, endYaw;
			std::vector<double> yaws;
			for (auto key : yawNumVoxels){
				yaws.push_back(key.first);
			}
			std::sort(yaws.begin(), yaws.end());

			double yawDiff = yaws[1] - yaws[0];
			bool findAngle = false;
			// 查找angle所在的区间
			for (int i=0; i<int(yaws.size())-1; ++i){
				if (angle >= yaws[i] and angle <= yaws[i+1]){
					startYaw = yaws[i];
					endYaw = yaws[i+1];
					findAngle = true;
					break;
				}
			}

			// 如果在最后一个角度和0之间
			if (not findAngle){
				startYaw = yaws.back();
				endYaw = 0.0;
			}

			// 线性插值计算
			int angleNumVoxels = yawNumVoxels[startYaw] + (angle - startYaw) * double(yawNumVoxels[endYaw] - yawNumVoxels[startYaw])/yawDiff;
			return angleNumVoxels;
		}

		/**
		 * @brief 获取信息增益最大的偏航角
		 * @return 最佳偏航角（弧度）
		 */
		double getBestYaw(){
			int maxNum = 0;
			double maxYaw = 0;
			for (auto key : yawNumVoxels){
				if (key.second > maxNum){
					maxNum = key.second;
					maxYaw = key.first;
				}
			}
			return maxYaw;
		}

		/**
		 * @brief 获取最佳偏航角对应的未知体素数
		 * @return 最大信息增益值
		 */
		int getBestYawVoxel(){
			if (int(yawNumVoxels.size()) == 0){
				return 0;
			}

			double bestYaw = this->getBestYaw();
			return this->yawNumVoxels[bestYaw];
		}
	};

	struct CompareNode{
		bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2){
			return n1->f > n2->f;
		}
	};

	struct GainCompareNode{
		bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2){
			return n1->numVoxels < n2->numVoxels;
		}
	};


	class KDTree{
	private:
		int size_;
		std::shared_ptr<Node> root_;
		double leastDistNN_ = std::numeric_limits<double>::infinity(); // temporarily save minimum distance for nearest neighbor search
		std::vector<std::shared_ptr<Node>> notTarget_;
		std::unordered_set<std::shared_ptr<Node>> notTargetTemp_;
		std::unordered_set<std::shared_ptr<Node>> notTargetPerm_;

	public:
		KDTree();
		void clear();
		std::shared_ptr<Node> getRoot();
		int getSize();
		void insert(std::shared_ptr<Node> n);
		std::shared_ptr<Node> nearestNeighbor(std::shared_ptr<Node> n, 
			                                  std::shared_ptr<Node> rootNode=NULL,
			                                  std::shared_ptr<Node> bestNode=NULL,
			                                  int depth=0);
		std::vector<std::shared_ptr<Node>> kNearestNeighbor(std::shared_ptr<Node> n, int num);
		void remove(std::shared_ptr<Node> n);
	};
}

#endif