/**
 * @file PRMAstar.h
 * @brief PRM路线图上的A*路径搜索算法
 *
 * 在概率路线图(PRM)上使用A*算法搜索最优路径
 * 结合了PRM的全局连通性和A*的启发式搜索效率
 *
 * 主要功能:
 * - 在PRM图上执行A*搜索
 * - 使用欧几里得距离作为启发函数
 * - 支持碰撞检测和路径验证
 * - 自动清理搜索缓存数据
 */

#ifndef PRM_ASTAR_H
#define PRM_ASTAR_H

#include <queue>
#include <global_planner/PRMKDTree.h>

namespace PRM{
	/**
	 * @brief 检查节点是否在关闭列表中
	 * @param n 待检查的节点
	 * @param close 关闭列表(已访问节点集合)
	 * @return 是否在关闭列表中
	 */
	bool inClose(std::shared_ptr<Node> n, const std::unordered_set<std::shared_ptr<Node>>& close){
		std::unordered_set<std::shared_ptr<Node>>::const_iterator got = close.find(n);
		return not (got == close.end());
	}

	/**
	 * @brief 在PRM路线图上执行A*路径搜索
	 * @param roadmap PRM路线图(KD树)
	 * @param start 起始节点
	 * @param goal 目标节点
	 * @param map 占据栅格地图(用于碰撞检测)
	 * @return 从起点到终点的路径节点序列,如果无路径则返回空向量
	 *
	 * A*算法流程:
	 * 1. 初始化开放列表(优先队列)和关闭列表(已访问集合)
	 * 2. 将起点加入开放列表
	 * 3. 循环直到找到目标或开放列表为空:
	 *    - 从开放列表取出f值最小的节点
	 *    - 将其加入关闭列表
	 *    - 遍历其所有邻居节点
	 *    - 对于未访问的邻居,计算代价并更新
	 * 4. 回溯路径
	 * 5. 清理搜索缓存数据
	 *
	 * 代价函数:
	 * - g(n): 从起点到节点n的实际代价(路径长度)
	 * - h(n): 从节点n到目标的启发式代价(欧几里得距离)
	 * - f(n) = g(n) + h(n): 总代价估计
	 */
	std::vector<std::shared_ptr<Node>> AStar(const std::shared_ptr<KDTree>& roadmap,
										     const std::shared_ptr<Node>& start,
										     const std::shared_ptr<Node>& goal,
										     const std::shared_ptr<mapManager::occMap>& map){
		std::vector<std::shared_ptr<Node>> path;

		// ========== 初始化开放列表(优先队列) ==========
		// 按f值排序,f值小的优先
		std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open;
		start->g = 0.0;
		open.push(start);

		// ========== 初始化关闭列表(已访问集合) ==========
		std::unordered_set<std::shared_ptr<Node>> close;

		// ========== 记录所有参与搜索的节点(用于后续清理) ==========
		std::vector<std::shared_ptr<Node>> record;
		record.push_back(start);
		bool findPath = false;

		// ========== A*主循环 ==========
		while (ros::ok()){
			// 检查是否到达目标
			if (inClose(goal, close)){
				findPath = true;
				break;
			}

			// 检查开放列表是否为空
			if (open.size() == 0){
				findPath = false;
				cout << "[Astar]: No valid path." << endl;
				break;
			}

			// 取出f值最小的节点
			std::shared_ptr<Node> currNode = open.top();
			open.pop();

			// 移除开放列表中的重复节点
			if (inClose(currNode, close)){
				continue;
			}

			// 将当前节点加入关闭列表
			close.insert(currNode);

			// ========== 遍历所有邻居节点 ==========
			for (std::shared_ptr<Node> neighborNode : currNode->adjNodes){
				// 节点必须不在关闭列表中
				if (not inClose(neighborNode, close)){
					// 检查从当前节点到邻居节点的路径是否无碰撞
					if (map->isInflatedFreeLine(currNode->pos, neighborNode->pos)){
						// 计算新的g值(实际代价)
						double cost = currNode->g + (currNode->pos - neighborNode->pos).norm();
						// 如果找到更优路径,更新邻居节点
						if (cost < neighborNode->g){
							neighborNode->g = cost;
							// 计算f值 = g + h(启发式距离)
							neighborNode->f = cost + (neighborNode->pos - goal->pos).norm();
							open.push(neighborNode);
							neighborNode->parent = currNode;
							record.push_back(neighborNode);
						}
					}
				}
			}
		}


		// ========== 回溯路径 ==========
		std::shared_ptr<Node> ptr = goal;
		while (ptr != NULL and findPath){
			path.push_back(ptr);
			ptr = ptr->parent;
		}
		std::reverse(path.begin(), path.end());

		// ========== 清理搜索缓存数据 ==========
		// 将所有节点的g、f和parent重置为初始状态
		for (std::shared_ptr<Node> n : record){
			n->g = std::numeric_limits<double>::infinity();
			n->f = std::numeric_limits<double>::infinity();
			n->parent = NULL;
		}
		return path;
	}
}

#endif