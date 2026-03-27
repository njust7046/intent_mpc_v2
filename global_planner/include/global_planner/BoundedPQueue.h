/**
 * @file BoundedPQueue.h
 * @brief 有界优先队列实现
 * @author Keith Schwarz (htiek@cs.stanford.edu)
 *
 * 有界优先队列是一种特殊的优先队列,具有固定的最大容量
 * 当队列满时插入新元素,会自动删除优先级最高(值最大)的元素
 *
 * 主要特性:
 * - 固定容量限制
 * - 自动淘汰最低优先级元素
 * - 基于multimap实现,保证O(log n)的插入和删除
 * - 适用于K近邻搜索等场景
 *
 * 使用示例:
 * @code
 * BoundedPQueue<int> bpq(15);  // 最多存储15个元素
 * bpq.enqueue(42, 2.71);       // 插入元素,优先级为2.71
 * int val = bpq.dequeueMin();  // 取出最小优先级的元素
 * @endcode
 */

#ifndef BOUNDED_PQUEUE_INCLUDED
#define BOUNDED_PQUEUE_INCLUDED

#include <map>
#include <algorithm>
#include <limits>

namespace KDTree{
/**
 * @class BoundedPQueue
 * @brief 有界优先队列模板类
 * @tparam T 存储的元素类型
 *
 * 维护固定数量的最小优先级元素
 * 当队列满时,新插入的元素如果优先级小于当前最大优先级,则替换最大优先级元素
 */
template <typename T>
class BoundedPQueue {
public:
    /**
     * @brief 构造函数
     * @param maxSize 队列的最大容量
     *
     * 创建一个空的有界优先队列,最多可存储maxSize个元素
     */
    explicit BoundedPQueue(std::size_t maxSize);

    /**
     * @brief 插入元素到队列
     * @param value 要插入的元素
     * @param priority 元素的优先级(越小越优先)
     *
     * 如果队列已满且新元素优先级大于队列中最大优先级,则不插入
     * 如果队列已满且新元素优先级小于队列中最大优先级,则插入并删除最大优先级元素
     */
    void enqueue(const T& value, double priority);

    /**
     * @brief 取出并删除最小优先级的元素
     * @return 最小优先级的元素
     *
     * 前提条件: 队列非空
     */
    T dequeueMin();

    /**
     * @brief 返回队列中的元素数量
     * @return 当前元素数量
     */
    std::size_t size() const;

    /**
     * @brief 判断队列是否为空
     * @return 是否为空
     */
    bool empty() const;

    /**
     * @brief 返回队列的最大容量
     * @return 最大容量
     */
    std::size_t maxSize() const;

    /**
     * @brief 返回队列中的最小优先级
     * @return 最小优先级值,如果队列为空则返回infinity
     *
     * 这是下次dequeueMin()将返回的元素的优先级
     */
    double best()  const;

    /**
     * @brief 返回队列中的最大优先级
     * @return 最大优先级值,如果队列为空则返回infinity
     *
     * 如果插入元素的优先级大于此值,该元素将被自动拒绝
     */
    double worst() const;

private:
    // 使用multimap实现,键为优先级,值为元素
    // multimap自动按键排序,支持重复键
    std::multimap<double, T> elems;  ///< 存储元素的multimap
    std::size_t maximumSize;         ///< 最大容量
};

// ========== BoundedPQueue类实现 ==========

/**
 * @brief 构造函数实现
 */
template <typename T>
BoundedPQueue<T>::BoundedPQueue(std::size_t maxSize) {
    maximumSize = maxSize;
}

/**
 * @brief 插入元素实现
 *
 * 先将元素插入multimap,然后检查大小
 * 如果超过最大容量,删除最大优先级(最后一个)元素
 */
template <typename T>
void BoundedPQueue<T>::enqueue(const T& value, double priority) {
    // 将元素添加到集合中
    elems.insert(std::make_pair(priority, value));

    // 如果元素过多,删除最后一个(优先级最高的)
    if (size() > maxSize()) {
        typename std::multimap<double, T>::iterator last = elems.end();
        --last; // 现在指向最高优先级元素
        elems.erase(last);
    }
}

/**
 * @brief 取出最小优先级元素实现
 *
 * 复制第一个元素(优先级最小)并删除它
 */
template <typename T>
T BoundedPQueue<T>::dequeueMin() {
    // 复制最佳值
    T result = elems.begin()->second;

    // 从map中删除它
    elems.erase(elems.begin());

    return result;
}

/**
 * @brief size()和empty()直接调用底层map的方法
 */
template <typename T>
std::size_t BoundedPQueue<T>::size() const {
    return elems.size();
}

template <typename T>
bool BoundedPQueue<T>::empty() const {
    return elems.empty();
}

/**
 * @brief 返回最大容量
 */
template <typename T>
std::size_t BoundedPQueue<T>::maxSize() const {
    return maximumSize;
}

/**
 * @brief best()和worst()函数检查队列是否为空
 * 如果为空返回infinity,否则返回相应的优先级
 */
template <typename T>
double BoundedPQueue<T>::best() const {
    return empty()? std::numeric_limits<double>::infinity() : elems.begin()->first;
}

template <typename T>
double BoundedPQueue<T>::worst() const {
    return empty()? std::numeric_limits<double>::infinity() : elems.rbegin()->first;
}
}
#endif // BOUNDED_PQUEUE_INCLUDED
