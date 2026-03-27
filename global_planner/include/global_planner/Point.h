/**
 * @file Point.h
 * @brief N维空间点类模板
 *
 * 提供N维空间中点的表示和基本运算
 * 支持迭代器访问、算术运算、距离计算等功能
 * 用于KD树和RRT算法中的空间点表示
 */
#ifndef POINT_INCLUDED
#define POINT_INCLUDED

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <Eigen/Eigen>

namespace KDTree{
    /**
     * @class Point
     * @brief N维空间点类模板
     * @tparam N 空间维度
     *
     * 表示N维空间中的一个点,支持索引访问、迭代器遍历和基本运算
     */
    template <std::size_t N>
    class Point {
    public:

        // ========== 类型定义 ==========
        typedef double* iterator;              ///< 可修改元素的迭代器类型
        typedef const double* const_iterator;  ///< 只读元素的迭代器类型

        /**
         * @brief 返回点的维度
         * @return 维度N
         */
        std::size_t size() const;

        /**
         * @brief 访问或修改指定索引的坐标值
         * @param index 坐标索引 (0到N-1)
         * @return 坐标值的引用
         */
        double& operator[](std::size_t index);

        /**
         * @brief 访问指定索引的坐标值(只读)
         * @param index 坐标索引 (0到N-1)
         * @return 坐标值
         */
        double operator[](std::size_t index) const;

        // ========== 迭代器接口 ==========
        /**
         * @brief 返回指向第一个坐标的迭代器
         */
        iterator begin();

        /**
         * @brief 返回指向最后一个坐标之后的迭代器
         */
        iterator end();

        /**
         * @brief 返回指向第一个坐标的只读迭代器
         */
        const_iterator begin() const;

        /**
         * @brief 返回指向最后一个坐标之后的只读迭代器
         */
        const_iterator end() const;


    private:
        double coords[N];  ///< 坐标数组
    };

    // ========== 全局函数声明 ==========

    /**
     * @brief 计算两点间的欧几里得距离
     * @param one 第一个点
     * @param two 第二个点
     * @return 欧几里得距离
     */
    template <std::size_t N>
    double Distance(const Point<N>& one, const Point<N>& two);

    /**
     * @brief 判断两点是否相等
     */
    template <std::size_t N>
    bool operator==(const Point<N>& one, const Point<N>& two);

    /**
     * @brief 判断两点是否不相等
     */
    template <std::size_t N>
    bool operator!=(const Point<N>& one, const Point<N>& two);
    

    template <std::size_t N>
    std::size_t Point<N>::size() const {
        return N;
    }

    template <std::size_t N>
    double& Point<N>::operator[] (std::size_t index) {
        return coords[index];
    }

    template <std::size_t N>
    double Point<N>::operator[] (std::size_t index) const {
        return coords[index];
    }

    template <std::size_t N>
    typename Point<N>::iterator Point<N>::begin() {
        return coords;
    }

    template <std::size_t N>
    typename Point<N>::const_iterator Point<N>::begin() const {
        return coords;
    }

    template <std::size_t N>
    typename Point<N>::iterator Point<N>::end() {
        return begin() + size();
    }

    template <std::size_t N>
    typename Point<N>::const_iterator Point<N>::end() const {
        return begin() + size();
    }




    //=========================================================

    template <std::size_t N>
    double Distance(const Point<N>& one, const Point<N>& two) {
        double result = 0.0;
        for (std::size_t i = 0; i < N; ++i)
            result += (one[i] - two[i]) * (one[i] - two[i]);
        result = sqrt(result);
        return result;
    }

    template <std::size_t N>
    bool operator==(const Point<N>& one, const Point<N>& two) {
        return std::equal(one.begin(), one.end(), two.begin());
    }

    template <std::size_t N>
    bool operator!=(const Point<N>& one, const Point<N>& two) {
        return !(one == two);
    }



    // ========== 算术运算符 ==========

    /**
     * @brief 点加法运算
     * @return 两点对应坐标相加的结果
     */
    template <std::size_t N>
    Point<N> operator+(const Point<N>& p1, const Point<N>& p2){
        Point<N> p_sum;
        for (size_t i=0; i<N; ++i){
            p_sum[i] = p1[i] + p2[i];
        }
        return p_sum;
    }

    /**
     * @brief 点减法运算
     * @return 两点对应坐标相减的结果
     */
    template <std::size_t N>
    Point<N> operator-(const Point<N>& p1, const Point<N>& p2){
        Point<N> p_sum;
        for (size_t i=0; i<N; ++i){
            p_sum[i] = p1[i] - p2[i];
        }
        return p_sum;
    }

    /**
     * @brief 点与标量相乘(点在左)
     * @param p1 点
     * @param r 标量
     * @return 点的每个坐标乘以标量的结果
     */
    template <std::size_t N>
    Point<N> operator*(const Point<N>& p1, double r){
        Point<N> p_result;
        for (size_t i=0; i<N; ++i){
            p_result[i] = r * p1[i];
        }
        return p_result;
    }

    /**
     * @brief 标量与点相乘(标量在左)
     * @param r 标量
     * @param p1 点
     * @return 点的每个坐标乘以标量的结果
     */
    template <std::size_t N>
    Point<N> operator*(double r, const Point<N>& p1){
        Point<N> p_result;
        for (size_t i=0; i<N; ++i){
            p_result[i] = r * p1[i];
        }
        return p_result;
    }


    // ========== 输出流运算符 ==========
    /**
     * @brief 将点输出到流(格式: (x y z))
     */
    template <std::size_t N>
    std::ostream &operator<<(std::ostream &os, Point<N> const &p){
        os << "(";
        for (size_t i=0; i<N; ++i){
            os << p[i];
            if (i != N-1){
                os << " ";
            }
        }
        os << ")";
        return os;
    }

    /**
     * @struct PointHasher
     * @brief 点的哈希函数对象
     *
     * 用于在unordered_map和unordered_set中使用Point作为键
     * 通过坐标的平方和计算哈希值
     */
    struct PointHasher{
        template <std::size_t N>
        std::size_t operator()(const Point<N>& p) const{
            double sum = 0.0;
            for (size_t i=0; i<N; ++i){
                sum += (p[i]/N) * (p[i]/N) * 1000000;
            }
            std::size_t hash = std::size_t(sum);
            return hash;
        }
    };


    // ========== 类型转换工具函数 ==========

    /**
     * @brief 将Point转换为std::vector
     * @param p 输入点
     * @return 坐标向量
     */
    template <std::size_t N>
    std::vector<double> point2Vec(const Point<N>& p){
        std::vector<double> result;
        for (size_t i=1; i<N; ++i){
            result.push_back(p[i]);
        }
        return result;
    }

    /**
     * @brief 将std::vector转换为Point
     * @param vec 坐标向量
     * @return Point对象
     */
    template <std::size_t N>
    Point<N> vec2Point(const std::vector<double>& vec){
        Point<N> result;
        for (size_t i=0; i<N; ++i){
            result[i] = vec[i];
        }
        return result;
    }

    /**
     * @brief 将Point转换为Eigen::Vector3d
     * @param p 输入点(至少3维)
     * @return Eigen向量
     */
    template <std::size_t N>
    Eigen::Vector3d point2Eig(const Point<N>& p){
        Eigen::Vector3d pE (p[0], p[1], p[2]);
        return pE;
    }

    /**
     * @brief 将Eigen::Vector3d转换为Point
     * @param pE Eigen向量
     * @return Point对象
     */
    template <std::size_t N>
    Point<N> eig2Point(const Eigen::Vector3d& pE){
        Point<N> result;
        result[0] = pE(0);
        result[1] = pE(1);
        result[2] = pE(2);
        return result;
    }

}
#endif // POINT_INCLUDED
