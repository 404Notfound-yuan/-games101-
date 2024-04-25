#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for(int i = 0; i < num_nodes; i++){
            masses.push_back(new Mass(start + (end - start) * ((double)i / ((double)num_nodes - 1.0)), node_mass, false));
        }
        // Comment-in this part when you implement the constructor

        for(int i=0;i<num_nodes-1;i++){
            springs.push_back(new Spring(masses[i], masses[i+1], k));
        }

        //每一个点都固定
        for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {    // TODO（第二部分）：使用胡克定律计算节点上的力

            // 计算弹簧的当前长度
            auto len = (s->m1->position - s->m2->position).norm();

            // 计算质点1上的弹簧力，并添加到质点1的总力中
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / len * (len - s->rest_length);

            // 计算质点2上的弹簧力，并添加到质点2的总力中
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / len * (len - s->rest_length); 
        }
        float kd=0.01;
        for (auto &m : masses)
        {
            // 如果质点不固定
            if (!m->pinned)
            {
                // TODO（第二部分）：添加重力造成的力，然后计算新的速度和位置

                // 计算质点受到的加速度，包括重力加速度和由弹簧力产生的加速度
                auto a = m->forces / m->mass + gravity-kd*m->velocity/m->mass;
                
                // 根据质点的加速度更新速度，并考虑时间步长
                m->velocity += a * delta_t; 
                
                // 根据质点的速度更新位置，并考虑时间步长
                m->position += m->velocity * delta_t;
                
                // TODO（第二部分）：添加全局阻尼
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO（第三部分）：使用显式Verlet方法模拟绳子的一个时间步（求解约束）
            auto len = (s->m1->position - s->m2->position).norm();
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / len * (len - s->rest_length);
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / len * (len - s->rest_length);
        }

        for (auto &m : masses)
        {   
            float kd=0.00005;
            // 如果质点不固定
            if (!m->pinned)
            {
                // 临时保存当前位置
                Vector2D temp_position = m->position;
                // 计算质点受到的加速度，包括重力加速度和由弹簧力产生的加速度
                auto a = m->forces / m->mass + gravity;

                // 加入阻尼
    

                // TODO（第三部分）：设置绳子质点的新位置
                m->position = temp_position +(1-kd) *(temp_position - m->last_position) + a * delta_t * delta_t;
                // 更新上一个时间步的位置
                m->last_position = temp_position;
            }
            // 重置每个质点上的所有力
            m->forces = Vector2D(0, 0);
        }
    }

}
