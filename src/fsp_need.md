我现在需要实现上下楼梯的落脚点规划，需求：
1. 自定义一个楼梯的形状（要求尽可能符合实际感知输入（比如中心点加多边形顶点））。
2. 对楼梯按照机器人脚掌进行侵蚀。
3. 按双圆模型确定可达区域，并确定这个区域，加入到落脚点规划下一步的时候（即此时落脚点落脚区域是确定的，不存在多个可达区域（比如每一步上一个台阶））。
4. 在进行下一步规划的时候不需要考虑z，仍然考虑x，y，yaw，等落脚点规划完后根据x，y判断落在哪个区域，根据此区域高度确定z，
5. 可视化（俯视图加剖面图），两个都保存为图片。
6.（可选）不跟踪参考线，只给目标点，达到目标点即可。
7. 初始猜测：区域大按步长，到了楼梯上按楼梯中心点

1. 楼梯定义不正确，要模拟正常感知到的楼梯，多个平面（x,y,z）以及多边形顶点，要有相对机器人的位姿及距离关系2. 首先对每一个平面进行侵蚀（即对每一个台阶平面进行侵蚀）以作为后续规划使用，而排序根据离机器人位置的远近排序） 3. 初始猜测也要修改，在台阶上就按每个台阶中心点，在大平面就按步长猜测，4. 有了楼梯台阶平面的排序，和初始猜测，则规划时的每一步区域也就确定在哪一个台阶上了，后面进行约束优化就可以。

1. 楼梯模型定义
1.1 数据结构
定义每个台阶的结构，包含：

台阶编号 id

顶面多边形（逆时针点列，二维平面）

台阶高度 z（底面高度 + 台阶厚度的一半？实际中我们关心落脚点的z坐标，通常取顶面高度）

中心点 center（用于初始猜测）

cpp
struct Step {
    int id;
    double z;                      // 顶面高度
    Eigen::Vector2d center;         // 中心点 (x, y)
    std::vector<Eigen::Vector2d> polygon; // 顶面多边形，顺时针或逆时针
};
1.2 楼梯生成函数
根据用户自定义参数生成台阶列表。常见楼梯参数：起始点 start（第一个台阶前沿中点），台阶宽度 width（沿x方向深度），台阶长度 length（沿y方向宽度），台阶高度 rise，台阶数量 num_steps。假设楼梯沿x方向延伸。

cpp
std::vector<Step> generateStairs(const Eigen::Vector2d& start, double width, double length, double rise, int num_steps) {
    std::vector<Step> stairs;
    for (int i = 0; i < num_steps; ++i) {
        Step s;
        s.id = i;
        s.z = (i + 1) * rise; // 假设地面为0，第一台阶高度rise
        double x_start = start.x() + i * width;
        double x_end = x_start + width;
        double y_half = length / 2.0;
        // 多边形顶点（逆时针）
        s.polygon = {
            {x_start, -y_half},
            {x_end,   -y_half},
            {x_end,    y_half},
            {x_start,  y_half}
        };
        s.center = { (x_start + x_end)/2.0, 0.0 };
        stairs.push_back(s);
    }
    return stairs;
}
如果楼梯有转弯或更复杂形状，可通过中心点加多边形顶点方式自定义。

2. 脚掌侵蚀
机器人脚掌通常简化为矩形或椭圆，需要从台阶多边形中减去脚掌投影，得到安全区域。

2.1 脚掌模型
假设脚掌为矩形，长 foot_length，宽 foot_width。侵蚀时考虑脚掌的包络圆或直接对多边形向内偏移（Minkowski差）。简单做法：将台阶多边形向内收缩半个脚掌尺寸（考虑朝向？但落脚点朝向可变，因此安全区域应允许脚掌以任意朝向放置而不超出台阶边缘）。最保守的做法是考虑脚掌外接圆半径 r_foot = sqrt((foot_length/2)^2 + (foot_width/2)^2)，然后对多边形向内偏移 r_foot。但更精确的可使用矩形在不同朝向下的包络，实现较复杂。推荐用圆形近似简化。

2.2 侵蚀算法
采用多边形向内偏移（缓冲）算法。这里可以用计算几何库如CGAL，但为了轻量，可手动实现：对每条边向内平移距离 d，然后求交点。d 可取 r_foot + safety_margin。

cpp
Step erodeStep(const Step& step, double erosion_radius) {
    Step eroded = step;
    // 对 step.polygon 进行向内偏移，得到 eroded.polygon
    // 可使用 Clipper2 或简单算法（如线段平移+角点圆弧），这里略
    return eroded;
}
如果不想实现复杂多边形运算，也可将每个台阶的可行区域简化为矩形：原始矩形向内缩 erosion_radius 得到新矩形，简单快速。

3. 确定落脚点与台阶的对应关系
在规划之前，我们需要知道每一步应该落在哪个台阶上。这取决于目标位置和楼梯布局。假设目标点 goal 位于某个台阶上，且机器人从平地开始，需要依次上台阶。

3.1 步数确定
根据目标点的 x 坐标和台阶深度，可以估算需要多少步到达目标台阶。同时要考虑机器人每步的最大前进距离（由运动学约束决定）。可以综合现有代码中的步数估计方法，但需要将楼梯高度变化转化为步数限制。

3.2 分配台阶索引
设步数为 N，那么第 i 步（i=0为第一步）应落在台阶 i 上（假设第一步上第一阶）。如果平地起步，第一步可能落在第一阶，第二步第二阶，以此类推。如果目标台阶低于步数，可以重复最后台阶。为简化，我们假设每一步上一个台阶，直到目标台阶。

cpp
int target_step_id = ...; // 根据 goal.x 确定
N = target_step_id + 1;   // 例如目标在第三阶，需要三步
3.3 为每个落脚点绑定台阶
在优化中，我们需要为每个变量 P(:,i) 添加约束，使其必须落在对应台阶 step_i 的侵蚀多边形内。

4. 修改优化问题
4.1 移除路径跟踪代价（可选）
根据需求6，如果不跟踪参考线，只需达到目标点，可以将 lambda_path_tracking 设为0或直接删除相关项。但目标点约束仍然需要。

4.2 添加多边形区域约束
对于每个落脚点 P(:,i)（x,y），约束其位于对应台阶的侵蚀多边形内。多边形约束可以用线性不等式表示（凸多边形）。假设侵蚀多边形是凸的（台阶矩形侵蚀后仍是矩形），我们可以将约束表示为：

text
A_i * [x; y] <= b_i
其中 A_i 和 b_i 由多边形各边法向量和常数组成。CasADi支持线性不等式约束，可以直接添加。

cpp
for (size_t i = 0; i < N; ++i) {
    const auto& poly = stairs[i].eroded_polygon; // 假设每个台阶对应一个侵蚀多边形
    // 将多边形转换为线性不等式
    for (size_t j = 0; j < poly.size(); ++j) {
        Eigen::Vector2d p1 = poly[j];
        Eigen::Vector2d p2 = poly[(j+1)%poly.size()];
        Eigen::Vector2d edge = p2 - p1;
        Eigen::Vector2d normal(edge.y(), -edge.x()); // 指向内侧的法向量（需要根据多边形方向调整）
        normal.normalize();
        double c = normal.dot(p1);
        // 约束：normal.dot([x;y]) <= c
        opti.subject_to( normal(0)*P(0,i) + normal(1)*P(1,i) <= c );
    }
}
注意：需要确保多边形顶点顺序一致（逆时针），法向量指向内部。

4.3 目标点约束
如果目标点在一个台阶上，我们可以约束最后一步落脚点位于目标台阶的区域内，并且可能希望最后两步的中间点接近目标点（类似现有代码中的硬约束）。但需求6只要求达到目标点，可以简单约束最后一步落在目标位置（目标台阶区域内一点）。更灵活的是让优化自己决定最后一步落在目标台阶何处，只要最终质心位置到达目标。

4.4 其他约束保持不变
双圆运动学约束（已存在）

相邻步转角约束

均匀性代价（可保留）

终点朝向约束（如果需要）

5. 初始猜测生成
5.1 平地部分
如果楼梯之前有平地行走，可以根据步长简单插值。

5.2 楼梯部分
对于楼梯上的步数，直接使用对应台阶的中心点作为初始猜测，并设置朝向为楼梯前进方向（如沿x轴方向）。

cpp
for (int i = 0; i < N; ++i) {
    int step_idx = i; // 假设从台阶0开始
    const auto& step = stairs[step_idx];
    opti.set_initial(P(0,i), step.center.x());
    opti.set_initial(P(1,i), step.center.y());
    opti.set_initial(P(2,i), 0.0); // 假设朝向沿x轴
}
如果平地部分步数多，需根据实际位置调整。

6. 可视化
6.1 俯视图
使用 matplotlibcpp 绘制：

楼梯每个台阶的轮廓（灰色填充或虚线）

侵蚀后的安全区域（可半透明填充）

规划的脚印（箭头）

目标点

cpp
// 绘制每个台阶多边形
for (const auto& step : stairs) {
    std::vector<double> poly_x, poly_y;
    for (const auto& pt : step.polygon) {
        poly_x.push_back(pt.x());
        poly_y.push_back(pt.y());
    }
    // 闭合多边形
    poly_x.push_back(poly_x[0]);
    poly_y.push_back(poly_y[0]);
    plt::plot(poly_x, poly_y, "k-");
}
// 类似绘制侵蚀区域（可用不同颜色填充）
6.2 剖面图
选取 y=0 剖面，显示台阶高度和落脚点的z值。由于落脚点z是根据x,y查找到对应台阶的高度确定的，我们可以绘制二维图：x为水平距离，z为高度，并用散点标出每个落脚点（x坐标，台阶高度）。

cpp
std::vector<double> stair_x, stair_z;
for (const auto& step : stairs) {
    stair_x.push_back(step.polygon[0].x()); // 前沿
    stair_z.push_back(step.z);
    stair_x.push_back(step.polygon[1].x()); // 后沿
    stair_z.push_back(step.z);
}
plt::plot(stair_x, stair_z, "k-");

// 规划落脚点
std::vector<double> foot_x, foot_z;
for (size_t i = 0; i < res_px.size(); ++i) {
    foot_x.push_back(res_px[i]);
    // 根据 res_px[i] 查找所在台阶高度
    double z = getStepHeight(stairs, res_px[i], res_py[i]);
    foot_z.push_back(z);
}
plt::scatter(foot_x, foot_z, ...);
保存图片：plt::save("profile.png")。

7. 其他注意事项
双圆运动学约束：该约束基于支撑脚位置和朝向，与高度无关，可直接复用。

ALIP 动力学：现有代码中 ALIP 部分未实际使用（注释掉了），若需要启用，需注意高度变化对动力学的影响（H_com变化）。但需求中暂不考虑z在规划中，所以可保持原样或暂时忽略。

台阶索引分配：如果机器人可能不在楼梯起点，需要根据当前支撑脚位置动态确定第一个目标台阶。

多边形约束的凸性：侵蚀后的多边形可能非凸（如台阶有凹槽），但一般楼梯台阶侵蚀后仍为凸多边形。若需处理非凸区域，可分解为多个凸多边形，分别添加约束。

求解性能：增加多边形约束会增加线性不等式数量，但一般规模可控。

代码修改示例（关键部分）
cpp
// 1. 生成楼梯
std::vector<Step> stairs = generateStairs(/*...*/);
double erosion = 0.1; // 侵蚀半径
for (auto& step : stairs) {
    step.eroded_polygon = erodeStep(step, erosion);
}

// 2. 确定步数及对应台阶
int target_step_id = findStepIndexByX(goal_x);
int N = target_step_id + 1; // 简单情况

// 3. 在优化中添加多边形约束
for (int i = 0; i < N; ++i) {
    const auto& poly = stairs[i].eroded_polygon; // 第i步落在第i阶
    for (size_t j = 0; j < poly.size(); ++j) {
        Eigen::Vector2d p1 = poly[j];
        Eigen::Vector2d p2 = poly[(j+1)%poly.size()];
        Eigen::Vector2d edge = p2 - p1;
        Eigen::Vector2d normal(edge.y(), -edge.x()); // 假设多边形逆时针
        normal.normalize();
        double c = normal.dot(p1);
        opti.subject_to( normal(0)*P(0,i) + normal(1)*P(1,i) <= c );
    }
}

// 4. 移除路径跟踪cost（可选）
// 将 lambda_path_tracking 设为0 或直接注释掉相关项

// 5. 初始猜测
for (int i = 0; i < N; ++i) {
    opti.set_initial(P(0,i), stairs[i].center.x());
    opti.set_initial(P(1,i), stairs[i].center.y());
    opti.set_initial(P(2,i), 0.0);
}
