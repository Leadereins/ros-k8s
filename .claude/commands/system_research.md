# system_research

目标：
对当前 ROS + Docker + K8s 仓库进行系统级研究，生成结构化研究报告。

执行原则：
- 只读不改代码
- 必须引用具体文件路径
- 输出写入 thoughts/shared/research/

研究范围：

1. 代码结构分析
   - src/ 是否为标准 ROS2 workspace
   - 是否存在 colcon / ament 构建配置
   - 是否存在 entry_points

2. 容器化状态
   - docker/Dockerfile 是否存在
   - 镜像构建策略
   - 基础镜像版本（ros:humble / iron）

3. K8s 架构分析
   - k8s/ 目录是否存在
   - Deployment / Service / ConfigMap 是否完整
   - 镜像拉取策略是否合理

4. 集群环境假设
   - 本地 k3s / remote cluster ?
   - container runtime 类型
   - 镜像导入策略是否明确

5. 关键风险
   - ROS DDS 多播在 K8s 内的限制
   - ROS_DOMAIN_ID 是否显式设置
   - Pod 网络模式是否适配

输出格式：

文件名：
YYYY-MM-DD_ros_k8s_system_research.md

必须包含：

- 当前系统结构图（ASCII）
- 构建流程图
- 依赖列表
- 风险列表
- 下一步建议（必须是可执行命令级别）

完成后提示：
"System research completed."
