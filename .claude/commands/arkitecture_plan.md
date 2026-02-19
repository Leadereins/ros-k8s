# architecture_plan

基于最新 `thoughts/shared/research/` 研究报告，生成一个可执行的系统架构计划，使仓库形成可运行的 **ROS2 → Docker → Kubernetes** 闭环。

本命令面向 Claude Code 的 vibe coding 工作流：允许并行调研与拆解，但最终必须输出一份"分 Phase、可验证、避免一次性大改"的实施计划，并写入 `thoughts/shared/plans/`。

---

## 目标（Goals）

1. 以研究报告为唯一事实来源：所有计划项必须能追溯到 research 文档的证据或结论（引用 research 里的章节或文件证据）。
2. 分阶段交付：每个 Phase 独立可验证、可回滚，且阶段间依赖清晰。
3. 命令级可执行：每个 Phase 必须提供"执行命令 + 自动验证命令 + 成功判据"。
4. 控制变更半径：禁止"大手术式重构"；优先最小变更闭环。

---

## 硬性约束（Non-negotiable）

1. **不允许一次性大改**：任一 Phase 的变更必须可在 1–2 次提交范围内完成（逻辑上自洽即可，不要求你真的提交）。
2. 每个 Phase 必须独立验证：不得把验证推到最后。
3. **必须按 Phase 输出**：Phase 1–5 缺一不可。
4. 若 research 文档信息不足：必须在"开放问题"中明确列出，并给出"验证命令"。
5. 允许写入（唯一）：新增计划文档 `thoughts/shared/plans/YYYY-MM-DD_ros_k8s_architecture.md`（若目录不存在可创建）。

---

## 输入与输出（I/O）

输入：

* 最新 research 文档：`thoughts/shared/research/YYYY-MM-DD_ros_k8s_system_research.md`

  * 如果存在多个版本：选择日期最新的一份，并在计划文档开头注明选用的 research 文件名。

输出（必须）：

* 写入：`thoughts/shared/plans/YYYY-MM-DD_ros_k8s_architecture.md`

完成后聊天输出（必须且仅一句）：

* `Architecture plan completed.`

---

## 执行流程（四代理并行）

你需要并行推进四条链路，最后合并为一份计划文档。每条链路都要产出"证据/约束 → 可执行决策"。

### Agent A：plan-locator（定位事实与边界）

目标：从 research 文档中抽取与计划直接相关的事实、约束、已知风险、关键接口与运行假设。

必抽取项：

* 当前 ROS2 workspace 判定与 package 列表（build 类型、入口、launch、参数来源）
* Dockerfile/entrypoint/运行 env 现状
* K8s 资源现状（Deployment/Service/ConfigMap/Secret/hostNetwork/imagePullPolicy 等）
* 风险清单（重点：DDS、多播、ROS_DOMAIN_ID、RMW_IMPLEMENTATION、网络模式、镜像分发）

输出要求：

* 形成"计划前提条件"列表（每条引用 research 章节或证据）。

---

### Agent B：plan-designer（Phase 方案设计）

目标：把"闭环目标"拆成 Phase 1–5 的最小可行路径（MVP pipeline），并为每个 Phase 定义：

* 修改文件（明确到路径）
* 变更内容（要点级，不写代码也要能落地）
* 执行命令（build/deploy/run）
* 自动验证命令（必须是命令级）
* 成功判据（可观测、可复现）

设计原则：

* 优先实现"最小闭环"：能跑、能看到通信、能在 K8s 中稳定启动。
* 所有网络/DDS 改动必须放在 Phase 4，除非 research 明确证明 Phase 1–3 无法验证。
* 镜像命名、tag、pull 策略要在 Phase 2/3 定型，避免后期返工。

---

### Agent C：plan-validator（一致性与可验证性检查）

目标：检查 Phase 之间依赖是否自洽，验证命令是否覆盖风险点，是否存在"验证缺口"。

必检项：

* Phase 1 的 ROS2 构建与运行能否在宿主机成功验证（无容器）
* Phase 2 的容器能否本地运行并复用 Phase 1 的验证
* Phase 3 的 K8s 是否能在最小资源上启动并通过 probes
* Phase 4 的网络/DDS 调整是否有 A/B 对照验证
* Phase 5 的观测与日志是否能定位 Phase 3/4 的常见故障

输出要求：

* 产出"验证覆盖矩阵"：Phase × 验证项（命令级）。

---

### Agent D：plan-risk-owner（风险驱动的执行策略）

目标：把 research 风险清单映射到 Phase 4/5 的具体动作与判据，避免"风险写了但不落地"。

必覆盖风险：

* DDS discovery：多播限制、跨 node 通信失败、Service/action 不可用
* ROS_DOMAIN_ID 冲突与隔离
* RMW_IMPLEMENTATION 选择与一致性
* hostNetwork 是否必要（以及替代方案）
* 镜像分发：containerd/k3s 下的 import/registry 策略

输出要求：

* 风险→缓解→验证 命令链条（必须命令级）。

---

## Phase 设计要求（固定骨架，不得改名）

计划必须包含如下五个 Phase，并按顺序输出。

### Phase 1 — ROS Workspace 标准化

必须包含：

* 文件变更列表（路径级）
* 构建命令（colcon/ament）
* 验证命令（ros2 run/ros2 launch/ros2 topic echo 等）
* 自动验证命令（可脚本化）
* 成功判据（输出、topic/service/action 可观测）

### Phase 2 — 容器化

必须包含：

* Dockerfile 策略（单镜像/多阶段；理由必须与 research 对齐）
* 基础镜像版本与 pin 策略（humble/iron；是否锁定 digest）
* entrypoint/env 约定（ROS_DOMAIN_ID/RMW_IMPLEMENTATION/DDS 配置路径等）
* 镜像命名规范（repo/name:tag）
* 构建命令 + 本地运行验证命令
* 成功判据

### Phase 3 — K8s 部署

必须包含：

* Deployment YAML（或等价资源；若 research 有既存则基于其最小修改）
* 环境变量注入（env/envFrom，ConfigMap/Secret）
* 镜像拉取策略（imagePullPolicy、imagePullSecrets、离线策略）
* Service（如需要）与端口暴露策略
* 应用命令（kubectl apply / kustomize / helm）
* 自动验证命令（kubectl rollout status / get pods -o wide / logs 等）
* 成功判据（Pod Ready、基础功能可验证）

### Phase 4 — 网络与 DDS 适配

必须包含：

* ROS_DOMAIN_ID 策略（多机器人/多 namespace 的默认规则）
* RMW_IMPLEMENTATION 策略（CycloneDDS/FastDDS 选择与一致性）
* hostNetwork 是否需要：

  * 若需要：明确启用位置与副作用
  * 若不需要：给出替代（discovery server / unicast 配置 / CNI 约束）
* A/B 对照验证（改动前后验证命令对比）
* 成功判据（跨 Pod/跨 Node 通信可观测、关键接口可用）

### Phase 5 — 观测与日志策略

必须包含：

* kubectl logs / describe / events 使用路径
* readinessProbe / livenessProbe（放在 YAML 的哪个容器）
* 基础指标与故障定位路径（最少可落地一条）
* 自动验证命令（探针状态、重启次数、日志关键字）
* 成功判据（故障可定位、回归可验证）

---

## 输出文档格式（必须严格按此结构）

文件名：`thoughts/shared/plans/YYYY-MM-DD_ros_k8s_architecture.md`

文档结构：

1. `# ROS2 → Docker → K8s Architecture Plan`
2. `## 计划日期`
3. `## 参考 Research 文档`

   * 写明引用的 research 文件名
4. `## 架构目标`
5. `## 总体闭环结构图（ASCII）`
6. `## 分阶段交付策略（为什么这样拆）`
7. `## Phase 1 — ROS Workspace 标准化`

   * 目标
   * 修改文件（表格：路径｜变更类型｜说明｜风险）
   * 执行命令（代码块）
   * 自动验证命令（代码块）
   * 成功判据
8. `## Phase 2 — 容器化`

   * 同上
9. `## Phase 3 — K8s 部署`

   * 同上
10. `## Phase 4 — 网络与 DDS 适配`

* 同上（必须含 A/B 对照验证）

11. `## Phase 5 — 观测与日志策略`

* 同上

12. `## 验证覆盖矩阵（Phase × 验证项）`
13. `## 风险与回滚策略`

* 每条：风险｜触发条件｜影响｜缓解动作｜回滚动作｜验证命令

14. `## 开放问题`

* 每条：缺失信息｜为什么缺｜验证命令｜预期结果

15. `## 下一步执行入口（建议命令序列）`

* 把 Phase 1–3 的最小闭环命令串起来，便于快速跑通

---

## 可执行性与粒度要求（强制）

1. "修改文件"必须是真实路径（来自 research 或仓库现状）；若不确定，列为开放问题并提供定位命令。
2. 命令必须可复制执行；允许占位符，但必须提供如何确定占位符的命令。
3. 每个 Phase 必须独立可验证：至少包含一次 `ros2` 层验证与一次容器/K8s 层验证（与 Phase 目标匹配）。
4. 任何涉及网络/DDS 的决定都必须提供验证闭环：topic 列表、echo、service call、action send goal 任选其一以上。

---

## 完成条件

* 已在 `thoughts/shared/plans/` 写入 `YYYY-MM-DD_ros_k8s_architecture.md`。
* 文档包含 Phase 1–5 且每个 Phase 都具备：修改文件 / 执行命令 / 自动验证命令 / 成功判据。
* 聊天输出仅一句：`Architecture plan completed.`
