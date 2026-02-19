# system_research

对当前仓库进行"ROS2 + Docker + Kubernetes"系统级只读研究，产出可审计、可复现的结构化研究报告，并写入 `thoughts/shared/research/`。

本命令面向 Claude Code 的 vibe coding 工作流：允许并行采集信息，但最终输出必须合并为一份单文件报告，且所有关键结论必须有"文件路径 + 行号范围"的证据引用。

---

## 目标

1. 系统级理解：代码如何构建、如何容器化、如何在 K8s 中运行、依赖与隐含假设是什么。
2. 证据可审计：所有关键结论、工程化接口索引、关键代码片段都必须带 `path:Lx-Ly` 引用。
3. 输出可执行：给出命令级别的下一步建议，用于复现构建、镜像制作、部署、排障。
4. 输出落盘：写入 `thoughts/shared/research/YYYY-MM-DD_ros_k8s_system_research.md`，写完后聊天只输出一句：`System research completed.`

---

## 硬性约束

1. 只读：不得修改任何现有代码、配置与文档（包括 YAML、Dockerfile、CMakeLists、package.xml、脚本等）。
2. 唯一允许写入：新增研究报告 Markdown 文件（路径见"输出"）。
3. 所有"关键结论"必须引用证据：`相对路径:行号起-行号止`。
4. 不允许猜测：未找到证据则写"未发现/证据不足"，并提供下一步验证命令（命令级别）。
5. 所有路径必须真实存在于仓库内（相对仓库根目录），不允许虚构目录与文件名。

---

## 输入与输出

输入：

* 当前仓库工作目录（默认在仓库根目录运行）。

输出（必须）：

* 生成并写入：`thoughts/shared/research/YYYY-MM-DD_ros_k8s_system_research.md`

写入后聊天输出（必须且仅一句）：

* `System research completed.`

---

## 执行流程（四代理并行）

你需要并行推进四条链路，最后合并为一份报告。每条链路都要产出"证据清单（路径+行号）+ 结论/洞察"。

### Agent A：codebase-locator（找相关文件）

目标：定位与 ROS2 / Docker / K8s 相关的关键文件，记录路径与关键行号范围。

最低覆盖：

* ROS2 workspace：`src/`、`package.xml`、`CMakeLists.txt`、`setup.py/setup.cfg`、`colcon` 相关
* 运行入口：节点入口 `main()`、launch 文件、参数 YAML、脚本入口
* 容器化：Dockerfile、entrypoint 脚本、compose/CI 相关
* K8s：Deployment/StatefulSet/DaemonSet、Service、ConfigMap、Secret、Kustomize/Helm/Argo 等

建议只读命令（按需）：

```bash
pwd
git rev-parse --show-toplevel
ls
tree -L 4

find . -maxdepth 6 -type f \( \
  -name "package.xml" -o -name "CMakeLists.txt" -o -name "setup.py" -o -name "setup.cfg" \
  -o -iname "README*" -o -path "./docs/*" -o -path "./design/*" -o -path "./thoughts/*" \
  -o -name "Dockerfile*" -o -path "./docker/*" -o -name "docker-compose*.yml" -o -name "docker-compose*.yaml" \
  -o -path "./k8s/*" -o -path "./manifests/*" -o -path "./charts/*" -o -name "*.yaml" -o -name "*.yml" \
\) | sed 's|^\./||'
```

行号证据采集方法（强制）：

```bash
nl -ba path/to/file | sed -n 'START,ENDp'
```

---

### Agent B：codebase-analyzer（实现解剖 + 关键接口抽取）

目标：回答"系统怎么工作"，并抽取关键类/函数/接口，形成可追溯证据链（路径+行号）。

硬性输出（必须同时产出）：

* B1) 工程化接口索引（Interface Index，带证据）：Topic/Service/Action/Parameter/TF
* B2) 关键代码片段（最短闭环证据，带文件与行号）
* B3) 三大系统分析：ROS2 结构 / Docker 容器化 / K8s 架构（每条结论都由 B1/B2 支撑）

#### B1：工程化接口索引（Interface Index，带证据）

索引必须可用于后续生成通信图；因此每条记录必须能定位 endpoint 与证据行号。缺失字段写"未显式"，不得猜测。

##### 1) Topic Index（必选）

表头固定如下（不得删列）：

| InterfaceType | MsgType | Name | Direction | Role | Endpoint | Source | QoS | Namespace/Remap | Notes |
| ------------- | ------- | ---- | --------- | ---- | -------- | ------ | --- | --------------- | ----- |

字段规则：

* InterfaceType：固定 `Topic`
* MsgType：全限定类型，如 `sensor_msgs/msg/LaserScan`
* Name：topic 名，如 `/scan`、`cmd_vel`；若非字面量来自参数/launch，写 `<from param: xxx>` 或 `<from launch remap>`，并在 Notes 写解析链条（每一步都要证据）
* Direction：Publish / Subscribe
* Role：Producer / Consumer
* Endpoint：Node 类名或可执行名（例如 `MyNode` / `talker_node` / `ComposableNode` 名称）
* Source：`path:Lx-Ly`（必须）
* QoS：能定位则填（Reliability/Durability/History/Depth），否则写 `未显式`
* Namespace/Remap：若由 launch/remap 注入，引用 launch 的证据
* Notes：频率、frame、是否 lifecycle、关键约束等（仅能由证据推导的内容）

##### 2) Service Index（存在则必选）

表头：

| InterfaceType | SrvType | Name | Direction | Role | Endpoint | Source | Timeout/Callback | Notes |
| ------------- | ------- | ---- | --------- | ---- | -------- | ------ | ---------------- | ----- |

##### 3) Action Index（存在则必选）

表头：

| InterfaceType | ActionType | Name | Direction | Role | Endpoint | Source | Feedback/Result | Notes |
| ------------- | ---------- | ---- | --------- | ---- | -------- | ------ | --------------- | ----- |

##### 4) Parameter Index（必选）

表头：

| ParamName | Type | Default | UsedFor | Reader/Writer | Source | Notes |
| --------- | ---- | ------- | ------- | ------------- | ------ | ----- |

UsedFor 示例：topic 名/QoS/frame/network/namespace/DDS 配置路径等。
Source 必须同时覆盖 declare 与使用点（至少两个引用；若 declare/使用在同段则一个引用可接受）。

##### 5) TF / Frame Index（若存在 tf2 broadcaster/listener 则必选）

表头：

| InterfaceType | FrameFrom | FrameTo | Publisher/Consumer | Source | Frequency/Static | Notes |
| ------------- | --------- | ------- | ------------------ | ------ | ---------------- | ----- |

抽取触发条件（强制）：

* 只要出现以下调用，就必须在索引中出现一条记录：

  * C++：`create_publisher<...>`、`create_subscription<...>`、`create_service<...>`、`create_client<...>`、`rclcpp_action::create_server/client`、`tf2_ros::TransformBroadcaster` 等
  * Python：`create_publisher(...)`、`create_subscription(...)`、`create_service(...)`、`create_client(...)`、`ActionServer/ActionClient` 等
* 名称由参数/launch 注入时：Name 字段不得"猜字符串"，必须写来源占位符并给证据链。

推荐检索命令（只读，按需）：

```bash
# ROS2 C++
rg -n "create_publisher<|create_subscription<|create_service<|create_client<|rclcpp_action::create_server|rclcpp_action::create_client" src
rg -n "declare_parameter|get_parameter|get_parameter_or" src
rg -n "QoS\\(|rclcpp::QoS|KeepLast|Reliable|BestEffort|TransientLocal|Durability" src
rg -n "tf2_ros::TransformBroadcaster|TransformListener|StaticTransformBroadcaster" src

# ROS2 Python
rg -n "create_publisher\\(|create_subscription\\(|create_service\\(|create_client\\(|ActionServer\\(|ActionClient\\(" src
rg -n "declare_parameter\\(|get_parameter\\(" src

# Launch / params
find src -type f \( -name "*.launch.py" -o -name "*.yaml" -o -name "*.yml" \) | sed 's|^\./||'
rg -n "Node\\(|ComposableNode\\(|parameters=|remappings=|namespace=|name=" src
```

#### B2：关键代码片段（最短闭环证据，带文件与行号）

目的：用最少代码证明系统的关键工作流闭环，不贴整文件。

片段质量门槛（强制）：

* 每个片段建议 40–80 行；超出必须拆分并说明拆分理由。
* 每个片段必须回答至少一个闭环问题：
  a) ROS2 节点如何启动（main/init/spin）？
  b) 如何通信（topic/service/action）？
  c) 参数如何注入（declare + load + use）？
  d) 容器如何 source ROS 环境并启动（entrypoint -> exec）？
  e) K8s 如何施加 env/config/网络模式（Deployment/ConfigMap/Service 的关键字段）？

片段组织（建议）：

* ROS2：节点入口与通信接口片段
* ROS2：launch / 参数注入片段
* Docker：镜像构建关键段 + entrypoint 片段
* K8s：Deployment/Service/ConfigMap 注入关键段

引用规则（强制）：

* 每个片段必须标注 `path:Lx-Ly`，并粘贴最小必要代码。
* 若片段依赖另一个文件（例如参数 YAML 被 launch 引用），必须在 Notes 标出依赖并引用对应行号。

#### B3：三大系统分析（必须保留）

你仍需在报告中保留以下三章，且每条结论必须由 B1/B2 的证据支撑：

* `## 代码结构分析（ROS2）`

  * workspace 判定、package 列表、build 类型、入口与 launch
* `## 容器化状态（Docker）`

  * Dockerfile/镜像策略/基础镜像/entrypoint/env
* `## K8s 架构分析`

  * 资源清单、配置注入、网络模式、镜像拉取策略

---

### Agent C：thoughts-locator（搜现有文档与设计决策）

目标：查找仓库内已有文档/注释/设计记录，避免重复发明，并收集集群假设与运行约束。

最低覆盖：

* README / docs / design / ADR / 注释里是否描述：目标场景、集群类型（k3s/remote）、网络假设、镜像导入策略、DDS/ROS_DOMAIN_ID 约定等
* 若存在 `thoughts/` 或类似目录，定位历史研究/实验记录/踩坑记录

建议只读命令：

```bash
find . -maxdepth 6 -type f \( -iname "README*" -o -path "./docs/*" -o -path "./design/*" -o -path "./thoughts/*" -o -name "*.md" \) | sed 's|^\./||'
rg -n "k3s|k8s|kubernetes|docker|image load|ctr|containerd|DDS|multicast|Cyclone|FastDDS|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|domain id" .
```

---

### Agent D：thoughts-analyzer（洞察、风险、边界条件、可执行建议）

目标：从 A/B/C 的证据中提炼系统风险、隐含假设、边界条件，并形成"可执行命令级别建议"。

最低覆盖风险（必须落到配置与可执行修复项）：

* ROS DDS 多播在 K8s/CNI 下的限制：是否依赖 multicast discovery；是否需要 unicast discovery server 或 RMW/DDS 配置
* `ROS_DOMAIN_ID` 是否显式；多机器人/多 namespace/多集群冲突风险
* Pod 网络模式适配：`hostNetwork`、跨节点发现、Service 是否破坏 peer-to-peer
* 镜像分发策略：k3s/containerd 下的 `ctr images import` 或私有 registry；离线方案是否明确
* 安全与权限边界：privileged、NET_ADMIN、CAP、hostPath、device 插入等

输出必须包含：

* 风险列表：每条包含 风险描述｜证据｜触发条件｜影响｜缓解方案（命令级别）
* 开放问题列表：需要补齐的信息 + 验证命令（命令级别）

---

## 最终报告结构（必须严格按顺序输出）

生成并写入：`thoughts/shared/research/YYYY-MM-DD_ros_k8s_system_research.md`

章节顺序（标题必须一致）：

1. `# ROS + K8s System Research Report`

2. `## 研究日期`

3. `## 研究问题（用户提出的原始问题）`

4. `## 发现摘要`（3–5 条；每条必须带证据引用）

5. `## 当前系统结构图（ASCII）`（体现 workspace/package、镜像、K8s 资源、配置注入路径）

6. `## 构建与运行流程图（ASCII）`（colcon → 镜像构建 → 推送/导入 → kubectl apply → 运行入口）

7. `## 工程化接口索引（Interface Index，带证据）`

   * `### Topic Index`
   * `### Service Index`（存在则必须）
   * `### Action Index`（存在则必须）
   * `### Parameter Index`
   * `### TF / Frame Index`（存在 tf2 则必须）

8. `## 关键代码片段（最关键闭环证据，带文件与行号）`

   * `### ROS2：节点入口与通信接口片段`
   * `### ROS2：launch / 参数注入片段`
   * `### Docker：镜像构建关键段 + entrypoint 片段`
   * `### K8s：Deployment/Service/ConfigMap 注入关键段`

9. `## 相关文件清单`

   * 表格列：文件路径｜作用说明｜关键行号范围

10. `## 代码结构分析（ROS2）`

    * workspace 判定、package 列表、build 类型、入口与 launch（全部有证据）

11. `## 容器化状态（Docker）`

    * Dockerfile/镜像策略/基础镜像/entrypoint/env（全部有证据）

12. `## K8s 架构分析`

    * 资源清单、配置注入、网络模式、镜像拉取策略（全部有证据）

13. `## 集群环境假设`

    * 本地 k3s vs remote、container runtime、镜像导入路径（必须基于证据；不足则列为开放问题）

14. `## 依赖列表`

    * ROS 包依赖（package.xml）、系统依赖（Dockerfile apt/pip）、K8s 依赖（CRD/Helm）

15. `## 关键风险与边界条件`

    * 每条：风险描述｜证据｜触发条件｜影响｜缓解方案（命令级别）

16. `## 下一步建议（命令级别）`

    * 可直接复制执行；允许占位符，但必须说明如何从仓库证据确定占位符取值

17. `## 开放问题`

18. `## 参考资料`

    * 优先列仓库内文档路径；外部资料可选，但不得替代仓库证据

---

## 质量门槛（写前自检）

* 是否每个核心结论都有至少一个 `path:Lx-Ly` 证据？
* 是否接口索引能定位 MsgType、Name（或来源链）、Endpoint、Direction、Source？
* 是否关键代码片段都能回答闭环问题（启动/通信/参数/容器入口/K8s 注入）？
* "下一步建议"是否全部是命令级别，并与仓库内容一致？
* 是否只新增了报告文件，没有改任何现有文件？

---

## 执行提示（落盘与结束语）

* 若 `thoughts/shared/research/` 不存在：允许创建目录（这是输出的一部分，不算修改现有代码）。
* 报告写入完成后：聊天中只输出一句 `System research completed.`，不附加任何解释或日志。
