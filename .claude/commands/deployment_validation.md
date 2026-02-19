# deployment_validation

对当前 Kubernetes 部署状态做系统化验证，判断部署是否符合架构计划（`architecture_plan`）的预期，并给出可执行的偏差修复建议与"是否可进入生产阶段"的结论。

本命令面向 Claude Code 的 vibe coding 工作流：强调范围锁定、证据化输出、命令级复现与明确的上线门槛。

---

## 目标（Goals）

1. 全面验证当前 K8s 部署健康度：Pod 状态、日志、镜像、网络/DDS、资源使用。
2. 对照架构计划：镜像版本、imagePullPolicy、env 注入（ROS_DOMAIN_ID/RMW_IMPLEMENTATION 等）、网络模式（hostNetwork 等）是否与计划一致。
3. 形成可审计报告：每个结论都由"命令 + 关键输出片段"支撑。
4. 给出修复建议：以命令级步骤描述最小修复路径，并明确风险与回滚方式。

---

## 硬性约束（Non-negotiable）

1. **只验证，不改动系统**：只允许只读命令（get/describe/logs/top）。默认禁止 `apply/edit/patch/delete/rollout restart` 等写操作。
2. **必须对照计划**：若用户提供计划文档，验证项必须以计划为基准；若未提供，报告必须标记"缺少基准"，并将关键基准列为开放问题。
3. **必须记录命令结果**：每个验证项至少包含 1 条命令和对应的关键输出摘要；必要时附完整输出（可截断，但要说明截断规则）。
4. **不得猜测**：无法确认就写"证据不足"，并给出补证命令。
5. **验证失败即止损**：若出现 CrashLoopBackOff、ImagePullBackOff、OOMKilled、探针持续失败等阻断项，必须优先定位并在"偏差项"中标红。

---

## 输入（Inputs）

用户必须提供（至少其一）：

* 部署计划文档路径：`thoughts/shared/plans/YYYY-MM-DD_ros_k8s_architecture.md`

  * 用于对照"期望镜像/标签、env、hostNetwork、pull policy、探针策略"。

可选输入（若不提供则使用默认/自动探测）：

* kube context（如 `--context <name>`）
* namespace（若只验证某个 namespace）
* 关键工作负载名称（Deployment/StatefulSet 名称列表）

---

## 输出（Outputs）

必须写入：

* `thoughts/shared/deploy/YYYY-MM-DD_ros_k8s_validation.md`

完成后聊天输出（必须且仅一句）：

* `Deployment validation completed.`

---

## 执行流程（模块化步骤）

### Step 0：锁定验证基准（计划对照）

1. 读取架构计划文档，提取并冻结以下"期望基准"（写入报告，后续所有偏差都以此为参照）：

   * 期望工作负载清单（Deployment/StatefulSet/DaemonSet）与 namespace
   * 镜像命名与 tag 规范（以及期望的 imagePullPolicy）
   * 关键 env：`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、DDS 配置路径（如有）
   * 网络模式：是否 `hostNetwork`、`dnsPolicy`、Service 类型
   * 观测策略：readinessProbe/livenessProbe（如计划包含）

若计划缺失某项：

* 在报告"开放问题"中列出，并给出补证命令或应补充到计划的字段。

---

### Step 1：部署快照（全局与目标命名空间）

目标：建立"当前部署状态"基线。

建议命令（按需，优先目标 namespace）：

```bash
kubectl config current-context
kubectl get nodes -o wide
kubectl get ns
kubectl get deploy,sts,ds -A -o wide
kubectl get pods -A -o wide
kubectl get events -A --sort-by=.metadata.creationTimestamp | tail -n 80
```

输出要求：

* 报告中给出：

  * 工作负载清单与副本数（期望 vs 实际）
  * Pod 状态分布（Running/Pending/CrashLoopBackOff/ImagePullBackOff 等）
  * 最近事件摘要（重点：拉镜像失败、探针失败、调度失败、OOM/eviction）

---

### Step 2：Pod 状态验证（健康度与阻断项）

目标：回答"是否存在阻断项（CrashLoopBackOff 等）以及原因"。

必做：

* 对所有非 Running/Ready 的 Pod，逐个执行 describe。

建议命令：

```bash
# 对异常 Pod：
kubectl describe pod <pod> -n <ns>

# 观察重启与退出原因：
kubectl get pod <pod> -n <ns> -o jsonpath='{.status.containerStatuses[*].lastState.terminated.reason}'
kubectl get pod <pod> -n <ns> -o jsonpath='{.status.containerStatuses[*].restartCount}'
```

报告要求：

* 列出异常 Pod 清单：状态｜原因（来自 describe/events）｜影响范围（哪个工作负载）｜是否阻断上线。

阻断项判定（至少包含）：

* CrashLoopBackOff / ImagePullBackOff / ErrImagePull
* OOMKilled / Evicted / FailedScheduling
* readinessProbe/livenessProbe 持续失败

---

### Step 3：日志验证（ROS 启动与运行信号）

目标：确认容器是否按预期启动 ROS2 进程，并是否出现明确错误（参数加载失败、DDS 初始化失败、找不到 workspace、权限问题等）。

建议命令：

```bash
# 最近日志：
kubectl logs <pod> -n <ns> --tail=200

# 多容器 Pod 指定容器：
kubectl logs <pod> -n <ns> -c <container> --tail=200

# 看重启前日志：
kubectl logs <pod> -n <ns> -c <container> --previous --tail=200
```

报告要求：

* 记录"ROS 正常输出"证据：例如节点启动信息、参数加载成功、开始 spin、发布/订阅建立等（以你实际观察到的日志为准，不允许硬编码关键词）。
* 若未观察到 ROS 启动迹象：明确是 entrypoint 未执行、程序提前退出、还是日志级别/重定向导致不可见，并给出补证命令（例如检查 command/args、检查进程列表：`kubectl exec ... ps aux`）。

可选补证（只读 exec）：

```bash
kubectl exec -n <ns> <pod> -c <container> -- ps aux
kubectl exec -n <ns> <pod> -c <container> -- printenv | rg 'ROS_|RMW_|CYCLONEDDS|FASTRTPS'
```

---

### Step 4：镜像验证（版本、拉取策略、与计划一致性）

目标：确认实际运行镜像与计划一致，并评估 imagePullPolicy 是否合理（尤其在 k3s/containerd 或离线环境）。

建议命令：

```bash
# 查看实际镜像：
kubectl get pod -A -o jsonpath='{range .items[*]}{.metadata.namespace}{"\t"}{.metadata.name}{"\t"}{range .spec.containers[*]}{.name}{":"}{.image}{"\t"}{.imagePullPolicy}{"\t"}{end}{"\n"}{end}'

# 查看工作负载模板镜像（Deployment/StatefulSet）：
kubectl get deploy -n <ns> <deploy> -o yaml | rg -n 'image:|imagePullPolicy:'
```

报告要求：

* "期望镜像（来自计划） vs 实际镜像（来自集群）"对照表：工作负载｜期望 image:tag｜实际 image:tag｜pullPolicy｜偏差说明。
* 若存在 `:latest` 或未 pin tag：标记为风险，并给出修复建议（命令级）。
* 若 imagePullPolicy 与环境不匹配（例如离线环境却 Always）：标记偏差并给出最小修复路径。

---

### Step 5：网络与 DDS/ROS 配置验证

目标：验证 ROS2 在 K8s 网络中是否满足发现与通信条件，并确认关键 env 一致性。

必查项：

* `ROS_DOMAIN_ID` 是否一致（至少在同一"应通信的工作负载集合"内一致）
* `RMW_IMPLEMENTATION` 是否一致（同集合一致；混用需明确理由与证据）
* 运行网络模式：是否 hostNetwork；若不是，是否存在计划中的替代方案（discovery server / unicast 配置等）

建议命令：

```bash
# 查看 env 注入（通过 Pod spec）：
kubectl get pod <pod> -n <ns> -o yaml | rg -n 'env:|envFrom:|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS|FASTRTPS|DDS'

# 查看 hostNetwork/dnsPolicy：
kubectl get pod <pod> -n <ns> -o jsonpath='{.spec.hostNetwork}{"\t"}{.spec.dnsPolicy}{"\n"}'

# 查看 Pod IP 分布（是否跨 Node/跨网段）：
kubectl get pods -n <ns> -o wide
```

可选通信验证（仅当容器内具备 ros2 CLI，且计划要求形成闭环）：

```bash
kubectl exec -n <ns> <podA> -- ros2 node list
kubectl exec -n <ns> <podA> -- ros2 topic list
kubectl exec -n <ns> <podA> -- ros2 topic echo <topic> --once
```

报告要求：

* 给出"配置一致性表"：工作负载/Pod｜ROS_DOMAIN_ID｜RMW_IMPLEMENTATION｜hostNetwork｜PodIP/Node。
* 对发现/通信风险做结论：若无法做通信验证，必须说明阻碍（例如无 ros2 CLI、权限不足），并给出替代验证路径。

---

### Step 6：资源使用验证（CPU/Memory/重启/限制）

目标：判断是否存在异常资源使用（CPU 飙高、内存泄漏、频繁重启、OOM）。

建议命令：

```bash
kubectl top nodes
kubectl top pods -A

# 若怀疑 OOM/限制：
kubectl describe pod <pod> -n <ns> | rg -n 'Limits|Requests|OOMKilled|Reason|Exit Code'
```

报告要求：

* 标记异常 Pod：使用量异常｜是否超 limits｜是否导致重启/探针失败｜可能原因（基于证据）。

---

### Step 7：偏差项（Deviation List）与修复建议（命令级）

目标：把所有"计划 vs 现实"的差异集中列出，并给出最小修复路径。

偏差项输出格式（必须）：

* 偏差项 ID
* 类别（Pod/Log/Image/Network/Resource/Probe）
* 期望（引用计划）
* 实际（引用命令输出）
* 影响（是否阻断上线）
* 修复建议（命令级步骤；若需要改 YAML，明确改哪个文件/字段，但本命令不执行写操作）
* 回滚策略（至少一句话 + 对应命令）

---

### Step 8：上线门槛与结论（是否可进入生产阶段）

必须给出结论：`可以进入生产阶段 / 不建议进入生产阶段 / 有条件进入生产阶段`。

门槛建议（可按项目调整，但必须在报告里明确）：

* 0 个阻断项（CrashLoopBackOff、ImagePullBackOff、OOMKilled、持续探针失败）。
* 工作负载副本数达到计划预期，且 rollout 完成。
* 镜像版本与计划一致（或偏差有明确批准/解释）。
* ROS_DOMAIN_ID / RMW_IMPLEMENTATION 与网络模式符合计划，并完成至少一种通信验证或等价证据。
* 资源使用无明显异常趋势（至少当前快照合理）。

---

## 报告模板（必须严格按此结构）

写入：`thoughts/shared/deploy/YYYY-MM-DD_ros_k8s_validation.md`

1. `# ROS K8s Deployment Validation`
2. `## 验证日期`
3. `## 参考计划文档`

   * 计划路径（若无则写"未提供"）
4. `## 当前部署状态（Snapshot）`

   * 集群/namespace/工作负载摘要
5. `## 验证命令与结果`

   * `### Pod 状态`
   * `### 日志验证`
   * `### 镜像验证`
   * `### 网络与 DDS/ROS 配置验证`
   * `### 资源使用`
6. `## 偏差项（Deviation List）`
7. `## 修复建议（命令级）`
8. `## 是否可以进入生产阶段`
9. `## 开放问题（需要补证的信息与命令）`

---

## 完成条件

* 已写入 `thoughts/shared/deploy/YYYY-MM-DD_ros_k8s_validation.md`。
* 覆盖验证范围 1–5（Pod/Logs/Image/Network/Resource），且每项都有命令与结果。
* 明确列出偏差项、修复建议与上线结论。
* 聊天输出仅一句：`Deployment validation completed.`
