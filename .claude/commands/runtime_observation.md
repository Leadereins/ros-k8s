# runtime_observation

对运行中的 **ROS2 + Docker + Kubernetes** 闭环系统进行系统观测与运行态分析，输出可审计的运行报告，判断系统是否满足稳定运行与规模扩展条件。

本命令面向 Claude Code 的 vibe coding 工作流：强调"只读观测、证据化输出、异常定位闭环、可执行优化建议"。

---

## 目标（Goals）

1. 运行态可见性：从 ROS 层、容器层、K8s 层、DDS 通信与长期运行风险五个维度建立观测证据。
2. 异常可定位：对每个异常给出"证据 → 可能原因 → 最小验证命令 → 修复/优化建议"。
3. 可扩展性评估：给出"是否适合规模扩展"的结论与前置条件。
4. 报告落盘：写入 `thoughts/shared/runtime/YYYY-MM-DD_runtime_report.md`。

---

## 硬性约束（Non-negotiable）

1. **只观测不改动**：默认只允许只读命令（get/describe/logs/top/exec 只读查询）。禁止 `apply/edit/patch/delete/rollout restart` 等写操作。
2. **必须引用证据**：每个结论必须附命令与关键输出摘要；必要时给出截断规则。
3. **不得猜测**：无法确认写"证据不足"，并给出补证命令。
4. **时间维度必须显式**：至少给出一个"当前快照（Snapshot）"与一个"短时趋势（Trend）"采样（例如 2–3 次间隔采样）。
5. **优先阻断项**：出现 CrashLoopBackOff、OOMKilled、探针持续失败、DDS 明显不可用等，应优先定位并在报告中标记为阻断扩展。

---

## 输入（Inputs）

用户可提供（任意其一即可运行；越多越精确）：

* 目标 namespace（默认：全局扫描，优先工作负载相关 namespace）
* 关键工作负载列表（Deployment/StatefulSet/Pod 名称）
* 期望 ROS_DOMAIN_ID / RMW_IMPLEMENTATION（用于一致性对照）
* 观测时长偏好（默认：10–15 分钟窗口的快照+短时趋势）

---

## 输出（Outputs）

必须写入：

* `thoughts/shared/runtime/YYYY-MM-DD_runtime_report.md`

完成后聊天输出（必须且仅一句）：

* `Runtime observation completed.`

---

## 执行流程（模块化步骤）

### Step 0：锁定观测对象与时间窗口

1. 确认 kube context、目标 namespace、关键工作负载（若未提供则自动发现）。
2. 记录观测时间窗口：开始时间、结束时间、采样间隔（例如每 2 分钟一次，采样 3 次）。

建议命令：

```bash
kubectl config current-context
kubectl get ns
kubectl get deploy,sts,ds -A -o wide
kubectl get pods -A -o wide
```

输出要求：

* 在报告中写"观测范围"：context、namespace、工作负载、Pod 列表。

---

### Step 1：K8s 层快照（稳定性与探针）

目标：建立运行拓扑与稳定性证据。

必做：

* Pod Ready/重启/所在 Node
* readinessProbe/livenessProbe 状态
* 事件（events）中是否存在探针失败、调度失败、OOM、拉镜像问题

建议命令：

```bash
kubectl get pods -A -o wide
kubectl get events -A --sort-by=.metadata.creationTimestamp | tail -n 120

# 关键 Pod 深入：
kubectl describe pod <pod> -n <ns>

# 探针字段快速检查：
kubectl get pod <pod> -n <ns> -o yaml | rg -n 'readinessProbe|livenessProbe|startupProbe'
```

输出要求：

* "运行拓扑图（ASCII）"：至少包含 Node → Pod → Container（含关键角色）。
* "稳定性摘要"：Running/Ready 比例、异常 Pod 清单、重启计数。

---

### Step 2：容器层观测（进程/重启/资源趋势）

目标：确认容器内关键进程存在、重启原因、以及短时资源趋势。

必做：

* 重启计数与最近退出原因
* CPU/Memory 当前快照
* 短时趋势采样（至少 2–3 次）
* 进程列表（只读 exec）

建议命令：

```bash
# 重启与退出原因：
kubectl get pod <pod> -n <ns> -o jsonpath='{.status.containerStatuses[*].name}{"\t"}{.status.containerStatuses[*].restartCount}{"\t"}{.status.containerStatuses[*].lastState.terminated.reason}{"\n"}'

# 资源快照与趋势（重复执行 2–3 次，间隔 1–3 分钟）：
kubectl top pods -A
kubectl top nodes

# 进程检查：
kubectl exec -n <ns> <pod> -c <container> -- ps aux
```

输出要求：

* "资源趋势表"：采样时间｜Pod｜CPU｜Memory｜RestartCount。
* 异常判定：CPU 长时间饱和、内存持续增长、频繁重启、进程缺失。

---

### Step 3：ROS 层观测（节点与 topic 活性）

目标：确认 ROS2 图（graph）是否符合预期：节点数量、topic 活性、关键 topic 是否可读。

前置条件：容器内需具备 `ros2` CLI；若不具备，必须说明并使用替代策略（例如通过应用日志或 sidecar 工具）。

建议命令（在关键 Pod 内执行）：

```bash
kubectl exec -n <ns> <pod> -c <container> -- ros2 node list
kubectl exec -n <ns> <pod> -c <container> -- ros2 topic list
kubectl exec -n <ns> <pod> -c <container> -- ros2 topic info <topic>

# 活性验证（一次性）：
kubectl exec -n <ns> <pod> -c <container> -- ros2 topic echo <topic> --once
```

输出要求：

* "ROS 拓扑摘要"：节点数、topic 数、关键 topic 是否能 echo。
* 若 echo 无输出：必须区分是"无发布者"还是"发现失败/通信失败/ QoS 不匹配"。

---

### Step 4：DDS 通信观测（发现/多播/QoS）

目标：判断 DDS 发现机制是否受限、多播是否被阻断、QoS 是否匹配导致通信失败。

必查项：

* `ROS_DOMAIN_ID` 一致性
* `RMW_IMPLEMENTATION` 一致性
* 是否使用 hostNetwork / 计划中的替代（discovery server/unicast 配置）

建议命令：

```bash
# Pod spec 中 env 与网络模式：
kubectl get pod <pod> -n <ns> -o yaml | rg -n 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS|FASTRTPS|DDS|hostNetwork|dnsPolicy'

# 实际环境变量（只读 exec）：
kubectl exec -n <ns> <pod> -c <container> -- printenv | rg 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS|FASTRTPS|DDS'
```

可选（若容器内有工具/权限）：

* 使用 `ros2 topic echo` 跨 Pod 对照（在两个 Pod 分别执行 node/topic list，观察是否一致）

输出要求：

* "DDS 配置一致性表"：Pod｜ROS_DOMAIN_ID｜RMW_IMPLEMENTATION｜hostNetwork｜PodIP/Node。
* 若怀疑多播阻断：必须给出证据链（例如节点/话题在单 Pod 内可见但跨 Pod 不可见），并提出验证方案与替代策略。

---

### Step 5：长期运行风险评估（内存泄漏/日志暴涨/线程泄露）

目标：在有限时间窗口内给出"长期风险迹象"判断，并明确需要补充的观测。

必做：

* 内存趋势（短时采样）
* 日志吞吐（同一 Pod 的 logs 采样长度变化，或日志速率迹象）
* 线程/进程数量粗判（ps 输出中线程数/进程数线索）

建议命令：

```bash
# 日志采样（重复两次观察增长/错误模式）：
kubectl logs <pod> -n <ns> -c <container> --since=2m | tail -n 200

# 进程/线程粗判：
kubectl exec -n <ns> <pod> -c <container> -- ps -eLf | wc -l
```

输出要求：

* 给出"风险迹象"与"证据不足项"：

  * 若看见内存持续增长：标记疑似泄漏/缓存增长，并建议延长观测与 profiling 手段。
  * 若日志暴涨：标记成本/存储风险，并建议限流/日志级别策略。
  * 若线程数异常：标记线程泄露风险，并建议进一步采样。

---

### Step 6：异常点汇总与优化建议（必须可执行）

目标：把所有异常点集中列出，并对每个异常提供"最小修复/优化建议"。

异常点输出格式（必须）：

* 异常 ID
* 维度（ROS/Container/K8s/DDS/Long-run）
* 证据（命令 + 关键输出摘要）
* 可能原因（限定在证据支持范围内）
* 最小验证命令（补证用）
* 优化建议（命令级或配置字段级；本命令不执行写操作）
* 扩展影响（是否阻断规模扩展）

---

### Step 7：规模扩展适配性结论

必须给出结论之一：

* 适合规模扩展
* 不适合规模扩展
* 有条件适合规模扩展

结论必须基于以下门槛（在报告中逐条对照）：

* Pod 稳定（无 CrashLoopBackOff/OOMKilled/持续探针失败）
* ROS 图可观测且关键通信可验证（topic/service/action 至少一类闭环）
* DDS 配置一致且跨 Pod/跨 Node 通信无明显阻断（或已采用替代机制并验证）
* 资源使用无明显异常趋势
* 日志/线程/内存无明显长期风险迹象（或已明确需要补充观测）

---

## 报告模板（必须严格按此结构）

写入：`thoughts/shared/runtime/YYYY-MM-DD_runtime_report.md`

1. `# Runtime Observation Report (ROS2 + K8s)`
2. `## 观测日期与时间窗口`
3. `## 观测范围（context / namespace / workloads）`
4. `## 当前运行拓扑图（ASCII）`
5. `## 观测数据与证据`

   * `### K8s 层（Pod 稳定性与探针）`
   * `### 容器层（进程/重启/资源趋势）`
   * `### ROS 层（节点与 topic 活性）`
   * `### DDS 通信（发现/多播/QoS）`
   * `### 长期运行风险（内存/日志/线程）`
6. `## 异常点（Findings）`
7. `## 优化建议（可执行）`
8. `## 是否适合规模扩展（结论与门槛对照）`
9. `## 开放问题（需要补证的命令与预期结果）`

---

## 完成条件

* 已写入 `thoughts/shared/runtime/YYYY-MM-DD_runtime_report.md`。
* 覆盖五个观测维度（ROS/Container/K8s/DDS/Long-run），且每项都有命令与结果。
* 给出异常点与可执行优化建议。
* 给出"是否适合规模扩展"的结论与门槛对照。
* 聊天输出仅一句：`Runtime observation completed.`
