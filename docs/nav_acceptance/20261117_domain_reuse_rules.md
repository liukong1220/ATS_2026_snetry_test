# 本轮历史 ROS domain 复用与清场规则

2026-11-17 用户明确答复：“允许本轮复用历史编号，，进行清除domain的规则”。这只覆盖本轮 O1/O1b/O0 工作，不改变历史运行的编号、失败记录或证据。原 0..232 已用完事实保留。

1. 仍限定 0..232，排除 5、77、91、92。同一轮不同仿真实例优先使用不同编号；每次使用新批次目录。
2. 启动前加载工作区 `log/nav_acceptance/dds/wlan_env.sh`，核验 XML 所选 WLAN 在线、`ROS_LOCALHOST_ONLY=0` 和 CycloneDDS，不修改系统/全局配置，不切回 lo。
3. 复用前同时核验该 domain 的进程环境和 RTPS UDP 端口段；任何残留均阻断启动。端口空闲本身不能证明已清场；权限不可见和其他主机的未知参与者属于限制，不能写成跨 WLAN 全局排他性已获证。
4. `scripts/nav_domain_guard.py` 对本机 domain 持有文件锁并记录执行命令、子进程和 kernel start time。只清理本轮记录且身份仍匹配的本用户进程；不按进程名全局 `pkill`，不杀未知同域进程，不删除历史日志或共享 DDS 配置。
5. 正常退出或收到信号后，先由 runner 退出导航链，再对记录的本轮残留依次 SIGINT/SIGTERM/SIGKILL，复查进程与端口。未清空则本次 guard 非零退出，下一次启动被阻断。没有 ROS 图运行时不需要通过重启系统或修改 domain ID 消除状态。
6. 单测套件可在一个受锁保护的批次串行执行；每个故障和每次 MuJoCo launch 建立独立批次。复用同一编号的后续实例仍需重新预检，不能沿用此前的 `clear=true`。

```bash
source install/setup.bash
source log/nav_acceptance/dds/wlan_env.sh
python3 -s scripts/nav_domain_guard.py check --domain 20 \
  --run-dir log/nav_acceptance/20261117_o1_o0_completion/preflight_d20 \
  --reuse-authorized
python3 -s scripts/nav_domain_guard.py run --domain 20 \
  --run-dir log/nav_acceptance/20261117_o1_o0_completion/unit_d20 \
  --reuse-authorized -- ctest --test-dir build/ats_goal_manager --output-on-failure -V
```

`run` 会把 `ROS_DOMAIN_ID` 写入子进程环境，退出后生成 `domain_guard.json`。异常中断后可用原批次目录执行 `cleanup`；它只接受该 manifest 中的 PID/start-time/domain 身份，不收编其他进程。`check` 仅检查，不启动 ROS、不杀进程。历史复用授权由操作者显式 `--reuse-authorized` 留痕，不自动推定。
