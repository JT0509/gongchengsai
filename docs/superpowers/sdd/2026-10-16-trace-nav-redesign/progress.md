# SDD ledger — plan: docs/superpowers/plans/2026-10-16-trace-nav-redesign.md

## Setup Rulings
- Ruling: 项目非 git 仓库 — 无 worktree/commit/BASE，计划中的 commit 步骤以「编译验证（armcc 语法检查 + EIDE Rebuild）/ 实车验证」替代 — 成本：无 git 回滚，靠开发记录.md + 本 ledger 追踪
- Ruling: 嵌入式硬件代码无法 host 自动测试 — 仅纯函数（Grey_Classify 等）做 host 单测（D 盘 MinGW gcc）；硬件相关代码以「armcc 单文件语法检查 + 实车验证清单」替代 TDD — 成本：硬件路径缺自动回归，靠实车验证步骤覆盖

## Pre-flight（接口一致性扫描）
- Task1→Task2: F_GW_classify() 签名一致 ✓
- Task2→Task4: Trace_LineFollow(int16_t)→TraceState / Trace_CenterHit()→uint8_t 一致 ✓
- Task3→Task4: Spin_Angle / ENC_PER_DEG 一致 ✓
- Task4→Task5: Nav_TaskStart(int16_t) 一致 ✓

## Tasks
Task 0: complete (修 eide.yml + 侧方灰度 PD10→PD12, tests: armcc 编译 18 源文件 → 0 error)
Task 1: complete (GREY_classify 边缘法分类 + F/B_GW_classify, tests: host gcc 单测 RED 16fail → GREEN ALL PASS; armcc 编译 OK)
Task 2: code done (档位查表 + 丢线锯齿 + 全黑标志, armcc OK) — 实车验证待用户
Task 3: code done (Turn_Calib_Test 入口, armcc OK) — ENC_PER_DEG 待实车标定
Task 4: code done (nav 里程定位 + 数线门控 + 状态机, armcc OK) — 实车验证待用户
Task 5: code done (NAV_TEST_TARGET 宏) — 实车验证 C/E 待用户

## Rulings
- Ruling (Task 2/3 方向): 用标准结论「左转=内侧左轮慢、外侧右轮快」固化方向 —— 循迹 U_TABLE LEFT→+U(左转)/RIGHT→-U(右转)，回摆 dir 线偏左=+1；Spin_Angle 改 Diff_Drive(-v,v)+err=(start-Spin_Encoder) 使正 deg=左转，并加 guard=5000 超时保护。LINE_SPEED 8→12 / U_WEAK 3→4 / U_STRONG 5→6 / SEARCH_V 4→6 避免差速后低速侧(原 V-U=3)进电机板死区。成本：若实车方向仍反→左右轮映射反(201/204 实为右轮)需交换 V_L/V_R；若转向不停→编码器「前进=正脉冲」假设反需反向 err
- Ruling (I2C 灰度读): Grey_I2C_ReadReg 读单字节后没恢复 ACK=ENABLE → 连续读灰度第二次读到脏数据(全黑误判)，是「循迹只纠正一次就直线」+「全白 st=JUNCTION」根因。已在末尾补 I2C_AcknowledgeConfig(ENABLE)
- Ruling (编码器极性): 实测前进时 odom 递减(-4→-7000) → 编码器「前进=负脉冲」(极性反)。修复：Spin_Angle err 改回 (Spin_Encoder()-start)；Trace_CenterHit/Nav_UpdateLandmark 里程差值取绝对值。成本：若实际极性是正，则 Spin_Angle 会再次不收敛，需反向
- Ruling (灰度位序): 实车确认 bit7=车左(高位)、bit0=车右(低位)，与 F_GW_position 注释「bit0=车左」相反。Grey_Classify 位序从 (data>>i) 反转为 (data>>(7-i))，host 单测镜像 14fail→ALL PASS
