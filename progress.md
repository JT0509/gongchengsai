# 进度快照（2026-10-25 收工）

## 状态
三块料流程已实现、编译通过、**实车跑通：物料都送对了靶位**。存在小问题，待用户描述具体现象。

## 代码现状
- 相位：1 TO_CENTER → 11 DECIDE → 2 TURN_NAV → 3 TO_POINT → 6 CLAMP_READ →（7 BACK_TO_CENTER | 9 TO_TARGET）→（8）→ 10 PLACE → 12 BACK_FROM_TARGET → 11 → … → 13 → 14 → 4
- 第一块固定取 **C**（正前方，不转向）；之后 C→E→A
- 判投：绿→A 空、红→C 空、蓝→E 空、白→B 空、黑→D 空；被挡暂存 B/D（B 优先）
- 内联清场：暂存块目标线一空就取回投掉
- 账本：`sg_heading`(uint16) `sg_taken[3]` `sg_stage[2]` `sg_claw` `sg_picked`；阈值宏在 nav.h
- 编译：Code=13148，0 error，仅 main.c 原有 6 条未引用警告

## 已避开的坑
- 白/黑暂存取回必须忽略自己占的槽（`Nav_Blocked_For`），否则自锁
- 暂存到 B/D 时不能再走 CLAMP_READ，直接 PLACE
- 角度必须 uint16（270° 装不进 uint8）

## 未做
- `NAV_TARGET_FINE_MS = 0`：实测料冲出靶心 31mm，应取负 ≈ -800ms，**待标定**
- 无全局 240s / 静停 10s 看门狗
- 无抓线确认（转偏时不扫线）
- 黑靶/白靶单独补偿未测

## 下次先做
1. 用户描述"小问题"的具体现象
2. 量落点 → 定 `NAV_TARGET_FINE_MS`
3. 按需补看门狗 / 抓线确认
