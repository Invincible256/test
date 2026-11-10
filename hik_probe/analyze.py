#!/usr/bin/env python3
"""analyze.py —— 分析 hik_probe 的输出（只用 Python 标准库）

用法：
  python3 analyze.py <运行目录>             分析一次运行，结果写入 <运行目录>/analysis.txt
  python3 analyze.py <目录A> <目录B>        额外对比两次运行的节点表（例如虚拟相机 vs 真机），
                                            结果写入 <目录B>/compare_nodes.txt
"""
import bisect
import csv
import glob
import math
import os
import statistics as st
import sys

OUT = []
CARD = []   # 结论卡：适合在屏幕上拍照的精简结果


def C(s):
    CARD.append(s)


def P(s=""):
    print(s)
    OUT.append(s)


def load(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def ok(r):
    return r.upper() in ("0X00000000", "0")


def pct(v, p):
    if not v:
        return float("nan")
    v = sorted(v)
    k = (len(v) - 1) * p
    i = int(k)
    return v[i] if i + 1 >= len(v) else v[i] * (1 - (k - i)) + v[i + 1] * (k - i)


def fit(xs, ys):
    """最小二乘 y = a + b*x，返回 (a, b, 残差列表)"""
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = my - b * mx
    return a, b, [y - (a + b * x) for x, y in zip(xs, ys)]


def node_value(nodes, name):
    for r in nodes:
        if r["node"] == name:
            return r
    return None


def analyze_camera(d, tag):
    P(f"\n==================== 相机 {tag} ====================")
    nodes = load(f"{d}/{tag}_nodes_before.csv")
    frames = load(f"{d}/{tag}_frames.csv")
    lstatic = load(f"{d}/{tag}_latch_static.csv")
    lrun = load(f"{d}/{tag}_latch_run.csv")
    status = load(f"{d}/{tag}_status.csv")
    res = {"tag": tag}

    inc = node_value(nodes, "DeviceTimestampIncrement")
    ts_node = node_value(nodes, "DeviceTimestamp")
    P(f"节点：DeviceTimestamp 读取 {ts_node['read_ret'] if ts_node else '无记录'}，"
      f"DeviceTimestampIncrement = {inc['value'] if inc and ok(inc['read_ret']) else '读取失败/不存在'}")

    # ---- U2：不 Latch 时重复读是否变化 ----
    for mode, desc in (("nolatch", "不 Latch 直接读"), ("invalidate", "InvalidateNodes 后读")):
        vals = [int(r["dev_ts"]) for r in lstatic if r["mode"] == mode and ok(r["ret_read"])]
        if vals:
            P(f"[U2] {desc}：{len(vals)} 次，不同取值 {len(set(vals))} 个（=1 说明读到的是缓存/锁存值；>1 说明是实时值）")
            C(f"{tag} U2 {mode}: {len(vals)}次 {len(set(vals))}个不同值")
        else:
            P(f"[U2] {desc}：没有成功读取")

    # ---- U1：单位 ----
    L = [r for r in lstatic if r["mode"] == "latch" and ok(r["ret_cmd"]) and ok(r["ret_read"])]
    unit = None
    if len(L) >= 2:
        xs = [(int(r["t0_mono_ns"]) + int(r["t1_mono_ns"])) / 2 for r in L]
        ys = [int(r["dev_ts"]) for r in L]
        f = fit(xs, ys)
        if all(y == 0 for y in ys):
            P("[U1] Latch 后读到的设备时间戳全部为 0 → 时间戳无效（虚拟相机常见），对时功能只能在真机上验证")
            C(f"{tag} U1 单位: 时间戳全为0(无效)")
        elif f:
            slope = f[1]
            P(f"[U1] 设备时间 / 主机时间 斜率 = {slope:.9f}")
            C(f"{tag} U1 斜率={slope:.6f} Increment={inc['value'] if inc and ok(inc['read_ret']) else '无'}")
            if abs(slope - 1) < 0.01:
                unit = "ns"
                P("     → 设备时间戳单位是 ns（与主机时钟同速）")
            elif inc and ok(inc["read_ret"]) and float(inc["value"]) > 0 and abs(slope * float(inc["value"]) - 1) < 0.01:
                unit = "tick"
                P(f"     → 单位是 tick，1 tick = {inc['value']} ns")
            else:
                P("     → 无法解释的斜率，需要人工检查原始数据")
    else:
        P("[U1] 没有足够的成功 Latch 采样")

    # ---- U12：往返时间 ----
    for name, rows in (("静态", L), ("取流中", [r for r in lrun if ok(r["ret_cmd"]) and ok(r["ret_read"])])):
        rtt = [int(r["rtt_ns"]) / 1e3 for r in rows]
        if rtt:
            P(f"[U12] 对时往返时间（{name}，{len(rtt)} 次）us：最小 {min(rtt):.1f}  中位 {pct(rtt, .5):.1f}  "
              f"p90 {pct(rtt, .9):.1f}  p99 {pct(rtt, .99):.1f}  最大 {max(rtt):.1f}")
    nrun = len(lrun)
    nok = sum(1 for r in lrun if ok(r["ret_cmd"]) and ok(r["ret_read"]))
    P(f"[U2] 取流中执行 Latch：成功 {nok} / {nrun}" + ("（取流中可以对时）" if nrun and nok == nrun else ""))
    rr = [int(r["rtt_ns"]) / 1e3 for r in lrun if ok(r["ret_cmd"]) and ok(r["ret_read"])]
    C(f"{tag} U2 取流中Latch成功 {nok}/{nrun}; U12 rtt us 最小{min(rr) if rr else float('nan'):.0f} 中位{pct(rr,.5):.0f} p99{pct(rr,.99):.0f}")

    # ---- 时钟模型：host = a + b * dev（只用往返时间最短的 20% 样本）----
    model = None
    R = [r for r in lrun if ok(r["ret_cmd"]) and ok(r["ret_read"]) and int(r["dev_ts"]) != 0]
    if len(R) >= 10:
        thr = pct([int(r["rtt_ns"]) for r in R], 0.2)
        S = [r for r in R if int(r["rtt_ns"]) <= thr]
        xs = [int(r["dev_ts"]) for r in S]
        ys = [(int(r["t0_mono_ns"]) + int(r["t1_mono_ns"])) / 2 for r in S]
        f = fit(xs, ys)
        if f:
            a, b, resid = f
            model = (a, b)
            ppm = (b - 1) * 1e6 if unit == "ns" else float("nan")
            P(f"[时钟模型] 样本 {len(S)}（rtt ≤ {thr/1e3:.1f} us）；斜率 b = {b:.9f}"
              + (f"（两时钟快慢差 {ppm:+.1f} ppm）" if unit == "ns" else ""))
            P(f"          残差 us：标准差 {st.pstdev(resid)/1e3:.1f}  最大 {max(abs(x) for x in resid)/1e3:.1f}")
            C(f"{tag} 时钟模型 b={b:.9f} 残差std={st.pstdev(resid)/1e3:.1f}us")
            half = len(S) // 2
            if half >= 5:
                f1 = fit(xs[:half], ys[:half])
                f2 = fit(xs[half:], ys[half:])
                if f1 and f2:
                    mid = xs[half]
                    jump = (f2[0] + f2[1] * mid) - (f1[0] + f1[1] * mid)
                    P(f"          前半段与后半段模型在中点相差 {jump/1e3:.1f} us（越小说明线性模型越稳定，见 U13）")
    else:
        P("[时钟模型] 取流中没有足够的有效对时样本，无法换算帧时间")

    # ---- 帧分析 ----
    if not frames:
        P("[帧] 没有帧数据")
        return res
    fn = [int(r["frame_num"]) for r in frames]
    gaps = sum(1 for i in range(1, len(fn)) if fn[i] != fn[i - 1] + 1)
    recv = [int(r["recv_mono_ns"]) for r in frames]
    dev = [int(r["dev_ts"]) for r in frames]
    dur = (recv[-1] - recv[0]) / 1e9 if len(recv) > 1 else 0
    P(f"[帧] {len(frames)} 帧，{dur:.1f} s，按接收时刻 {((len(frames)-1)/dur) if dur else float('nan'):.3f} fps；帧号不连续 {gaps} 处；"
      f"本帧丢包 >0 的帧 {sum(1 for r in frames if int(r['lost_packet'])>0)} 个")
    C(f"{tag} 帧: {len(frames)}帧 {((len(frames)-1)/dur) if dur else float('nan'):.2f}fps 帧号跳变{gaps} 丢包帧{sum(1 for r in frames if int(r['lost_packet'])>0)} "
      f"格式{sorted(set(r['pixel_type'] for r in frames))} 尺寸{sorted(set((r['width'], r['height']) for r in frames))}")
    P(f"     像素格式 {sorted(set(r['pixel_type'] for r in frames))}，尺寸 {sorted(set((r['width'], r['height']) for r in frames))}，"
      f"帧长度 {sorted(set(r['frame_len'] for r in frames))}")
    ddev = [dev[i] - dev[i - 1] for i in range(1, len(dev))]
    if all(x == 0 for x in dev):
        P("[U3] 帧设备时间戳全部为 0 → 无效，只能在真机上验证")
        C(f"{tag} U3 帧时间戳全为0(无效)")
    else:
        P(f"[U3] 相邻帧设备时间戳差：中位 {pct(ddev,.5):.0f}  最小 {min(ddev)}  最大 {max(ddev)}（60 Hz 时按 ns 应约为 16666667）")
        if model:
            a, b = model
            t_skip = recv[0] + 1e9  # 跳过开始取流后的第 1 秒，避开启动瞬态
            lat = [(rv - (a + b * dv)) / 1e6 for rv, dv in zip(recv, dev) if rv >= t_skip] or \
                  [(rv - (a + b * dv)) / 1e6 for rv, dv in zip(recv, dev)]
            P(f"[U3] 接收时刻 − 换算后的帧时间（ms）：中位 {pct(lat,.5):.3f}  p1 {pct(lat,.01):.3f}  p99 {pct(lat,.99):.3f}  "
              f"标准差 {st.pstdev(lat):.3f}")
            C(f"{tag} U3 帧时间差中位{pct(ddev,.5):.0f}; 接收-帧时间 ms 中位{pct(lat,.5):.3f} std{st.pstdev(lat):.3f}")
            P("     → 若为正且稳定（标准差很小），说明帧时间戳与 DeviceTimestamp 是同一时钟；"
              "中位数约等于“时间戳对应时刻到主机收到”的延迟（与 U4 有关）")
            res["times"] = [a + b * dv for rv, dv in zip(recv, dev) if rv >= t_skip]
    res["recv"] = [rv for rv in recv if rv >= recv[0] + 1e9]

    # ---- U5：nHostTimeStamp ----
    host = [int(r["host_ts"]) for r in frames]
    dh = [host[i] - host[i - 1] for i in range(1, len(host))]
    dm = [recv[i] - recv[i - 1] for i in range(1, len(recv))]
    if dh and pct(dm, .5) > 0 and pct(dh, .5) != 0:
        ratio = pct(dh, .5) / pct(dm, .5)
        scale = None
        for s, name in ((1e-6, "ms"), (1e-3, "us"), (1.0, "ns")):
            if abs(ratio / s - 1) < 0.2:
                scale, unit_h = s, name
        if scale:
            P(f"[U5] nHostTimeStamp 单位：{unit_h}")
            hn = [h / scale for h in host]
            best = []
            for col, name in (("recv_mono_ns", "CLOCK_MONOTONIC"), ("recv_raw_ns", "CLOCK_MONOTONIC_RAW"), ("recv_real_ns", "CLOCK_REALTIME")):
                off = [(int(r[col]) - h) / 1e6 for r, h in zip(frames, hn)]
                P(f"     与 {name:<20} 的差（ms）：中位 {pct(off,.5):14.3f}  标准差 {st.pstdev(off):.3f}")
                best.append((abs(pct(off, .5)), name, pct(off, .5)))
            bm = min(best)
            C(f"{tag} U5 nHostTimeStamp 单位{unit_h} 最接近{bm[1]}(中位差{bm[2]:.3f}ms)")
            P("     → 中位差接近 0 且标准差很小的那个时钟，就是 nHostTimeStamp 使用的时钟")
        else:
            P(f"[U5] nHostTimeStamp 增量与主机时钟增量之比 {ratio:.6g}，不是常见单位，需要人工检查")
    else:
        P("[U5] nHostTimeStamp 无变化或无数据")

    # ---- 状态 ----
    if status:
        thr = [int(s["link_throughput_kb"]) for s in status if int(s["link_throughput_kb"]) >= 0]
        tmp = [float(s["temperature"]) for s in status if s["temperature"] not in ("nan", "-nan")]
        vi = [int(s["valid_images"]) for s in status]
        last = status[-1]
        P(f"[状态] 链路带宽 KB/s 中位 {pct(thr,.5) if thr else '无'}；温度 °C {min(tmp) if tmp else '无'}–{max(tmp) if tmp else ''}；"
          f"SDK 缓存积压最大 {max(vi) if vi else '无'}；USB 统计（最后一次）收到 {last['usb_recv_frames']} 帧，错误 {last['usb_err_frames']} 帧")
    return res


def pairing(results, fps):
    if len(results) < 2:
        return
    A, B = results[0], results[1]
    T = 1000.0 / fps
    P(f"\n==================== 左右配对（{A['tag']} ↔ {B['tag']}）====================")
    for key, desc in (("times", "用换算后的设备时间（方案采用的方式）"), ("recv", "用主机接收时刻（对照）")):
        ta, tb = A.get(key), B.get(key)
        if not ta or not tb:
            P(f"[{desc}] 数据不足，跳过")
            continue
        tb_sorted = sorted(tb)
        dts = []
        for t in ta:
            i = bisect.bisect_left(tb_sorted, t)
            cand = [tb_sorted[j] for j in (i - 1, i) if 0 <= j < len(tb_sorted)]
            dts.append(min(abs(t - c) for c in cand) / 1e6)
        over = sum(1 for x in dts if x > T / 2 + 0.5)
        C(f"配对[{key}] 时间差ms 平均{st.mean(dts):.2f} 中位{pct(dts,.5):.2f} p99{pct(dts,.99):.2f} 最大{max(dts):.2f} 超半周期{over}对")
        P(f"[{desc}] 最近帧时间差（ms）：平均 {st.mean(dts):.3f}  中位 {pct(dts,.5):.3f}  p99 {pct(dts,.99):.3f}  最大 {max(dts):.3f}；"
          f"超过半个帧周期（{T/2:.2f} ms）的 {over} 对")
    P(f"（已跳过每台相机开始取流后的第 1 秒）\n理论：两台相机各自自由运行时，最近帧时间差应在 0 到 {T/2:.2f} ms 之间。用设备时间计算的结果才代表真实的拍摄时间差。")


def compare(d1, d2):
    global OUT
    OUT = []
    P(f"节点对比：A = {d1}\n          B = {d2}")
    f1 = sorted(glob.glob(f"{d1}/*_nodes_before.csv"))
    f2 = sorted(glob.glob(f"{d2}/*_nodes_before.csv"))
    if not f1 or not f2:
        P("缺少节点表")
        return
    n1 = {r["node"]: r for r in load(f1[0])}
    n2 = {r["node"]: r for r in load(f2[0])}
    P(f"使用 A：{os.path.basename(f1[0])}，B：{os.path.basename(f2[0])}\n")
    P(f"{'节点':<32}{'A 状态':<26}{'B 状态':<26}差异")
    avail_diff, type_diff, range_diff = [], [], []

    def state(r):
        if r is None:
            return "无记录"
        if r["expected_type"] == "C":
            return "命令 " + (r["access_mode"] or "不存在")
        return ("可读 " + r["access_mode"]) if ok(r["read_ret"]) else ("读失败 " + r["read_ret"])

    for name in [r for r in n1] + [r for r in n2 if r not in n1]:
        a, b = n1.get(name), n2.get(name)
        sa, sb = state(a), state(b)
        diff = []
        if sa != sb:
            diff.append("可用性不同")
            avail_diff.append(f"{name}({sa}→{sb})")
        if a and b and ok(a["read_ret"]) and ok(b["read_ret"]):
            if a["read_type"] != b["read_type"]:
                diff.append("类型不同")
                type_diff.append(name)
            if a["range"] != b["range"]:
                diff.append("范围/选项不同")
                range_diff.append(name)
            if a["value"] != b["value"]:
                diff.append(f"值 {a['value'][:20]} → {b['value'][:20]}")
        if diff:
            P(f"{name:<32}{sa:<26}{sb:<26}{'；'.join(diff)}")
    C(f"节点对比 A={os.path.basename(d1)} B={os.path.basename(d2)}")
    C("  可用性不同: " + (", ".join(avail_diff) if avail_diff else "无"))
    C("  类型不同: " + (", ".join(type_diff) if type_diff else "无"))
    C("  范围/选项不同: " + (", ".join(range_diff) if range_diff else "无"))
    with open(f"{d2}/compare_nodes.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(OUT) + "\n")
    P(f"\n已写入 {d2}/compare_nodes.txt（节点范围/选项的完整内容见各自的 *_nodes_before.csv）")


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    d = sys.argv[1].rstrip("/")
    tags = sorted(os.path.basename(p)[: -len("_frames.csv")] for p in glob.glob(f"{d}/*_frames.csv"))
    fps = 60.0
    env = os.path.join(d, "env.txt")
    if os.path.exists(env):
        for line in open(env, encoding="utf-8"):
            if line.startswith("options:"):
                for tok in line.split():
                    if tok.startswith("fps="):
                        fps = float(tok[4:])
    P(f"运行目录：{d}（目标帧率 {fps}）")
    cfg = load(f"{d}/config.csv")
    if cfg:
        P("\n[配置] U8 / 取流中写入测试 / 失败的设置：")
        for r in cfg:
            if r["phase"] in ("U8", "lock_test", "U14") or not ok(r["ret"]):
                P(f"  {r['cam']:<10}{r['phase']:<10}{r['node']:<30}{r['action']:<14}ret={r['ret']}  {r['readback']}")
                C(f"{r['cam']} {r['phase']} {r['node']} {r['action']} ret={r['ret'][-4:]} {r['readback'][:40]}")
    results = [analyze_camera(d, t) for t in tags]
    pairing(results, fps)
    with open(f"{d}/analysis.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(OUT) + "\n")
    print(f"\n已写入 {d}/analysis.txt")
    if len(sys.argv) == 3:
        print()
        compare(d, sys.argv[2].rstrip("/"))
    card = [f"==== 结论卡 {os.path.basename(d)} ===="] + CARD
    with open(f"{d}/brief.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(card) + "\n")
    print("\n" + "\n".join(card))
    print(f"（以上结论卡已写入 {d}/brief.txt，可直接拍照）")


if __name__ == "__main__":
    main()
