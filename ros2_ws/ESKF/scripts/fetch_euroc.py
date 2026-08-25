#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
#  下载 EuRoC V1_01_easy 的 IMU + 真值 CSV（只拉我们需要的文件）
#
#  背景：官方 zip（vicon_room1.zip ≈ 6GB）绝大部分是相机图片，
#  我们做 ESKF 只需要 3 个小文件（≈ 5MB）：
#    mav0/imu0/data.csv                     IMU 读数（200Hz）
#    mav0/imu0/sensor.yaml                  IMU 噪声参数
#    mav0/state_groundtruth_estimate0/data.csv  真值（100Hz）
#
#  原理：ETH 服务器支持 HTTP Range 请求。remotezip 只下载
#  zip 的中央目录 + 被请求文件的压缩字节段，不解压整个 zip。
#
#  用法：pip3 install --user remotezip
#        python3 scripts/fetch_euroc.py
# ============================================================
import os
import sys
import time
import requests
import remotezip

# ETH Research Collection 上 EuRoC 条目的 vicon_room1.zip
URL = ("https://www.research-collection.ethz.ch/server/api/core/bitstreams/"
       "02ecda9a-298f-498b-970c-b7c44334d880/content")

# 要拉的文件（在 zip 内以 .../V1_01_easy/mav0/... 开头）
SEQUENCE = "V1_01_easy"
WANT_SUFFIX = ("/imu0/data.csv",
               "/imu0/sensor.yaml",
               "/state_groundtruth_estimate0/data.csv")

OUT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "data", "euroc"))


def fetch():
    os.makedirs(OUT, exist_ok=True)
    # ETH 服务器按 User-Agent 拦爬虫：必须带浏览器 UA，否则 403。
    session = requests.Session()
    session.headers.update({
        "User-Agent": ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                       "AppleWebKit/537.36 (KHTML, like Gecko) "
                       "Chrome/120.0 Safari/537.36"),
        "Accept": "*/*",
    })
    # remotezip 内部会发若干个 Range 请求；ETH 有速率限制（约 500 次），
    # 我们在连接层做重试 + 退避。
    last_err = None
    for attempt in range(4):
        try:
            with remotezip.RemoteZip(URL, session=session) as zf:
                names = [n for n in zf.namelist()
                         if n.startswith(SEQUENCE) and n.endswith(WANT_SUFFIX)]
                if not names:
                    # 打印 zip 里 V1_01_easy 的目录结构，方便排错
                    print("!! 没匹配到目标文件。V1_01_easy 下有的条目：")
                    for n in zf.namelist():
                        if n.startswith(SEQUENCE):
                            print("   ", n)
                    sys.exit(1)
                for name in names:
                    data = zf.read(name)
                    rel = name.split("/mav0/", 1)[-1]       # imu0/data.csv, ...
                    out_path = os.path.join(OUT, rel)
                    os.makedirs(os.path.dirname(out_path), exist_ok=True)
                    with open(out_path, "wb") as f:
                        f.write(data)
                    print(f"{len(data)/1e6:7.2f} MB  {out_path}")
            return
        except Exception as e:  # noqa: BLE001 —— 网络错误 / 429 都算
            last_err = e
            print(f"第 {attempt+1} 次尝试失败: {e}")
            if attempt < 3:
                time.sleep(5 * (attempt + 1))   # 5s, 10s, 15s 退避

    print(f"!! 4 次都失败，最后一个错误: {last_err}")
    print("降级方案：注释里提到可全量下载 vicon_room1.zip 后用 zipfile 提取。")
    sys.exit(1)


if __name__ == "__main__":
    fetch()
    print("\n完成。文件已存到 data/euroc/")
