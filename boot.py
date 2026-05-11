"""MicroPython 启动文件。

当前只保留轻量启动配置，避免某个外设异常导致系统无法进入 main。
"""

try:
    import gc

    gc.collect()
    print("[BOOT] smart helmet boot ok")
except Exception as exc:
    print("[BOOT] ignored error:", exc)

