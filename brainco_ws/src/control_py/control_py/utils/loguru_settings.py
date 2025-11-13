import datetime
import os
import sys
import logging
from loguru import logger

__all__ = ["setup_loguru"]


# 添加 InterceptHandler() 类  兼容logging
class __InterceptHandler(logging.Handler):
    def emit(self, record):
        # ✓ corresponding Loguru level if it exists
        try:
            level = logger.level(record.levelname).name
        except ValueError:
            level = record.levelno

        # Find caller from where originated the logged message
        frame, depth = logging.currentframe(), 2
        while frame.f_code.co_filename == logging.__file__:
            frame = frame.f_back
            depth += 1

        logger.opt(depth=depth, exception=record.exc_info).log(level, record.getMessage())


def setup_loguru(log_folder_path, is_developer_mode=False, is_compatible_logging=False, show_on_terminal=False):
    """
    日志配置初始化
    Args:
        is_compatible_logging: 是否兼容 logging
        log_folder_path: 日志文件存放文件夹路径
        is_developer_mode: 开启堆栈变量跟踪, 方便调试代码, 生产模式下禁用(可能会泄露敏感信息)
        show_on_terminal: 日志是否显示在终端

    Returns:

    """
    if is_compatible_logging:
        logging.basicConfig(handlers=[__InterceptHandler()], level=0)  # 兼容logging 重新向logging的输出到loguru
    logger.remove()  # 移除已添加的 handler 防止重复记录

    log_path = os.path.join(log_folder_path, datetime.datetime.now().strftime("%Y-%m-%d"))
    os.makedirs(log_folder_path, exist_ok=True)
    os.makedirs(log_path, exist_ok=True)

    retention = "15 days"  # 保存15天内的日志文件
    level = "INFO"  # 存储日志的最低级别
    rotation = "100 MB"  # 日志文件存储大小 超过此大小会自动切分文件
    compression = "zip"  # 日志压缩格式
    encoding = "UTF-8"
    format_st = "{time:YYYY-MM-DD HH:mm:ss.SSS} | {level}   | {file}:{line} - {message}"  # 日志文本记录格式
    enqueue = False  # loguru默认是多线程安全的, 如果需要多进程安全请将此参数置为True

    backtrace = is_developer_mode
    diagnose = is_developer_mode

    if show_on_terminal:
        logger.configure(handlers=[{"sink": sys.stderr, "level": 'INFO'}])   # 添加输出到终端的handler

    # 添加handler

    # 1. 所有的等级的日志都会记录到文件
    logger.add(sink=os.path.join(log_path, "system.log"),
               retention=retention,
               level=level,
               rotation=rotation,
               compression=compression,
               format=format_st,
               encoding=encoding,
               backtrace=backtrace,
               diagnose=diagnose,
               enqueue=enqueue)

    # 2. 只记录INFO等级的日志
    logger.add(sink=os.path.join(log_path, "system_info.log"),
               retention=retention,
               level="INFO",
               rotation=rotation,
               compression=compression,
               format=format_st,
               encoding=encoding,
               backtrace=backtrace,
               diagnose=diagnose,
               enqueue=enqueue,
               filter=lambda x: 'INFO' in str(x['level']).upper())

    # 2. 只记录ERROR等级的日志
    logger.add(sink=os.path.join(log_path, "system_error.log"),
               retention=retention,
               level="ERROR",
               rotation=rotation,
               compression=compression,
               format=format_st,
               encoding=encoding,
               backtrace=backtrace,
               diagnose=diagnose,
               enqueue=enqueue,
               filter=lambda x: 'ERROR' in str(x['level']).upper())

    # 2. 只记录DEBUG等级的日志
    logger.add(sink=os.path.join(log_path, "system_debug.log"),
               retention=retention,
               level="DEBUG",
               rotation=rotation,
               compression=compression,
               format=format_st,
               encoding=encoding,
               backtrace=backtrace,
               diagnose=diagnose,
               enqueue=enqueue,
               filter=lambda x: 'DEBUG' in str(x['level']).upper())
    

# def log_info(msg, bold=False, end=1):
#     """
#     打印 INFO 日志，简洁显示，可选择是否加粗或取消换行
#     """

#     if bold:
#         msg = f"\033[1m{msg}\033[0m"  # 加粗
#     else:
#         msg = f"\033[22m{msg}\033[0m"  # 取消加粗（显式设置为正常粗细）

#     # 临时添加一个只输出 message 的 handler
#     # logger.remove()  # 移除默认的 handler

#     i = logger.add(sys.stderr, format="{message}", level="INFO", colorize=True)
#     logger.info(msg)
#     if end != 1:
#         sys.stdout.write(msg)   # 不会换行
#         sys.stdout.flush()      # 刷新缓冲区
#     logger.remove(i)




