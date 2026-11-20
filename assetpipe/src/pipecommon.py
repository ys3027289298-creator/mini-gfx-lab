"""assetpipe 共用异常类型。"""


class PipeError(Exception):
    """流水线内所有可预期错误的基类，消息中必须带文件名或配置键。"""


class ConfigError(PipeError):
    """plan 文件本身不合法。"""


class MissingSource(PipeError):
    """plan 引用的源文件不存在。"""


class UnsupportedFormat(PipeError):
    """源文件扩展名 / 编码格式不在支持列表内。"""


class AtlasOverflow(PipeError):
    """图集装箱失败，输入塞不下指定尺寸。"""
