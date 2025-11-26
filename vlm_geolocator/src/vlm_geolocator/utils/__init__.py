"""工具函数"""
import yaml


def load_yaml_config(filepath: str) -> dict:
    """加载YAML配置文件"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

