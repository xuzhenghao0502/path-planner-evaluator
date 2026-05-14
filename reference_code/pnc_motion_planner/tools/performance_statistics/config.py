from pathlib import Path
from typing import Final, Dict, Any

PROJECT_DIR: Final[Path] = Path(__file__).parent

# --------------------------------------------------------------------------
# 第1步：将所有模块的配置信息，存放在一个主字典中
# --------------------------------------------------------------------------
MODULE_CONFIGS: Final[Dict[str, Dict[str, Any]]] = {
    "SPATIOTEMPORAL": {
        "main_keyword": "[SpatiotemporalOptimizer]",
        "loop_keywords": [
            "[populateInputData]",
            "[BaseDataPreprocess]",
            "[DecisionResultPreprocess]",
            "[DecisionObjectsParser]",
            "[LateralBoundParser]",
            "[LongitudinalBoundParser]",
            "[SpatiotemporalOptimizerLoader]",
            "[SpatiotemporalOptimizer]",
        ],
    },
    "REDUNDANT": {
        "main_keyword": "[SpeedPlanning]",
        "loop_keywords": [
            "[RedundantDataPreparation]",
            "[RedundantLateralPreprocess]",
            "[RedundantLateralBoundParser]",
            "[RedundantLateralOptimization]",
            "[RedundantLateralUnconstrained]",
            "[RedundantLateralProtection]",
            "[RedundantLateralCollisionPostProcess]",
            "[LateralPlanning]",
            "[SpeedPlanning]",
        ],
    },
    "REALTIMELATERAL": {
        "main_keyword": "[RealTimeVisualization]",
        "loop_keywords": [
            "[RealTimePreprocess]",
            "[RealTimeBoundParser]",
            "[RealTimePathOptimizer]",
            "[RealTimeCollisionPostProcess]",
            "[RealTimeVisualization]",
        ],
    },
    # 添加更多模块
    # "NEW_MODULE": {
    #     "main_keyword": "...",
    #     "loop_keywords": [...]
    # }
}

# --------------------------------------------------------------------------
# 第2步：设置一个“选择器”变量，用来指定当前要分析哪个模块
# 只需要修改下面这一行，就能切换不同的统计任务！
# --------------------------------------------------------------------------
ACTIVE_MODULE: Final[str] = "SPATIOTEMPORAL"  # 或者 "REDUNDANT", "REALTIMELATERAL"

# --------------------------------------------------------------------------
# 第3步：根据选择器，自动导出当前活动的配置
# --------------------------------------------------------------------------
try:
    CURRENT_CONFIG = MODULE_CONFIGS[ACTIVE_MODULE]
    MAIN_KEYWORD: Final[str] = CURRENT_CONFIG["main_keyword"]
    LOOP_KEYWORDS: Final[list[str]] = CURRENT_CONFIG["loop_keywords"]
except KeyError:
    raise KeyError(f"错误：活动的模块 '{ACTIVE_MODULE}' 在 MODULE_CONFIGS 中未定义。")
