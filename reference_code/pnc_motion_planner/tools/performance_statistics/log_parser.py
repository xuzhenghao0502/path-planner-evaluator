import pickle
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class LogParser:
    """
    Class to do performance statistics of the prediction node.
    """

    log_path: str | Path

    lines: list[str] = field(default_factory=list)

    def __post_init__(self):
        self.log_path = Path(self.log_path)
        self.lines = self._read_log()

    def run(self, output_path: str | Path, main_keyword: str, keywords: list[str]):
        statistics = self._parse_loop_logs(main_keyword, keywords)

        with open(output_path, "wb") as fw:
            pickle.dump(statistics, fw)

    def _read_log(self):
        lines = []
        if self.log_path.is_dir():
            for file in self.log_path.rglob("*.log"):
                lines += self._read_file(file)
        else:
            lines = self._read_file(self.log_path)
        return lines

    def _read_file(self, file_path: Path):
        with open(file_path, "r") as f:
            lines = f.readlines()
        return [line.strip() for line in lines]

    def _parse_loop_logs(self, main_keyword: str, keywords: list[str]):
        statistics = {keyword: [] for keyword in keywords}
        single_stat = dict.fromkeys(keywords, -1.0)
        valid_cnt = 0
        for line in self.lines:
            if "ms" not in line:
                continue

            value = -1.0
            for keyword in keywords:
                if keyword not in line:
                    continue

                try:
                    line_split = line.split()
                    # 初始化时间单位的索引
                    time_index = -1
                    # 遍历所有单词，找到以 'ms' 开头的那个词 (可以匹配 'ms' 和 'ms,')
                    for i, word in enumerate(line_split):
                        if word.startswith("ms"):
                            time_index = i
                            break

                    # 如果找到了 'ms'，那么它前面的那个词就是时间数值
                    if time_index > 0:
                        value = float(line_split[time_index - 1])
                    else:
                        # 如果没找到 'ms'，就跳过这一行
                        continue
                except (ValueError, IndexError):
                    # 如果转换失败或索引越界，也跳过
                    continue
                single_stat[keyword] = value

            if value < 0.0:
                continue

            # print(single_stat)
            values = list(single_stat.values())
            if single_stat[main_keyword] > 0:
                if -1.0 not in values:
                    [
                        statistics[key].append(single_stat[key])
                        for key in single_stat.keys()
                    ]
                    valid_cnt += 1
                single_stat = dict().fromkeys(keywords, -1.0)

        return statistics


def main():
    log_path = "/home/gpal/workspace/spatiotemporal_planner/hil_logs/eka-rt-log"
    ps = LogParser(log_path)
    ps.run(STAT_RESULT_NAME, MAIN_KEYWORD, LOOP_KEYWORDS)


if __name__ == "__main__":
    main()
