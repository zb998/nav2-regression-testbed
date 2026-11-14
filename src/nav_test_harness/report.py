"""把测试数据，变成直观的图表、网页报告、以及方便机器读取的 JSON 数据。

@author: zbb
@date: 2026/09/30
@version: v1.0
"""

from __future__ import annotations
import html
import json
from pathlib import Path
from typing import Optional, Sequence


def plot_trajectory(actual, planned=None,
                    out_path='results/plots/trajectory.png'):
    try:
        import matplotlib
        # 使用 Agg 后端：无需图形界面，在后台静默生成图片文件（适合服务器/CI）
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        return None
    # 确保输出目录存在（如果 results/plots/ 不存在则自动创建）
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 6))
    if planned:
        ax.plot([p[0] for p in planned], [p[1] for p in planned],
                '--', color='gray', label='planned')
    if actual:
        ax.plot([p[0] for p in actual], [p[1] for p in actual],
                '-', color='tab:blue', label='actual')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    # 保存图片，120 DPI 清晰度
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return str(out_path)


def render_html(scenario, results, metrics=None, out_path='results/reports/report.html'):
    """生成一个独立的 HTML 报告文件，方便直接用浏览器查看"""
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = ''
    for r in results:
        # 使用 HTML 实体字符：&#10003; 是绿色对勾(✓)，&#10007; 是红色叉号(✗)
        mark = '&#10003;' if r.passed else '&#10007;'
        cls = 'pass' if r.passed else 'fail'
        rows += (f'<tr class="{cls}"><td>{mark}</td><td>{html.escape(r.name)}</td>'
                 f'<td>{html.escape(r.quality_property)}</td>'
                 f'<td>{html.escape(r.detail)}</td></tr>\n')
    mtr = ''
    if metrics is not None:
        rates = ', '.join(f'{t}={v:.2f}Hz' for t, v in (metrics.rates or {}).items())
        mtr = (f'<h3>Metrics</h3>'
               f'<p>rates: {html.escape(rates)}<br/>'
               f'peak cpu: {metrics.peak_cpu_pct}%  peak mem: {metrics.peak_mem_pct}%</p>')
    # 获取场景字典（如果 scenario 是对象就调用 to_dict，如果已经是字典就直接用）
    sc = scenario.to_dict() if hasattr(scenario, 'to_dict') else scenario
    doc = f"""<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<title>Scenario: {html.escape(sc['name'])}</title>
<style> body{{font-family:sans-serif;margin:2em}} table{{border-collapse:collapse}}
th,td{{border:1px solid #ccc;padding:6px 10px;text-align:left}}
tr.pass td:first-child{{color:#0a0}} tr.fail td:first-child{{color:#c00}} </style>
</head><body><h1>Scenario: {html.escape(sc['name'])}</h1>
<p>{html.escape(sc.get('description',''))}</p>
<p>sut: <code>{html.escape(sc['sut'])}</code></p>{mtr}
<h2>Assertions</h2>
<table><tr><th>ok</th><th>name</th><th>quality</th><th>detail</th></tr>{rows}</table>
</body></html>"""
    out.write_text(doc, encoding='utf-8')
    return str(out)


def write_results_json(scenario, results, metrics=None, out_path='results/report.json'):
    """将所有结果导出为 JSON 格式，方便后续自动化工具（如 CI/CD）解析"""
    data = {
        'scenario': scenario.to_dict() if hasattr(scenario, 'to_dict') else scenario,
        'results': [{'name': r.name, 'passed': r.passed,
                     'quality_property': r.quality_property, 'detail': r.detail}
                    for r in results],
        'metrics': None if metrics is None else {
            'rates': metrics.rates,
            'peak_cpu_pct': metrics.peak_cpu_pct,
            'peak_mem_pct': metrics.peak_mem_pct,
        },
    }
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    return str(out)