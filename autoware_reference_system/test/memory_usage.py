# Memory usage
#
# Get trace data using the provided launch file:
#    $ ros2 launch tracetools_analysis memory_usage.launch.py
#    (wait a few seconds, then kill with Ctrl+C)
#
# (optional) convert trace data:
#    $ ros2 trace-analysis convert ~/.ros/tracing/memory-usage  path = '~/.ros/tracing/memory-usage'  import sys
# Assuming a workspace with:
#   src/tracetools_analysis/
#   src/ros-tracing/ros2_tracing/tracetools_read/
sys.path.insert(0, '../')
sys.path.insert(0, '../../../ros-tracing/ros2_tracing/tracetools_read/')
import datetime as dt

from bokeh.palettes import viridis
from bokeh.plotting import figure
from bokeh.plotting import output_notebook
from bokeh.io import show
from bokeh.models import ColumnDataSource
from bokeh.models import DatetimeTickFormatter
from bokeh.models import NumeralTickFormatter
import numpy as np
import pandas as pd

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor import Processor
from tracetools_analysis.processor.memory_usage import KernelMemoryUsageHandler
from tracetools_analysis.processor.memory_usage import UserspaceMemoryUsageHandler
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.memory_usage import MemoryUsageDataModelUtil
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil  # Process
events = load_file(path)
ust_memory_handler = UserspaceMemoryUsageHandler()
kernel_memory_handler = KernelMemoryUsageHandler()
ros2_handler = Ros2Handler()
Processor(ust_memory_handler, kernel_memory_handler, ros2_handler).process(events)  memory_data_util = MemoryUsageDataModelUtil(
    userspace=ust_memory_handler.data,
    kernel=kernel_memory_handler.data,
)
ros2_data_util = Ros2DataModelUtil(ros2_handler.data)

output_notebook()
psize = 650  # Plot memory usage
ust_memory_usage_dfs = memory_data_util.get_absolute_userspace_memory_usage_by_tid()
kernel_memory_usage_dfs = memory_data_util.get_absolute_kernel_memory_usage_by_tid()
tids = ros2_data_util.get_tids()

colours = viridis(len(tids) + 1)
first_tid = min(tids)
starttime = ust_memory_usage_dfs[first_tid].loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')
memory = figure(
    title='Memory usage per thread/node',
    x_axis_label=f'time ({starttime})',
    y_axis_label='memory usage',
    plot_width=psize, plot_height=psize,
)

i_colour = 0
for tid in tids:
    legend = str(tid) + ' ' + str(ros2_data_util.get_node_names_from_tid(tid))
    # Userspace
    memory.line(
        x='timestamp',
        y='memory_usage',
        legend=legend + ' (ust)',
        line_width=2,
        source=ColumnDataSource(ust_memory_usage_dfs[tid]),
        line_color=colours[i_colour],
    )
    # Kernel
    memory.line(
        x='timestamp',
        y='memory_usage',
        legend=legend + ' (kernel)',
        line_width=2,
        source=ColumnDataSource(kernel_memory_usage_dfs[tid]),
        line_color=colours[i_colour],
        line_dash='dotted',
    )
    i_colour += 1

memory.title.align = 'center'
memory.legend.label_text_font_size = '11px'
memory.xaxis[0].formatter = DatetimeTickFormatter(seconds=['%Ss'])
memory.yaxis[0].formatter = NumeralTickFormatter(format='0.0b')

show(memory