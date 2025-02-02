# Copyright 2021 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from bokeh.models import ColumnDataSource
from bokeh.models.ranges import FactorRange
from bokeh.models.tools import HoverTool
from bokeh.models.widgets.markups import Div
from bokeh.models.widgets.tables import DataTable, TableColumn
from bokeh.palettes import cividis
from bokeh.plotting import figure
from bokeh.transform import factor_cmap
from constants import SIZE_AXIS_LABEL, SIZE_CATEGORY_LABEL, SIZE_MAJOR_LABEL, SIZE_TITLE
from constants import SIZE_TABLE_ROW, SIZE_TABLE_WIDTH
import pandas as pd


def summary(path, duration, size):
    data, test_name, hot_path_name = parseLogSummary(path, duration)
    x = []
    all_data = {
        'exe': [],
        'rmw': [],
        'type': [],
        'low': [],
        'mean': [],
        'high': [],
        'std_dev': [],
        'top': [],
        'bottom': []
    }
    for results in data:
        exe = results[0]
        rmw = results[1]
        for data_type in results[2]['hot_path']:
            all_data['exe'].append(exe)
            all_data['rmw'].append(rmw)
            all_data['type'].append(data_type)
            all_data['low'].append(results[2]['hot_path'][data_type]['low'])
            all_data['mean'].append(results[2]['hot_path'][data_type]['mean'])
            all_data['high'].append(results[2]['hot_path'][data_type]['high'])
            all_data['std_dev'].append(results[2]['hot_path'][data_type]['std_dev'])
            all_data['top'].append(results[2]['hot_path'][data_type]['top'])
            all_data['bottom'].append(results[2]['hot_path'][data_type]['bottom'])
        # add behavior planner data to dataframe
        cyclic_node = 'behavior_planner'
        data_type = 'period'
        all_data['exe'].append(exe)
        all_data['rmw'].append(rmw)
        all_data['type'].append(data_type)
        all_data['low'].append(results[2][cyclic_node][data_type]['low'])
        all_data['mean'].append(results[2][cyclic_node][data_type]['mean'])
        all_data['high'].append(results[2][cyclic_node][data_type]['high'])
        all_data['std_dev'].append(results[2][cyclic_node][data_type]['std_dev'])
        all_data['top'].append(results[2][cyclic_node][data_type]['top'])
        all_data['bottom'].append(results[2][cyclic_node][data_type]['bottom'])
    df = pd.DataFrame.from_records(
        all_data, columns=[
            'exe', 'rmw', 'type', 'low', 'mean', 'high', 'top', 'bottom', 'std_dev'])
    # sort by exe and rmw
    df = df.sort_values(['exe', 'rmw'], ascending=True)
    exes = []
    rmws = []
    for exe in df.exe:
        for rmw in df.rmw:
            # add exe and rmw to list
            if exe not in exes:
                exes.append(exe)
            if rmw not in rmws:
                rmws.append(rmw)
    for exe in exes:
        for rmw in rmws:
            x.append((exe, rmw))
    latency = df.type == 'latency'
    dropped = df.type == 'dropped'
    period = df.type == 'period'
    latency_source = ColumnDataSource(df[latency])
    dropped_source = ColumnDataSource(df[dropped])
    period_source = ColumnDataSource(df[period])
    # add exe and rmw list of tuples for x axis
    latency_source.data['x'] = x
    dropped_source.data['x'] = x
    period_source.data['x'] = x
    # initialize list of figures
    std_figs = []
    # initialize latency figure
    test_info = str(duration) + 's [' + hot_path_name + ']'
    latency_fig = figure(
        title='Latency Summary ' + test_info,
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Average Latency (ms)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    latency_fig.segment(
        x, df.high[latency].values, x, df.low[latency].values, color='black', line_width=2)
    latency_fig.vbar(
        width=0.2,
        x='x',
        top='top',
        bottom='bottom',
        line_color='black',
        line_width=1,
        source=latency_source,
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    latency_fig.scatter(
        size=25,
        x='x',
        y='high',
        source=latency_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    latency_fig.scatter(
        size=25,
        x='x',
        y='low',
        source=latency_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    latency_fig.y_range.start = 0
    latency_fig.x_range.range_padding = 0.1
    latency_fig.title.text_font_size = SIZE_TITLE
    latency_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    latency_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    latency_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    latency_fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    latency_fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL

    # initialize dropped message figure
    dropped_fig = figure(
        title='Dropped Messages Summary ' + test_info,
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Dropped Messages',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    dropped_fig.segment(
        x, df.high[dropped].values, x, df.low[dropped].values, color='black', line_width=2)
    dropped_fig.vbar(
        width=0.2,
        x='x',
        top='mean',
        source=dropped_source,
        line_color='black',
        line_width=1,
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    dropped_fig.scatter(
        size=25,
        x='x',
        y='high',
        source=dropped_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    dropped_fig.y_range.start = 0
    dropped_fig.x_range.range_padding = 0.1
    dropped_fig.title.text_font_size = SIZE_TITLE
    dropped_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    dropped_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    dropped_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    dropped_fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    dropped_fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL

    # initialize period figure
    period_fig = figure(
        title='Behavior Planner Jitter Summary ' + str(duration) + 's',
        x_axis_label=f'Executors (with RMW)',
        y_axis_label='Period (ms)',
        x_range=FactorRange(*x),
        plot_width=int(size * 2.0),
        plot_height=size,
        margin=(10, 10, 10, 10)
    )
    period_fig.segment(
        x, df.high[period].values, x, df.low[period].values, color='black', line_width=2)
    period_fig.vbar(
        width=0.2,
        x='x',
        top='top',
        bottom='bottom',
        source=period_source,
        line_color='black',
        line_width=1,
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    period_fig.scatter(
        size=25,
        x='x',
        y='high',
        source=period_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    period_fig.scatter(
        size=25,
        x='x',
        y='low',
        source=period_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color=factor_cmap(
            'x', palette=cividis(len(rmws)), factors=list(rmws), start=1, end=2)
    )
    period_fig.y_range.start = 0
    period_fig.x_range.range_padding = 0.1
    period_fig.title.text_font_size = SIZE_TITLE
    period_fig.xaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    period_fig.yaxis.axis_label_text_font_size = SIZE_AXIS_LABEL
    period_fig.yaxis.major_label_text_font_size = SIZE_MAJOR_LABEL
    period_fig.below[0].group_text_font_size = SIZE_CATEGORY_LABEL
    period_fig.below[0].major_label_text_font_size = SIZE_MAJOR_LABEL

    # add latency hover tool
    latency_hover = HoverTool()
    latency_hover.tooltips = [
        ('Average Latency', '@{mean}{0.00}ms'),
        ('Minimum Latency', '@{low}{0.00}ms'),
        ('Maximum Latency', '@{high}{0.00}ms'),
    ]
    latency_fig.add_tools(latency_hover)
    # add dropped messages hover tool
    dropped_hover = HoverTool()
    dropped_hover.tooltips = [
        ('Dropped Messages Average', '@{mean}{0.00}'),
        ('Dropped Messages Min', '@{low}{0.00}'),
        ('Dropped Messages Max', '@{high}{0.00}'),
    ]
    dropped_fig.add_tools(dropped_hover)
    # add period hover tool
    period_hover = HoverTool()
    period_hover.tooltips = [
        ('Period Average', '@{mean}{0.00}ms'),
        ('Period Min', '@{low}{0.00}ms'),
        ('Period Max', '@{high}{0.00}ms'),
    ]
    period_fig.add_tools(period_hover)

    # add tables
    # create tables
    columns = [TableColumn(field=col, title=col) for col in df[latency].columns]
    latency_table_title = Div(
        text='<b>Latency Summary Table ' + test_info + '</b>',
        width=SIZE_TABLE_WIDTH,
        height=SIZE_TABLE_ROW)
    latency_table = [
        latency_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df[latency].round(decimals=3)),
            autosize_mode='fit_viewport',
            margin=(0, 10, 10, 10),
            height=(len(df[latency].exe.values.tolist()) * SIZE_TABLE_ROW),
            width=SIZE_TABLE_WIDTH)]
    dropped_table_title = Div(
        text='<b>Dropped Messages Summary Table ' + test_info + '</b>',
        width=SIZE_TABLE_WIDTH,
        height=SIZE_TABLE_ROW)
    dropped_table = [
        dropped_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df[dropped].round(decimals=1)),
            autosize_mode='fit_viewport',
            margin=(0, 10, 10, 10),
            height=(len(df[dropped].exe.values.tolist()) * SIZE_TABLE_ROW),
            width=SIZE_TABLE_WIDTH)]
    period_table_title = Div(
        text='<b>Behavior Planner Jitter Summary Table ' + str(duration) + 's</b>',
        width=SIZE_TABLE_WIDTH,
        height=SIZE_TABLE_ROW)
    period_table = [
        period_table_title,
        DataTable(
            columns=columns,
            source=ColumnDataSource(df[period].round(decimals=3)),
            autosize_mode='fit_viewport',
            margin=(0, 10, 10, 10),
            height=(len(df[period].exe.values.tolist()) * SIZE_TABLE_ROW),
            width=SIZE_TABLE_WIDTH)]

    std_figs = [
        [latency_table], [latency_fig],
        [dropped_table], [dropped_fig],
        [period_table], [period_fig]]
    return std_figs


def parseLogSummary(path, duration):
    # open file
    data = open(path).read().splitlines()
    # hold info on each test (start line and end line)
    log_map = {}
    in_test = False
    test_name = ''
    hot_path_name = ''
    log_map = []
    count = 0
    for index, line in enumerate(data):
        if 'generate_std_trace' in line:
            if str(duration) in line:
                if 'Start' in line:
                    search = ': generate_std_traces_'
                    test_name = line[line.find(search) + len(search):line.find('.py')]
                    rmw_idx = test_name.find('_rmw')
                    exe = test_name[0:rmw_idx]
                    rmw = test_name[rmw_idx + 1:test_name.rfind('_')]
                    start_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map.append(
                        [exe, rmw, {
                            'start': start_time,
                            'end': 0,
                            'hot_path': {
                                'latency': {},
                                'dropped': {}
                            },
                            'behavior_planner': {
                                'period': {}
                            }
                        }])
                    in_test = True
                elif 'Passed' in line:
                    in_test = False
                    count += 1
        # if within a test, add parse current line to dataframe
        if in_test:
            if 'hot path' in line:
                if 'hot path:' in line:
                    indent = '                 '
                    hot_path_name = line[line.find('hot path:') + len('hot path:' + indent):]
                if 'latency' in line:
                    end_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map[count][2]['end'] = end_time
                    stats = parseStats(line)
                    if stats is not None:
                        log_map[count][2]['hot_path']['latency'] = stats
                elif 'drops' in line:
                    end_time = line[line.find('[') + 1:line.find(']') - 1]
                    log_map[count][2]['end'] = end_time
                    stats = parseStats(line)
                    if stats is not None:
                        log_map[count][2]['hot_path']['dropped'] = stats
            elif 'period' in line:
                # behavior planner period
                # log_map[count][2]['behavior_planner']['period'] = \
                #    float(line[line.find('period') + len('period:  '):line.find('ms')])
                stats = parseStats(line)
                if stats is not None:
                    log_map[count][2]['behavior_planner']['period'] = stats
    return log_map, test_name, hot_path_name


def parseStats(line):
    # assume all stats in milliseconds
    stats = {
        'timestamp': 0.0,
        'low': 0.0,
        'high': 0.0,
        'mean': 0.0,
        'std_dev': 0.0,
        'top': 0.0,
        'bottom': 0.0
    }
    try:
        stats['timestamp'] = float(
            line[line.find('[') + 1:line.find(']') - 1]) * 1000  # convert to ms
        parsed_stats = line.split(',')
        stats['low'] = parsed_stats[0][parsed_stats[0].find('min') + len('min='):]
        stats['high'] = parsed_stats[1][parsed_stats[1].find('max') + len('max='):]
        stats['mean'] = parsed_stats[2][parsed_stats[2].find('average') + len('average='):]
        stats['std_dev'] = \
            parsed_stats[3][parsed_stats[3].find('deviation') + len('deviation='):-1]
        for val in stats:
            if isinstance(stats[val], str):
                if stats[val].endswith('ms'):
                    stats[val] = stats[val][:-2]
            stats[val] = float(stats[val])
        stats['top'] = stats['mean'] + stats['std_dev']
        stats['bottom'] = stats['mean'] - stats['std_dev']
        if stats['bottom'] < 0:
            stats['bottom'] = 0
    except IndexError as e:
        print('Line incomplete:')
        print('[line]: ' + line)
        print('Returning None for this line')
        print(e)
        stats = None
    return stats
