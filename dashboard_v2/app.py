# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.
from urllib.request import urlopen


def internet_on():
    try:
        urlopen('http://216.58.192.142', timeout=1)
        return True
    except:
        return False

if internet_on():
	bm = "stamen-terrain"
else:
	bm = "white-bg"
	

import datetime as dt

import dash
import dash_daq as daq
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import geopandas as gpd



import plotly.express as px

import os

from scripts.helpers import load_db3, gdf_builder, interkrige, rasterbuilder, html_points, html_raster, reducer

#Set file location names
rosbagname = "test9_April28"

db_loc = os.path.join(os.path.expanduser('~'), rosbagname , rosbagname+"_0.db3")

gdf = gdf_builder(load_db3(db_loc))



app_color = {"graph_bg": "#082255", "graph_line": "#007ACE"}


fig1 = px.line(gdf, y= 'tds', x = 'timestamp', width = 400, height =400, color_discrete_sequence = px.colors.sequential.turbid)


fig1.update_layout(
    plot_bgcolor=app_color['graph_bg'],
    paper_bgcolor=app_color['graph_bg'],
    font_color=app_color['graph_line'],
    annotations=[], overwrite=True
)
fig1.update_xaxes(showticklabels=False)

fig2 = px.line(gdf,y = ['ph_v','t_v'], x = "timestamp", width = 400, height =400, color_discrete_sequence = px.colors.sequential.Plasma)


fig2.update_layout(
    plot_bgcolor=app_color['graph_bg'],
    paper_bgcolor=app_color['graph_bg'],
    font_color=app_color['graph_line'],
    annotations=[], overwrite=True
    )
    
fig2.update_xaxes(showticklabels=False)


temp = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "temp",mapbox_style = bm, zoom = 18, width = 800, height =800)

temp.update_traces(marker_showscale = False, marker_colorbar_x = 0.9, selector=dict(type='scattermapbox'))
temp.update_layout(margin = dict(l = 0, r = 0, t = 0, b = 0))

app = dash.Dash(
    __name__,
    meta_tags=[{"name": "viewport", "content": "width=device-width, initial-scale=1"}],
)

server = app.server


app.layout = html.Div(
    [
        # header
        html.Div(
            [
                html.Div(
                    [
                        html.H4("WUR-USV Monitoring", className="app__header__title"),
                        html.P(
                            "This app continually queries a SQL database and displays live charts of the USV status.",
                            className="app__header__title--grey",
                        ),
                    ],
                    className="app__header__desc",
                ),
                html.Div(
                    [
                        html.Img(
                            src=app.get_asset_url("dash-new-logo.png"),
                            className="app__menu__img",
                        )
                    ],
                    className="app__header__logo",
                ),
            ],
            className="app__header",
        ),
        html.Div(
            [
                # point map
                html.Div(
                    [
                        html.Div(
                            [html.H6("LOCATION", className="graph__title")]
                        ),
                        dcc.Graph(
                            id="point-temp-map",
                            figure=dict(
                                layout=dict(
                                    plot_bgcolor=app_color["graph_bg"],
                                    paper_bgcolor=app_color["graph_bg"],
                                )
                            ),
                        ),
                        dcc.Interval(
                            id="point-temp-update",
                            interval=10000,
                            n_intervals=0,
                        ),
                    ],
                    className="one-two column wind__speed__container",
                ),
                html.Div(
                    [
                        # lineplot turb ph
                        html.Div(
                            [
                                html.Div(
                                    [
                                        html.H6(
                                            "TURB + PH VOLTAGES",
                                            className="graph__title",
                                        )
                                    ]
                                ),
                                                               
                                dcc.Graph(
                                    id="turb-ph-plot",
                                    figure=dict(
                                        layout=dict(
                                            plot_bgcolor=app_color["graph_bg"],
                                            paper_bgcolor=app_color["graph_bg"],
                                        )
                                    ),
                                ),
                                dcc.Interval(
                                    id="turb-ph-update",
                                    interval=6000,
                                    n_intervals=0,
                                ),
                            ],
                            className="graph__container first",
                        ),
                        # lineplot tds
                        html.Div(
                            [
                                html.Div(
                                    [
                                        html.H6(
                                            "TDS", className="graph__title"
                                        )
                                    ]
                                ),
                                dcc.Graph(
                                    id="tds-plot",
                                    figure=dict(
                                        layout=dict(
                                            plot_bgcolor=app_color["graph_bg"],
                                            paper_bgcolor=app_color["graph_bg"],
                                        )
                                    ),
                                ),
                                
                            ],
                            className="graph__container second",
                        ),
                    ],
                    className="three columns histogram__direction"
                    ),
                html.Div([    
                html.Div(
                id="card-1",
                children=[
                    html.P("pH"),
                    daq.LEDDisplay(
                        id="ph-led",
                        value="1704",
                        color="#92e0d3",
                        backgroundColor="#1e2130",
                        size=50,
                    )],
                    
                
                    
            ),
            html.Div(
                id="card-2",
                children=[
                    html.P("TDS"),
                    daq.LEDDisplay(
                        id="tds-led",
                        value="1704",
                        color="#92e0d3",
                        backgroundColor="#1e2130",
                        size=50,
                    ),
                    ],
                
                    
            ),
            html.Div(
                id="card-3",
                children=[
                    html.P("Turbidity"),
                    daq.LEDDisplay(
                        id="turb-led",
                        value="1704",
                        color="#92e0d3",
                        backgroundColor="#1e2130",
                        size=50,
                    ),
                    ],
                
                    
            ),
            html.Div(
                id="card-4",
                children=[
                    html.P("Temperature"),
                    daq.LEDDisplay(
                        id="temp-led",
                        value="1704",
                        color="#92e0d3",
                        backgroundColor="#1e2130",
                        size=50,
                    ),
                    ],
                
                    
            ),
            dcc.Interval(id="ph-b-update",
                                interval=5000,
                                n_intervals=0)],
                             
                className = "three columns buttons"),
   
            ],
            className="app__content",
        ),
    ],
    className="app__container",
)

def get_current_time():
    """ Helper function to get the current time in seconds. """

    now = dt.datetime.now()
    total_time = (now.hour * 3600) + (now.minute * 60) + (now.second)
    return total_time

#@app.callback(Output("intermediate-value", 'data'),
#             [Input('ph-b-update', 'n_intervals')])    

#def redo_data():
##    gdf = gdf_builder(load_db3(db_loc))
#    gdf.to_file('dataframe.geojson', driver='GeoJSON')  
#    return gdf.to_file('dataframe.geojson', driver='GeoJSON')  

@app.callback(Output("point-temp-map", 'figure'),
             [Input('point-temp-update', 'n_intervals')])

def update_plots(n):
    filename = "dataframe.geojson"
    gdf = gpd.read_file(open(filename))
    
    temp = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "temp",mapbox_style = bm, zoom = 18, width = 800, height =800)
    temp.update_traces(marker_showscale = False, marker_colorbar_x = 0.9, selector=dict(type='scattermapbox'))
    temp.update_layout(margin = dict(l = 0, r = 0, t = 0, b = 0))
    
    return temp


@app.callback(Output('turb-ph-plot', 'figure'),Output("tds-plot", "figure"),
              [Input('turb-ph-update', 'n_intervals')])


def update_plots(n):
    filename = "dataframe.geojson"
    gdf = gpd.read_file(open(filename))
    fig2 = px.line(gdf,y = ['ph_v','t_v'], x = "timestamp", width = 400, height =400, color_discrete_sequence = px.colors.qualitative.Pastel1)


    fig2.update_layout(
        plot_bgcolor=app_color['graph_bg'],
        paper_bgcolor=app_color['graph_bg'],
        font_color=app_color['graph_line'],
        annotations=[], overwrite=True
    )
    fig2.update_xaxes(showticklabels=False)
    
    fig1 = px.scatter(gdf, y= ['tds'], x = 'timestamp', width = 400, height =400, color_discrete_sequence = px.colors.qualitative.Pastel2)
    fig1.update_layout(
        plot_bgcolor=app_color['graph_bg'],
        paper_bgcolor=app_color['graph_bg'],
        font_color=app_color['graph_line'],
        annotations=[], overwrite=True
    )
    fig1.update_xaxes(showticklabels=False)

    
    return fig2, fig1


@app.callback(Output('ph-led',"value"),Output('tds-led', "value"),Output('turb-led', "value"),Output('temp-led', "value"),
    [Input("ph-b-update", "n_intervals")])
    
def update_buttons(n):
    loaddb3 = load_db3(db_loc)
    print("did db3 ", n, " times")
    gdf = gdf_builder(loaddb3)
    print("did gdf  ", n, " times")
    gdf.to_file('dataframe.geojson', driver='GeoJSON')
    ph = gdf["ph_v"].iloc[-1]
    tds = gdf["tds"].iloc[-1]
    turb = gdf["t_v"].iloc[-1]
    temp = gdf["temp"].iloc[-1]
    return f'{ph:.2f}', f'{tds:.2f}', f'{turb:.2f}', f'{temp:.2f}'

if __name__ == '__main__':
    app.run_server(debug=True)
