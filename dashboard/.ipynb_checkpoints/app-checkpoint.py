# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

import plotly.express as px

import os

from scripts.helpers import load_db3, gdf_builder, interkrige, rasterbuilder, html_points, html_raster

#Set file location names
rosbagname = "my_test_bag"
db_loc = os.path.join(os.path.expanduser('~'), rosbagname,rosbagname+"_0.db3")

gdf = gdf_builder(load_db3(db_loc, rosbagname))
fig = px.scatter(gdf, y="test", x="timestamp")

#app thingymajigz
app = dash.Dash(__name__)


app.layout = html.Div(children=[
                      html.Div(className='row', 
 # Define the row element
                               children=[
                                  html.Div(className='four columns div-user-controls',
                                           children = [
                                                html.H2('Striker v2 Progress Monitor'),
                                                html.P('''IMU measurements'''),
                                                dcc.Graph(id = "imu1", figure = fig),
                                                dcc.Interval(id='interval-plots', interval=1*2000, n_intervals=0)
                                           ]),# Every 2 sec, update plot
                                  html.Div(className='four columns div-for-charts bg-grey',
                                           children = [
                                               html.H2('''GPS Points acquired'''),
                                               html.Iframe(id = "map_points", srcDoc = open("./html_folium/points.html","r").read(),width = "100%", height = "100%"),
                                               dcc.Interval(id='interval-maps1', interval=1*5000, n_intervals=0)#every 5s update the points
                                           ]),  # Define the middle elements
                                    html.Div(className='four columns div-for-charts bg-grey',
                                           children = [
                                                html.P('''TDS'''),
                                               html.Iframe(id = "map_raster1", srcDoc = open("./html_folium/test_raster.html","r").read(),width = "100%", height = "25%"),
                                               html.P('''Turbidity'''),
                                               html.Iframe(id = "map_raster2", srcDoc = open("./html_folium/test_raster.html","r").read(),width = "100%", height = "25%"),
                                               html.P('''Temperature'''),
                                               html.Iframe(id = "map_raster3", srcDoc = open("./html_folium/test_raster.html","r").read(),width = "100%", height = "25%"),
                                               html.P('''pH'''),
                                               html.Iframe(id = "map_raster4", srcDoc = open("./html_folium/test_raster.html","r").read(),width = "100%", height = "25%"),
                                               dcc.Interval(id='interval-maps2', interval=1*10000, n_intervals=0)#every 10s update the rasters
                                           ])  # Define the right element
                                  
                                        ])
                                ])


@app.callback(Output("imu1", 'figure'),
             [Input('interval-plots', 'n_intervals')])

def update_plots(n):
    gdf = gdf_builder(load_db3(db_loc, rosbagname))
    fig = px.scatter(gdf, y="test", x="timestamp")
    return fig


@app.callback(Output('map_points', 'srcDoc'),

              [Input('interval-maps1', 'n_intervals')])


def update_maps(n):
    gdf = gdf_builder(load_db3(db_loc, rosbagname))
        
    p_html = html_points(gdf = gdf, zoomlvl = 16, out = "points")
   
    return open('./' + p_html, 'r').read()


@app.callback(Output('map_raster1', 'srcDoc'),
              Output('map_raster2', 'srcDoc'),
              Output('map_raster3', 'srcDoc'),
              Output('map_raster4', 'srcDoc'),        
              [Input('interval-maps2', 'n_intervals')])

def update_maps(n):
    gdf = gdf_builder(load_db3(db_loc, rosbagname))
    
    tds_z, tds_SS,x,y = interkrige(gdf["lon"], gdf["lat"], gdf["test"], res=int(round(len(gdf)/10,0)))
        
    r1_html = html_raster(gdf = gdf, zoomlvl = 16, rasterloc = rasterbuilder(tds_z, x,y,"TDS"), colormap = "Green", out = "Temp")
    
    r2_html = html_raster(gdf = gdf, zoomlvl = 16, rasterloc = rasterbuilder(tds_z, x,y,"Turbidity"), colormap = "Blue", out = "Turb")
    
    r3_html = html_raster(gdf = gdf, zoomlvl = 16, rasterloc = rasterbuilder(tds_z, x,y,"ph"), colormap = "Red", out = "ph")
    
    r4_html = html_raster(gdf = gdf, zoomlvl = 16, rasterloc = rasterbuilder(tds_z, x,y,"Temp"), colormap = "Grey", out = "Temp")
    
    return open('./' + r1_html, 'r').read(), open('./' + r2_html, 'r').read(), open('./' + r3_html, 'r').read(), open('./' + r4_html, 'r').read()



if __name__ == '__main__':
    app.run_server(debug=True)
