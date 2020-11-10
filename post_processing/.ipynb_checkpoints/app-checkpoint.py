# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px


external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)


app.layout = html.Div(children=[
    html.H1(children='USV - progress Monitoring'),

    html.Div(children='''
        Testing the possibilites
    '''),

    html.Iframe(id = 'map', srcDoc = open("./html_folium/test.html","r").read(), width = "600", height = "600")
    
])

if __name__ == '__main__':
    app.run_server(debug=True)
